/* Deterministic whole-region counterfactual model. No live control. */
(function(root){
  'use strict';
  function capture({time,region,scenario,streams,objects,vehicles,states,config},horizon=600){
    if(!Number.isInteger(horizon)||horizon<60||horizon>900)throw Error('推演时长须为60–900秒');
    return captureInput({time,region,scenario,streams,objects,vehicles,states,config},horizon);
  }
  // Runtime models schedule the known local demand for the whole replay, without reading future observations.
  function captureRuntime(args){return captureInput(args,args.config.duration);}
  function captureInput({time,region,scenario,streams,objects,vehicles,states,config},horizon){
    const facilities=Object.fromEntries([...objects.values()].map(o=>[o.id,{id:o.id,kind:o.kind,l:o.l??null,p:o.p??null,reserve:!!o.reserve,capacity:o.capacity,open:states[o.id].status==='开放',state:{...states[o.id]}}]));
    const groups=Object.fromEntries([...objects.values()].filter(o=>o.kind==='pipe'&&!o.reserve).map(o=>[o.id,[...objects.values()].filter(p=>p.kind==='pipe'&&p.l===o.l&&Math.floor(p.p/5)===Math.floor(o.p/5)).map(p=>p.id)]));
    const paths={},jobs=[],priorExits=[],end=time+horizon;
    function path(key,g){if(!paths[key]){const samples=g.samples.map(p=>[...p]),lengths=[0];for(let i=1;i<samples.length;i++)lengths.push(lengths[i-1]+Math.hypot(...samples[i].map((v,j)=>v-samples[i-1][j])));paths[key]={samples,lengths,length:g.length};}return key;}
    for(const o of objects.values())if(o.kind==='pipe')path('object:'+o.id,o);
    for(const s of streams){
      const segments=s.segments.map((seg,i)=>({id:seg.id,stage:seg.stage,length:seg.g.length,speed:seg.speed,path:path(s.id+':'+i,seg.g)}));
      const completionLayer=[...segments].reverse().map(seg=>facilities[seg.id]).find(o=>o.kind==='pipe')?.l??null;
      for(const e of s.events){const first=Math.max(0,Math.ceil((time-60-s.start-e.offset)/s.interval)),last=Math.min(Math.ceil((time-s.start-e.offset)/s.interval)-1,Math.ceil((s.end-s.start)/s.interval)-1);for(let k=first;k<=last;k++)priorExits.push({id:e.id,time:s.start+k*s.interval+e.offset});}
      const first=Math.max(0,Math.floor((time-s.start-s.duration)/s.interval)+1),last=Math.min(Math.floor((end-s.start)/s.interval),Math.ceil((s.end-s.start)/s.interval)-1);
      for(let k=first;k<=last;k++){
        const start=s.start+k*s.interval,age=time-start;if(age>=s.duration)continue;
        const index=age<0?0:s.segments.findIndex(seg=>age<seg.end);
        jobs.push({id:`R${String(region+1).padStart(2,'0')}:${s.id}:${k}`,stream:s.id,k,start,index,distance:age<0?0:(age-s.segments[index].offset)*s.segments[index].speed,
          initial:start<=time,divertible:s.id.startsWith('B-')&&!!groups[s.segments[0].id],completionLayer,segments});
      }
    }
    jobs.sort((a,b)=>a.start-b.start||a.id.localeCompare(b.id));
    const known=new Set(vehicles.map(v=>v.id));if(jobs.filter(j=>j.initial).length!==known.size||jobs.some(j=>j.initial&&!known.has(j.id)))throw Error('快照个体与任务输入不一致');
    return {version:config.version,time,horizon,region,scenario,facilities,groups,paths,priorExits,speedLimit:time<1800?18:14,initialCount:known.size,initialVehicles:vehicles.map(v=>({...v,position:[...v.position],ahead:[...v.ahead]})),jobs};
  }
  function point(path,distance){
    const {samples,lengths}=path,total=lengths.at(-1),s=Math.max(0,Math.min(1,distance/(path.length||1)))*total;let a=0,b=lengths.length-1;
    while(a+1<b){const m=(a+b)>>1;if(lengths[m]<s)a=m;else b=m;}const u=(s-lengths[a])/(lengths[b]-lengths[a]||1);return samples[a].map((v,i)=>v+(samples[b][i]-v)*u);
  }
  function createRunner(input,openReserve,reserveLayers=null,{onFrame=null,retainFrames=true,summaryOnly=false,sampleStep=5}={}){
    if(!Number.isInteger(sampleStep)||sampleStep<1)throw Error('采样步长须为正整数秒');
    const facilities=input.facilities,ids=Object.keys(facilities),jobs=input.jobs.map(j=>({...j,segments:j.segments.map(s=>({...s})),active:j.initial,done:false,waiting:false,wait:0,speed:j.segments[j.index].speed}));
    const isOpen=id=>facilities[id].open||openReserve&&facilities[id].reserve&&(!reserveLayers||reserveLayers.includes(facilities[id].l)),controlled=id=>facilities[id].capacity!=null;
    const frames=[],finished=[],completedLayers=Array(6).fill(0),exits=Object.fromEntries(ids.map(id=>[id,[]]));
    for(const e of input.priorExits||[])exits[e.id].push(e.time);
    const order=new Map(jobs.map((j,i)=>[j,i])),pending=jobs.filter(j=>!j.active).sort((a,b)=>a.start-b.start||order.get(a)-order.get(b));
    let active=jobs.filter(j=>j.active),nextJob=0,elapsed=0,completed=0,totalWait=0,highLoadSeconds=0,peakQueue=0,diverted=0,incoming=0,outgoing=0,arrived=active.length;
    const n=Object.fromEntries(ids.map(id=>[id,0]));for(const j of active)n[j.segments[j.index].id]++;
    const summarySpeeds=Object.fromEntries(ids.map(id=>[id,0]));
    function save(elapsed,n){
      if(summaryOnly){for(const id of ids)summarySpeeds[id]=0;let queue=0;for(const j of active){if(j.waiting)queue++;else summarySpeeds[j.segments[j.index].id]+=j.speed;}
        let count=0,speedCount=0,weighted=0,valid=0;for(const id of ids){if(elapsed===0&&facilities[id].state?.missing)continue;valid++;count+=n[id];if(n[id]){speedCount+=n[id];weighted+=(summarySpeeds[id]/n[id])*n[id];}}
        const frame={elapsed,t:input.time+elapsed,meanWait:arrived?totalWait/arrived:0,queue,highLoadSeconds,diverted,n:valid?count:null,speed:speedCount?weighted/speedCount:null};frames.length=0;frames.push(frame);if(onFrame)onFrame(frame);return;}
      const positions=[],speeds=Object.fromEntries(ids.map(id=>[id,0])),queues=Object.fromEntries(ids.map(id=>[id,0])),layers=Array.from({length:6},(_,l)=>({l,n:0,queue:0,completed:completedLayers[l]}));
      let queue=0;for(const j of active){const s=j.segments[j.index];if(!summaryOnly)positions.push({id:j.id,stream:j.stream,k:j.k,objectId:s.id,stage:s.stage,path:s.path,distance:j.distance,speed:j.waiting?0:j.speed,waiting:j.waiting});if(j.waiting){queue++;queues[s.id]++;const l=facilities[s.id].l;if(l!=null)layers[l].queue++;}else speeds[s.id]+=j.speed;}
      const states=Object.fromEntries(ids.map(id=>{const f=facilities[id],initial=elapsed===0,missing=initial&&f.state?.missing,open=initial?f.open:isOpen(id),status=open?'开放':n[id]?'排空':'关闭';if(f.l!=null)layers[f.l].n+=n[id];const time=input.time+elapsed;exits[id]=exits[id].filter(t=>t>=time-60);return [id,{...f.state,id,n:missing?null:n[id],visible:n[id],speed:missing||!n[id]?null:speeds[id]/n[id],flow:initial?f.state?.flow:exits[id].filter(t=>t<time).length,density:f.kind==='pipe'?(missing?null:n[id]/(input.paths['object:'+id].length/1000)):null,u:missing||!f.capacity?null:Math.min(1,Math.max(0,n[id]/f.capacity)),capacity:f.capacity,status,missing,queue:queues[id],speedLimit:input.speedLimit,sampleTime:new Date(Date.parse('2026-10-20T10:00:00+08:00')+time*1000).toISOString()}];}));
      const frame={elapsed,completed,queue,arrived,unfinished:arrived-completed,meanWait:arrived?totalWait/arrived:0,highLoadSeconds,diverted,incoming,outgoing,positions,states,layers};
      if(!retainFrames)frames.length=0;frames.push(frame);if(onFrame)onFrame(frame);
    }
    save(0,n);
    function advance(to){
      if(!Number.isInteger(to)||to<elapsed||to>input.horizon)throw Error('运行时刻须单调推进且位于模拟时域内');
      if(to>elapsed&&retainFrames&&!summaryOnly&&frames.at(-1).elapsed%sampleStep!==0)frames.pop();
      for(;elapsed<to;elapsed++){
      const now=input.time+elapsed;highLoadSeconds+=ids.filter(id=>facilities[id].kind==='pipe'&&n[id]>=facilities[id].capacity*.85).length;
      let reorder=false;while(nextJob<pending.length&&pending[nextJob].start<=now+1){const job=pending[nextJob++];if(active.length&&order.get(active.at(-1))>order.get(job))reorder=true;active.push(job);}if(reorder)active.sort((a,b)=>order.get(a)-order.get(b));
      for(const j of active){
        let remaining=1;
        if(!j.active){
          j.active=true;arrived++;j.waiting=true;remaining=Math.min(1,now+1-j.start);
          if(j.divertible){const seg=j.segments[0],choices=input.groups[seg.id].filter(isOpen).sort((a,b)=>n[a]/facilities[a].capacity-n[b]/facilities[b].capacity||a.localeCompare(b));if(choices.length&&choices[0]!==seg.id){seg.id=choices[0];seg.path='object:'+seg.id;seg.length=input.paths[seg.path].length;diverted++;}}
        }
        if(j.done)continue;
        while(remaining>1e-9&&!j.done){
          const seg=j.segments[j.index],f=facilities[seg.id];
          if(j.waiting){if(controlled(seg.id)&&(!isOpen(seg.id)||n[seg.id]>=f.capacity)){j.wait+=remaining;totalWait+=remaining;j.speed=0;break;}j.waiting=false;n[seg.id]++;}
          const load=controlled(seg.id)?n[seg.id]/f.capacity:0,speed=Math.min(seg.speed,input.speedLimit)/(1+Math.max(0,load-.7)*1.8),travel=(seg.length-j.distance)/speed;j.speed=speed;
          if(travel>remaining){j.distance+=speed*remaining;remaining=0;}
          else{remaining-=Math.max(0,travel);n[seg.id]--;const exit=now+1-remaining;if(!summaryOnly)exits[seg.id].push(exit);if(seg.stage==='汇入')incoming++;if(seg.stage==='降落')outgoing++;j.index++;j.distance=0;
            if(j.index===j.segments.length){j.done=true;completed++;if(j.completionLayer!=null)completedLayers[j.completionLayer]++;finished.push({id:j.id,elapsed:exit-input.time,intervalTime:exit-Math.max(input.time,j.start),wait:j.wait});}else j.waiting=true;
          }
        }
      }
      let waiting=0;active=active.filter(j=>{if(!j.done&&j.waiting)waiting++;return !j.done;});peakQueue=Math.max(peakQueue,waiting);
      if((elapsed+1)%sampleStep===0)save(elapsed+1,n);
    }
    if(frames.at(-1).elapsed!==elapsed)save(elapsed,n);
    return {openReserve,frames,finished,metrics:{completed,arrived,unfinished:arrived-completed,diverted,totalWait,meanWait:arrived?totalWait/arrived:0,meanIntervalTime:finished.length?finished.reduce((n,j)=>n+j.intervalTime,0)/finished.length:null,highLoadSeconds,peakQueue},final:frames.at(-1).positions};
    }
    return {advance};
  }
  function simulate(input,openReserve,reserveLayers=null){return createRunner(input,openReserve,reserveLayers).advance(input.horizon);}
  function snapshot(input,branch,index){const f=branch.frames[index],vehicles=f.elapsed===0&&input.initialVehicles?input.initialVehicles:f.positions.map(p=>{const path=input.paths[p.path];return {...p,position:point(path,p.distance),ahead:point(path,Math.min(path.length,p.distance+2))};});return {t:input.time+f.elapsed,vehicles,states:new Map(Object.entries(f.states)),completion:{in:f.incoming,out:f.outgoing,completed:f.completed},frame:f};}
  function compare(input,policy={id:"all",label:"开放全部备用",layers:null}){if(!["baseline","all","layer"].includes(policy.id)||policy.id==="layer"&&(!Array.isArray(policy.layers)||policy.layers.length!==1||!Number.isInteger(policy.layers[0])||policy.layers[0]<0||policy.layers[0]>5))throw Error("无效策略");return {input,policy,baseline:simulate(input,false),strategy:simulate(input,policy.id!=="baseline",policy.id==="layer"?policy.layers:null)};}
  const api={capture,captureRuntime,createRunner,simulate,compare,snapshot,point};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.BranchSimulation=api;
})(typeof window==='undefined'?globalThis:window);
