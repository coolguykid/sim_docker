/* Deterministic macro inventory model, not individual flight or collision simulation. */
(function(root){
  'use strict';
  const M=typeof module!=='undefined'&&module.exports?require('./center-model.js'):root.CenterModel;
  const policies={baseline:'维持现状',balance:'域间均衡导引',reserve:'均衡＋备用容量'},step=5;
  function capture(time,scenario='peak',horizon=600,quality='normal'){
    if(![300,600,900].includes(horizon))throw Error('推演时域须为5、10或15分钟');
    return buildInput(time,scenario,horizon,quality);
  }
  function buildInput(time,scenario,horizon,quality){
    if(!Number.isFinite(time)||time<0||time>60)throw Error('起报时刻须为0–60分钟');
    if(!['normal','peak','reserve'].includes(scenario))throw Error('未知模拟场景');
    const cells=M.state(time,scenario,quality).cells.map(c=>({...c})),topology=M.topology(time,quality),carry=cells.map(()=>0);
    // Future demand is an exogenous fixture from the captured capacities, not future observations.
    const demands=Array.from({length:horizon/step},(_,k)=>cells.map((c,i)=>{
      if(c.missing)return 0;
      const phase=M.seed(i+191)*Math.PI*2,rate=c.capacity*22/(c.length*1000)*(.66+.14*M.seed(i+72));
      carry[i]+=rate*step*(1+.12*Math.sin(phase+k/19));const count=Math.floor(carry[i]);carry[i]-=count;return count;
    }));
    return {time,horizon,step,scenario,quality,source:'MOCK-CENTER-MACRO-01',id:'CENTER-'+[time,scenario,quality,horizon].join('-'),cells,topology,demands,initialCount:cells.reduce((sum,c)=>sum+(c.n??0),0)};
  }
  function createRunner(input,policy,metricsOnly=false){
    const base=input.cells,valid=base.map(c=>!c.missing&&c.n!==null),n=base.map(c=>c.n??0),queue=base.map(()=>0),carry=base.map(()=>0);
    const neighbors=base.map(()=>[]),indices=new Map(input.topology.regions.map(r=>[r.id,r.index]));
    for(const edge of input.topology.connections)if(edge.open){const a=indices.get(edge.from),b=indices.get(edge.to);for(let l=0;l<6;l++){neighbors[a*6+l].push(b*6+l);neighbors[b*6+l].push(a*6+l);}}
    let completed=0,arrived=input.initialCount,totalWait=0,highLoadSeconds=0,diverted=0,k=0,current;
    function save(elapsed,cells,transfers){
      const aggregate=M.aggregate(cells);
      current={elapsed,time:input.time+elapsed/60,...(metricsOnly?{}:{cells,transfers}),metrics:{meanWait:arrived?totalWait/arrived:0,queue:queue.reduce((a,b)=>a+b,0),highLoadSeconds,diverted,n:aggregate.n,speed:aggregate.speed,completed,arrived,totalWait}};
    }
    save(0,base.map(c=>({...c})),[]);
    function advance(){
      if(k>=input.horizon/input.step)return null;
      const dt=input.step,capacity=base.map((c,i)=>valid[i]?Math.floor(c.capacity*(policy==='reserve'?1.2:1)):0),transfers=[];
      for(let i=0;i<base.length;i++)if(valid[i]){
        if(n[i]/capacity[i]>=.85)highLoadSeconds+=dt;
        totalWait+=queue[i]*dt;
        const speed=M.clamp(22*(1-.72*Math.min(1,n[i]/capacity[i])**2),3,24);
        carry[i]+=n[i]*speed*dt/(base[i].length*1000);
        const exits=Math.min(n[i],Math.floor(carry[i]));carry[i]-=exits;n[i]-=exits;completed+=exits;
        const admitted=Math.min(queue[i],Math.max(0,capacity[i]-n[i]));n[i]+=admitted;queue[i]-=admitted;
      }
      for(let i=0;i<base.length;i++)if(valid[i]){
        let incoming=input.demands[k][i];arrived+=incoming;
        if(policy!=='baseline'){
          // Only fresh arrivals may be diverted, through open same-layer adjacency.
          const candidates=neighbors[i].filter(j=>valid[j]&&queue[j]===0&&n[j]<capacity[j]).sort((a,b)=>n[a]/capacity[a]-n[b]/capacity[b]);
          for(const j of candidates){
            if(!incoming)break;
            if(n[i]<capacity[i]&&n[j]/capacity[j]>=n[i]/capacity[i]-.10)continue;
            const moved=Math.min(incoming,capacity[j]-n[j],Math.max(1,Math.ceil((n[i]/capacity[i]-n[j]/capacity[j])*capacity[j]/2)));
            n[j]+=moved;incoming-=moved;diverted+=moved;transfers.push({from:i,to:j,count:moved});
          }
        }
        const admitted=Math.min(incoming,Math.max(0,capacity[i]-n[i]));n[i]+=admitted;queue[i]+=incoming-admitted;
      }
      const time=input.time+(k+1)*dt/60,cells=base.map((c,i)=>{
        if(!valid[i])return {...c,sampleTime:M.iso(time)};
        const speed=n[i]?M.round(M.clamp(22*(1-.72*Math.min(1,n[i]/capacity[i])**2),3,24)):null;
        return {...c,n:n[i],capacity:capacity[i],u:n[i]/capacity[i],speed,density:M.round(n[i]/c.length),flow:M.round(n[i]*(speed??0)*60/(c.length*1000)),queue:queue[i],sampleTime:M.iso(time),source:'本地中心宏观库存流推演'};
      });
      save((k+1)*dt,cells,transfers);
      k++;return current;
    }
    return {advance,current:()=>current};
  }
  function simulate(input,policy){
    const runner=createRunner(input,policy),frames=[runner.current()];let frame;
    while((frame=runner.advance()))frames.push(frame);
    return {frames};
  }
  function createHistory(scenario='peak',quality='normal'){
    const input=buildInput(0,scenario,3600,quality),ids=['baseline',...Object.keys(policies)],runners=ids.map(id=>createRunner(input,id,true));let to=-step;
    return {advance(target){
      if(!Number.isFinite(target)||target<0||target>3600)throw Error('历史截止须为0–3600秒');
      const runs=ids.map(id=>({id,label:policies[id],timeline:[]}));
      while(to+step<=target){
        to+=step;for(let i=0;i<runners.length;i++){const frame=to===0?runners[i].current():runners[i].advance();runs[i].timeline.push({t:to,...frame.metrics});}
      }
      return {to,runs};
    }};
  }
  function compare(input,policy='balance'){
    if(!Object.hasOwn(policies,policy))throw Error('未知中心推演策略');
    const baseline=simulate(input,'baseline');return {input,policy:{id:policy,label:policies[policy]},baseline,strategy:policy==='baseline'?baseline:simulate(input,policy)};
  }
  function snapshot(input,frame,selectedLayer='all'){
    if(selectedLayer!=='all'&&!M.layers.some(l=>l.id===selectedLayer))throw Error('未知方向层');
    const cells=frame.cells,selected=cells.filter(c=>selectedLayer==='all'||c.layerId===selectedLayer);
    return {cells,summary:M.aggregate(selected),regions:M.regions.map(r=>({...r,...M.aggregate(selected.filter(c=>c.regionId===r.id))})),layers:M.layers.map(l=>({...l,...M.aggregate(cells.filter(c=>c.layerId===l.id))})),sampleTime:M.iso(frame.time),quality:input.quality,source:'本地中心宏观推演 · 非真实运行'};
  }
  const api={capture,compare,snapshot,createHistory};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.CenterProjection=api;
})(typeof window!=='undefined'?window:globalThis);
