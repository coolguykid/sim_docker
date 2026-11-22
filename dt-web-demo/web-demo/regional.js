/* Offline, deterministic review model. No database, fetch, or control commands. */
window.RegionDemo = (() => {
  'use strict';
  const C = Object.freeze({version:'MOCK-S3-05',date:'2026-10-20',duration:3600,exampleOnly:true,
    spatial:{coordinateSystem:'LOCAL_ENU',origin:[0,0,0],horizontalUnit:'m',verticalUnit:'m',heightReference:'MOCK_LOCAL_GROUND'},
    layerAngles:[0,30,60,90,120,150],freeTop:120,firstHeight:135,layerPitch:130,
    pipeDiameter:30,laneCount:5,laneWidth:6,pipeGap:5,pipeLength:2000,
    pipesPerLayer:10,guideDiameter:10,guideLanes:1,guideLaneWidth:5,
    funnelBottom:40,funnelTop:110,funnelBottomRadius:90,funnelTopRadius:22,
    takeoff:[-1070,-820],landing:[1070,-820],bounds:[-1500,1500,-1500,1500],
    noFly:[1050,1370,1030,1350,0,850],capacity:32,reserveCapacity:12,
    reserveStart:600,reserveStop:2700,historyStep:30,window:60,sampleStep:1,
    expansion:115,rampStartHandle:100,rampEndHandle:220,thresholds:[.45,.65,.85],colors:['#4E8398','#51846C','#AC7B2C','#B56655']});
  const $ = id => document.getElementById(id), limit=(x,a,b)=>Math.max(a,Math.min(b,x));
  const lerp=(a,b,u)=>a.map((x,i)=>x+(b[i]-x)*u), dist=(a,b)=>Math.hypot(...a.map((x,i)=>x-b[i]));
  const nfmt=(x,d=0)=>x==null?'暂无数据':x.toLocaleString('zh-CN',{maximumFractionDigits:d,minimumFractionDigits:d});
  const clock=t=>{t=Math.floor(limit(t,0,4500));return `${String(10+Math.floor(t/3600)).padStart(2,'0')}:${String(Math.floor(t/60)%60).padStart(2,'0')}:${String(t%60).padStart(2,'0')}`;};
  const iso=t=>new Date(Date.parse(C.date+'T10:00:00+08:00')+t*1000).toISOString();
  const objects=new Map(), pipes=[], ramps=[], branchesIn=[],branchesOut=[];
  let enterCenter;
  const Q={selected:'L1-P01',metric:'utilization',chart:'count',focus:'overview',type:'all',open:'all',load:'all',search:'',
    show:{free:true,funnel:true,guide:true,ramp:true,noFly:true,labels:true,prediction:true},guideDetail:false,yaw:-.38,zoom:1,expanded:true,rate:15,
    state:null,model:null,key:'',last:0,lastUI:0,lastChart:0,stamp:'',hits:[],visible:0,frames:[],renderMs:0,updateMs:0,chartMs:0,
    watched:{in:0,out:0},message:'',ready:false,drag:null,drawnPaths:[],
    // REGION_DB_TOPOLOGY: provisioned regions may replace the mock scene with
    // topology loaded from their own Docker regional service.
    dbTopology:null,dbTopologyRegion:null,dbTopologyLoading:new Set(),dbTopologyError:'',dbSelected:null,
    // REGION_DT_SIMULATION_RESULT: newest CSV attempt and one frame-sized
    // snapshot for the selected Docker region.
    dbSimulation:null,dbSimulationRegion:null,dbSimulationSnapshot:null,
    dbSimulationLoading:false,dbSimulationSnapshotKey:'',dbSimulationError:'',
    dbSimulationSnapshotLoading:false,dbSimulationSnapshotPending:false,
    dbSimulationStream:null,dbSimulationFollow:true,dbSimulationStatsHistory:new Map()};
  function geometry(points){
    const at=points.length===2?u=>lerp(points[0],points[1],u):u=>points[0].map((v,i)=>(1-u)**3*v+3*(1-u)**2*u*points[1][i]+3*(1-u)*u*u*points[2][i]+u**3*points[3][i]);
    const samples=Array.from({length:101},(_,i)=>at(i/100)), lengths=[0];
    for(let i=1;i<samples.length;i++)lengths.push(lengths[i-1]+dist(samples[i-1],samples[i]));
    const length=lengths.at(-1);
    return {points,samples,length,at,atDistance(s){s=limit(s,0,length);let a=0,b=100;while(a+1<b){const m=(a+b)>>1;if(lengths[m]<s)a=m;else b=m;}return lerp(samples[a],samples[b],(s-lengths[a])/(lengths[b]-lengths[a]||1));}};
  }
  function add(id,kind,name,points,extra={}){
    const o={id,kind,name,...geometry(points),...extra};objects.set(id,o);return o;
  }
  function chainGeometry(curves){const parts=curves.map(geometry),length=parts.reduce((n,g)=>n+g.length,0);
    const atDistance=s=>{s=limit(s,0,length);for(const g of parts){if(s<=g.length)return g.atDistance(s);s-=g.length;}return parts.at(-1).at(1);};
    return {points:curves,parts,length,at:u=>atDistance(u*length),atDistance,samples:Array.from({length:201},(_,i)=>atDistance(i/200*length))};
  }
  // Display geometry around the outside of the trunk envelope. Each piece is cubic.
  function guideGeometry(hub,pipe,incoming){const a=pipe.at(incoming?.2:.8),d=pipe.at(1).map((v,i)=>(v-pipe.at(0)[i])/pipe.length),edge=pipe.at(incoming?0:1),outside=edge.map((v,i)=>v+d[i]*(incoming?-350:350));
    const radius=Math.hypot(outside[0],outside[1]),angleA=Math.atan2(hub[1],hub[0]),angleB=Math.atan2(outside[1],outside[0]);let turn=angleB-angleA;while(turn>Math.PI)turn-=2*Math.PI;while(turn<-Math.PI)turn+=2*Math.PI;
    const ring=[Math.cos(angleA)*radius,Math.sin(angleA)*radius,hub[2]],curves=[[hub,lerp(hub,ring,.33),lerp(hub,ring,.67),ring]],pieces=Math.max(1,Math.ceil(Math.abs(turn)/(Math.PI/3)));
    for(let i=0;i<pieces;i++){const aa=angleA+turn*i/pieces,bb=angleA+turn*(i+1)/pieces,k=4/3*Math.tan((bb-aa)/4),z0=hub[2]+(a[2]-hub[2])*i/pieces,z1=hub[2]+(a[2]-hub[2])*(i+1)/pieces,p0=[Math.cos(aa)*radius,Math.sin(aa)*radius,z0],p3=[Math.cos(bb)*radius,Math.sin(bb)*radius,z1];
      curves.push([p0,[p0[0]-Math.sin(aa)*radius*k,p0[1]+Math.cos(aa)*radius*k,z0+(z1-z0)/3],[p3[0]+Math.sin(bb)*radius*k,p3[1]-Math.cos(bb)*radius*k,z1],p3]);}
    const last=curves.at(-1);last[3]=outside;last[2]=outside.map((v,i)=>v+d[i]*(incoming?-140:140));
    curves.push([outside,lerp(outside,a,.33),lerp(outside,a,.67),a]);
    return chainGeometry(incoming?curves:curves.reverse().map(points=>points.slice().reverse()));
  }
  function pipePoint(l,p,u){const a=C.layerAngles[l]*Math.PI/180,off=(p-4.5)*(C.pipeDiameter+C.pipeGap),x=(p<5?-1:1)*C.pipeLength*(.5-u);
    return [x*Math.cos(a)-off*Math.sin(a),x*Math.sin(a)+off*Math.cos(a),C.firstHeight+l*C.layerPitch];}
  for(let l=0;l<6;l++)for(let p=0;p<C.pipesPerLayer;p++){
    const id=`L${l+1}-P${String(p+1).padStart(2,'0')}`;
    const o=add(id,'pipe',`${l+1}层 ${p<5?'正向':'反向'}主干`,[pipePoint(l,p,0),pipePoint(l,p,1)],
      {l,p,reserve:p%5!==0,capacity:p%5!==0?C.reserveCapacity:C.capacity,lanes:C.laneCount,
       startNode:`N-${id}-A`,endNode:`N-${id}-B`,height:C.firstHeight+l*C.layerPitch});pipes.push(o);
  }
  add('FF-01','free','自由飞空域',[[ -1450,-1450,0],[1450,1450,C.freeTop]],{height:'0–120',capacity:null});
  const takeBottom=[...C.takeoff,C.funnelBottom],takeTop=[...C.takeoff,C.funnelTop];
  const landBottom=[...C.landing,C.funnelBottom],landTop=[...C.landing,C.funnelTop];
  add('TK-01','takeoff','起飞漏斗',[takeBottom,takeTop],{height:'40–110',capacity:6,connection:'FF-01 → TK-01 → 目标层汇入引路 IN-L1…L6 / 正反向'});
  add('LD-01','landing','降落漏斗',[landTop,landBottom],{height:'40–110',capacity:6,connection:'目标层汇出引路 OUT-L1…L6 / 正反向 → LD-01 → FF-01'});
  for(let l=0;l<6;l++)for(const p of [0,5]){
    const dest=pipes.find(o=>o.l===l&&o.p===p),a=dest.at(.2),b=dest.at(.8);
    const suffix=`L${l+1}${p===5?'-R':''}`;
    const bi=add('IN-'+suffix,'in','目标层汇入引路 '+suffix,[takeTop,a],{l,p,height:`${C.funnelTop}–${dest.height}`,capacity:12,connection:`TK-01 → ${dest.id} @400m`,pipe:dest.id});Object.assign(bi,guideGeometry(takeTop,dest,true));branchesIn.push(bi);
    const bo=add('OUT-'+suffix,'out','目标层汇出引路 '+suffix,[b,landTop],{l,p,height:`${C.funnelTop}–${dest.height}`,capacity:12,connection:`${dest.id} @1600m → LD-01`,pipe:dest.id});Object.assign(bo,guideGeometry(landTop,dest,false));branchesOut.push(bo);
  }
  for(let l=0;l<5;l++)for(const p of [0,5])for(const descending of [false,true]){
    const source=descending?l+1:l,target=descending?l:l+1,exit=descending?.3:.6,entry=descending?.42:.72;
    const a=pipePoint(source,p,exit),b=pipePoint(target,p,entry),va=pipePoint(source,p,1).map((x,i)=>(x-pipePoint(source,p,0)[i])/C.pipeLength),vb=pipePoint(target,p,1).map((x,i)=>(x-pipePoint(target,p,0)[i])/C.pipeLength);
    const startHandle=descending?C.rampEndHandle:C.rampStartHandle,endHandle=descending?C.rampStartHandle:C.rampEndHandle;
    ramps.push(add(`X-L${source+1}-L${target+1}${p?'-R':''}`,'ramp',`层间${descending?'下行':'上行'}引路`,[a,a.map((x,i)=>x+va[i]*startHandle),b.map((x,i)=>x-vb[i]*endHandle),b],{l:source,targetLayer:target,p,height:`${a[2]}–${b[2]}`,capacity:8,connection:`L${source+1}-P${p?'06':'01'} @${exit*C.pipeLength}m → L${target+1}-P${p?'06':'01'} @${entry*C.pipeLength}m`}));
  }
  const freeIn=geometry([[-1410,-1250,65],[-1370,-980,85],[-1180,-940,40],takeBottom]);
  const freeOut=geometry([landBottom,[1190,-980,40],[1360,-1020,85],[1410,-1250,65]]);
  function segment(id,stage,speed,g=objects.get(id)){return {id,stage,speed,g,duration:g.length/speed};}
  const part=(id,a,b)=>geometry([objects.get(id).at(a),objects.get(id).at(b)]);
  function stream(id,segments,start,interval,end=3600,boundary=false){let duration=0;const events=[];
    for(const s of segments){s.offset=duration;duration+=s.duration;s.end=duration;events.push({id:s.id,stage:s.stage,offset:duration});}
    return {id,segments,start,interval,end,duration,events,boundary};
  }
  function makeModel(region,scenario){
    const streams=[];const mainSpeed=scenario==='peak'?9:12;
    for(const o of pipes){
      if(o.id==='L4-P07')continue; // An explicitly empty reserve object, never fabricated particles.
      if(o.reserve&&scenario!=='reserve')continue;
      const speed=mainSpeed+(o.l+o.p+region)%3,interval=o.reserve?24:scenario==='peak'?8:18;
      const start=o.reserve?C.reserveStart+(o.p+o.l)*2:-220+(o.p+o.l+region)%17;
      streams.push(stream('B-'+o.id,[segment(o.id,'主干',speed)],start,interval,o.reserve?C.reserveStop:3600,true));
    }
    const inSegments=[segment('FF-01','自由飞',8,freeIn),segment('TK-01','起飞',5),segment('IN-L1','汇入',12),segment('L1-P01','主干',12,part('L1-P01',.2,.6)),segment('X-L1-L2','层间转移',10),segment('L2-P01','主干',12,part('L2-P01',.72,1))];
    const outSegments=[segment('L1-P01','主干',12,part('L1-P01',0,.8)),segment('OUT-L1','汇出',12),segment('LD-01','降落',5),segment('FF-01','自由飞',8,freeOut)];
    const incoming=stream('SIM-IN',inSegments,0,90,3300,true),outgoing=stream('SIM-OUT',outSegments,35,100,3300,false);
    streams.push(incoming,outgoing);
    for(const r of ramps){
      const suffix=r.p?'-R':'',source=`L${r.l+1}-P${r.p?'06':'01'}`,target=`L${r.targetLayer+1}-P${r.p?'06':'01'}`,down=r.targetLayer<r.l;
      const segments=[segment('FF-01','自由飞',8,freeIn),segment('TK-01','起飞',5),segment(`IN-L${r.l+1}${suffix}`,'汇入',12),segment(source,'主干',12,part(source,.2,down?.3:.6)),segment(r.id,'层间转移',10),segment(target,'主干',12,part(target,down?.42:.72,.8)),segment(`OUT-L${r.targetLayer+1}${suffix}`,'汇出',12),segment('LD-01','降落',5),segment('FF-01','自由飞',8,freeOut)];
      // Pre-roll complete tasks so every guide has traffic throughout the displayed hour.
      const duration=segments.reduce((n,s)=>n+s.duration,0),interval=Math.min(...segments.filter(s=>['汇入','汇出','层间转移'].includes(s.stage)).map(s=>s.duration))*.85;
      streams.push(stream('SIM-'+r.id,segments,-duration-(r.l+r.p+region)%11,interval,3600,true));
    }
    return {region,scenario,streams,incoming,outgoing};
  }
  function vehicle(s,k,t,region=Q.model?.region??0){const start=s.start+k*s.interval,age=t-start;if(k<0||start>=s.end||age<0||age>=s.duration)return null;
    const seg=s.segments.find(x=>age<x.end),distance=(age-seg.offset)*seg.speed;return {id:`R${String(region+1).padStart(2,'0')}:${s.id}:${k}`,stream:s.id,k,start,stage:seg.stage,objectId:seg.id,speed:seg.speed,position:seg.g.atDistance(distance),ahead:seg.g.atDistance(Math.min(seg.g.length,distance+2)),age};}
  function eventCount(s,offset,from,to){if(to<=from)return 0;
    const lo=Math.max(0,Math.ceil((from-offset-s.start)/s.interval));
    const hi=Math.min(Math.ceil((to-offset-s.start)/s.interval)-1,Math.ceil((s.end-s.start)/s.interval)-1);
    return Math.max(0,hi-lo+1);
  }
  function activeVehicles(t,model=Q.model){const result=[];
    for(const s of model.streams){const first=Math.max(0,Math.floor((t-s.start-s.duration)/s.interval)+1),last=Math.floor((t-s.start)/s.interval);
      for(let k=first;k<=last;k++){const v=vehicle(s,k,t,model.region);if(v)result.push(v);}}
    return result;
  }
  function missing(o,t){return o.id==='L3-P01'&&t>=1680&&t<1860;}
  function limits(t){return {speedLimit:t<1800?18:14,safeDistance:t<1800?80:100};}
  function stats(o,t,vehicles,model=Q.model){const vs=vehicles.filter(v=>v.objectId===o.id);let status='开放';
    if(o.reserve){if(model.scenario!=='reserve'||t<C.reserveStart)status='关闭';else if(t>=C.reserveStop)status=vs.length?'排空':'关闭';}
    const absent=missing(o,t),flow=model.streams.reduce((sum,s)=>sum+s.events.filter(e=>e.id===o.id).reduce((v,e)=>v+eventCount(s,e.offset,Math.max(0,t-C.window),t),0),0);
    const n=absent?null:vs.length,speed=absent||!n?null:vs.reduce((a,v)=>a+v.speed,0)/n,capacity=o.capacity??null;
    return {id:o.id,n,speed,flow:absent||t<C.window?null:flow,density:absent||o.kind!=='pipe'?null:n/(o.length/1000),capacity,
      u:absent||!capacity?null:Math.min(1,Math.max(0,n/capacity)),status,missing:absent,...limits(t),sampleTime:iso(Math.floor(t)),visible:vs.length};
  }
  const snapshotCache=new Map(),objectCache=new Map(),chartCache=new Map();let seekFrame=null;
  function remember(cache,key,value,limit=48){cache.set(key,value);if(cache.size>limit)cache.delete(cache.keys().next().value);return value;}
  function snapshot(t){const key=Q.key+'|'+t;if(snapshotCache.has(key))return snapshotCache.get(key);const vehicles=activeVehicles(t),states=new Map();for(const o of objects.values())states.set(o.id,stats(o,t,vehicles));return remember(snapshotCache,key,{t,vehicles,states});}
  function objectSnapshot(o,t){const key=Q.key+'|'+o.id+'|'+t;if(objectCache.has(key))return objectCache.get(key);const vehicles=activeVehicles(t);return remember(objectCache,key,{vehicles,states:new Map([[o.id,stats(o,t,vehicles)]])},96);}
  function color(a,metric=Q.metric){if(a.missing)return '#93928C';if(a.status==='关闭')return '#A4AAA0';
    const v=metric==='count'?(a.n??0)/24:metric==='speed'?(a.speed==null?null:1-a.speed/20):metric==='density'?(a.density??0)/12:a.u;
    if(v==null)return '#93928C';return C.colors[v<.45?0:v<.65?1:v<.85?2:3];}
  function filteredPipes(snap=Q.snap){const layer=Q.state.layer;
    return pipes.filter(o=>{const a=snap.states.get(o.id);return (layer==='all'||o.l===+layer)&&(Q.type==='all'||(Q.type==='reserve')===o.reserve)&&
      (Q.open==='all'||(Q.open==='missing'?a.missing:a.status===Q.open))&&(Q.load==='all'||(Q.load==='high'?a.u!=null&&a.u>=.85:a.u!=null&&a.u<.85))&&
      (!Q.search||o.id.includes(Q.search)||(`${regionId()}:${o.id}`).includes(Q.search));});
  }
  const regionId=()=>`R${String(Q.state.region+1).padStart(2,'0')}`;
  function aggregateScope(list,snap){let n=0,weighted=0,capacity=0,valid=0,missingCount=0;
    for(const o of list){const a=snap.states.get(o.id);if(a.n==null){missingCount++;continue;}n+=a.n;weighted+=(a.speed??0)*a.n;if(a.status!=='关闭')capacity+=a.capacity||0;valid++;}
    return {n,speed:n?weighted/n:null,u:capacity?Math.min(1,Math.max(0,n/capacity)):null,valid,missingCount};}
  function completion(t){const m=Q.model,from=Math.max(0,t-C.window);let incoming=0,outgoing=0;
    for(const s of m.streams)for(const e of s.events){if(e.stage==='汇入')incoming+=eventCount(s,e.offset,from,t);if(e.stage==='降落')outgoing+=eventCount(s,e.offset,from,t);}
    return {in:incoming,out:outgoing,completed:m.streams.reduce((n,s)=>n+eventCount(s,s.duration,from,t),0)};}
  function init(){
    $('regionalDemo').innerHTML=`
      <div class="rd-viewbar"><div><strong>区域数字孪生 · 运行与推演</strong><span class="rd-note">历史观测采用模拟替代值</span></div><div id="rdSlideControls"><label>区域 <select id="rdSlideRegion" aria-label="展示视图区域"></select></label><label>场景 <select id="rdSlideScenario" aria-label="展示视图场景"></select></label><button type="button" id="rdSlideEnterCenter">查看中心</button></div><button id="rdViewToggle" aria-pressed="false">切换到16:9全览</button></div><div class="rd-kpis"><div class="rd-kpi"><span>主干内飞行器数量</span><b id="rdCount">—</b><small> 架</small><span id="rdScope"></span></div><div class="rd-kpi"><span>主干加权平均速度</span><b id="rdSpeed">—</b><small> m/s</small><span>按有效个体数量加权</span></div><div class="rd-kpi"><span>主干容量利用率</span><b id="rdUtil">—</b><small> %</small><span>有效数量 / 开放及排空容量</span></div><div class="rd-kpi"><span>连接观察 · 过去 60 秒</span><b id="rdPass">—</b><small> 架</small><span>汇入主干完成 / 降落完成</span></div></div>
      <div class="rd-layout"><section class="panel rd-scene-panel"><div class="rd-head"><div><h3 id="rdMapTitle">区域连接与管道态势</h3></div></div>
      <div class="rd-toolbar" id="rdLayers"><button data-rdlayer="all">全层</button>${C.layerAngles.map((a,l)=>`<button data-rdlayer="${l}" title="原始高度 ${C.firstHeight+l*C.layerPitch}m">L${l+1} · ${a}°</button>`).join('')}</div>
      <div class="rd-toolbar"><select id="rdMetric" aria-label="区域着色指标"><option value="utilization">容量利用率 · %</option><option value="count">数量 · 架</option><option value="speed">平均速度 · m/s</option><option value="density">密度 · 架/km管道</option></select><button id="rdExpand">分层展开</button><button id="rdReset">复位视角</button><button id="rdLabels" title="控制区域总览场景中的文字标注" aria-pressed="true">隐藏总览文字</button><div class="rd-views"><button data-focus="overview">总览</button><button data-focus="in">汇入示例 ↗</button><button data-focus="out">汇出示例 ↘</button></div></div>
      <div class="rd-toolbar">${[['free','自由飞空域'],['funnel','起降漏斗'],['guide','汇入/汇出引路'],['ramp','层间引路'],['noFly','模拟禁飞区']].map(([k,v])=>`<label><input type="checkbox" data-show="${k}" ${Q.show[k]?'checked':''}>${v}</label>`).join('')}<button id="rdGuideDetail" title="重点模式突出所选对象关联引路，其他引路淡化；全部模式显示完整连接">引路：重点</button></div>
      <div class="rd-toolbar rd-projection-tools" id="rdProjectionTools"></div><div class="rd-stage"><canvas id="rdScene" aria-label="区域主干、自由飞及起降连接模拟场景"></canvas><div class="rd-overlay"><strong id="rdCameraTitle"></strong></div><div class="rd-scene-hint">拖动旋转 · 滚轮 / 双指按钮缩放 · 点击图元选择 <button id="rdZoomOut" aria-label="缩小">−</button> <button id="rdZoomIn" aria-label="放大">＋</button></div><div class="rd-tooltip" id="rdTip"></div></div>
      <div id="rdProjectionCompare" hidden></div><div class="rd-legend" id="rdLegend"></div><div class="rd-legend"><span><i style="background:#7556A0"></i>汇入 · 实线紫</span><span><i style="background:#7C5895"></i>汇出 · 点划紫</span><span><i style="background:#A27B29"></i>层间上行 · 金虚线</span><span><i style="background:#377F94"></i>层间下行 · 蓝点划线</span><span>○节点　┄关闭　▰排空　×缺失</span></div>
      <div class="rd-message" id="rdMessage" aria-live="polite"></div></section>
      <aside class="rd-aside">      <section class="panel" id="rdRankPanel"><div class="rd-head"><h3>本区域主干利用率排序</h3><span id="rdRankScope" class="rd-note"></span></div><div class="rd-toolbar"><select id="rdType" aria-label="管道类型"><option value="all">全部类型</option><option value="main">主用</option><option value="reserve">备用</option></select><select id="rdOpen" aria-label="开放状态"><option value="all">全部状态</option><option>开放</option><option>关闭</option><option>排空</option><option value="missing">缺失</option></select><select id="rdLoad" aria-label="拥堵等级"><option value="all">全部负载</option><option value="high">高负载 ≥85%</option><option value="low">负载 &lt;85%</option></select><input id="rdSearch" type="search" placeholder="搜索 ID，例如 L1-P01" aria-label="搜索管道或连接ID"><button id="rdSearchGo">定位</button></div><div class="rd-ranks" id="rdRanks"></div></section><section class="panel rd-detail" id="rdDetail"></section></aside></div><div class="rd-small rd-visibility" id="rdVisible"></div><div class="rd-evaluations" id="rdEvaluations"></div>
      <div class="rd-lower"><div><section class="panel"><div class="rd-head"><div><h3 id="rdChartTitle"></h3><span class="rd-note">实线模拟历史 / 虚线模拟预测 / 缺失留断点</span></div><div class="rd-chart-tabs" id="rdChartTabs">${[['count','数量'],['speed','均速'],['flow','流量'],['density','密度']].map(([k,v])=>`<button data-chart="${k}">${v}</button>`).join('')}<label class="rd-note"><input id="rdPrediction" type="checkbox" checked>预测</label></div></div><div class="rd-history"><canvas id="rdHistory" aria-label="所选对象历史与预测曲线"></canvas></div><div id="rdForecastNote" class="rd-chart-caption rd-small"></div></section>
      <div class="rd-mini-grid"><section class="panel"><div class="rd-head"><h3>通行约束 · 上限与运行对比</h3><span class="rd-note">约束虚线 / 运行实线</span></div><div class="rd-mini-chart"><canvas id="rdLimits" aria-label="安全距离与最小纵向间距、速度上限与运行最高速度对比"></canvas></div><div class="rd-chart-caption rd-small"><div class="rd-limits-legend" aria-label="通行约束线条图例"><span><i class="rd-limit-distance rd-limit-bound"></i>安全距离下限 · m</span><span><i class="rd-limit-distance"></i>运行最小纵向间距 · m</span><span><i class="rd-limit-speed rd-limit-bound"></i>速度上限 · m/s</span><span><i class="rd-limit-speed"></i>运行最高速度 · m/s</span></div><span id="rdLimitsNote"></span></div></section></div></div>
</div>
      <section class="panel rd-time"><button id="rdPlay">暂停</button><button id="rdReplay">重新播放</button><strong id="rdTime"></strong><input id="rdTimeSlider" type="range" min="0" max="3600" step="1" value="1800" aria-label="区域仿真时间秒"><select id="rdRate" aria-label="动画播放倍率"><option>1</option><option selected>15</option><option>30</option><option>60</option></select><span class="rd-note">×</span><button id="rdLatest">返回末帧</button></section><div class="rd-metadata" id="rdTimeMeta"></div>
`;
    bind();RegionEvaluation.mount();RegionProjection.mount();
    $('rdTime').parentElement.append($('rdProjectionPlayback'));
    $('rdCount').previousElementSibling.textContent='主干内飞行器数量';
    const utilState=document.createElement('span');utilState.id='rdUtilState';utilState.className='rd-kpi-state';utilState.textContent='无数据';utilState.dataset.level='unknown';$('rdUtil').nextElementSibling.after(utilState);
    for(const [id,label] of [['rdLayers','图层'],['rdMetric','观察'],['rdGuideDetail','显示'],['rdProjectionTools','推演']]){
      const control=$(id),group=control.matches('.rd-toolbar')?control:control.closest('.rd-toolbar');
      group.classList.add('rd-tool-group');group.dataset.groupLabel=label;group.setAttribute('role','group');group.setAttribute('aria-label',label);
    }
    window.DTDesktop?.enhance?.($('regionalDemo'));Q.ready=true;
    let queued=false;new ResizeObserver(()=>{if(!queued&&Q.state?.view==='region'){queued=true;requestAnimationFrame(()=>{queued=false;resize();});}}).observe($('rdScene').parentElement);
  }
  function resetFilters(){Q.type=Q.open=Q.load='all';Q.search='';for(const [id,v] of [['rdType','all'],['rdOpen','all'],['rdLoad','all'],['rdSearch','']])$(id).value=v;}
  function follow(){const t=Q.state.t*60;for(const [key,s] of [['in',Q.model.incoming],['out',Q.model.outgoing]])Q.watched[key]=Math.max(0,Math.floor((t-s.start-s.duration)/s.interval)+1);}
  function select(id){const o=objects.get(id);if(!o)return;Q.selected=id;Q.message='';
    if(o.l!=null&&Q.state.layer!=='all'&&+Q.state.layer!==o.l)Q.state.layer=String(o.l);
    if(o.kind==='pipe'){if(!filteredPipes().includes(o))resetFilters();Q.focus='overview';Q.zoom=1;}
    if(o.kind==='free'||o.kind==='ramp'){Q.focus='overview';Q.zoom=1;}
    if(o.kind==='free')Q.show.free=true;else if(o.kind==='takeoff'||o.kind==='landing')Q.show.funnel=true;else if(o.kind==='in'||o.kind==='out')Q.show.guide=true;else if(o.kind==='ramp')Q.show.ramp=true;
    if(o.kind==='in'||o.kind==='takeoff')Q.focus='in';if(o.kind==='out'||o.kind==='landing')Q.focus='out';
    document.querySelectorAll('[data-show]').forEach(x=>x.checked=Q.show[x.dataset.show]);refresh(true);
  }
  // REGION_DB_TOPOLOGY: database object selection is kept separate from the
  // legacy mock IDs because pipe:0 and ramp:0 may both exist.
  function selectDatabaseObject(key){if(!Q.dbTopology?.objects.has(key))return;Q.dbSelected=key;Q.message='';refresh(false);}
  function bind(){
    const root=$('regionalDemo');
    root.addEventListener('click',e=>{const b=e.target.closest('button');if(!b)return;
      if(b.dataset.rdlayer!=null){Q.state.layer=b.dataset.rdlayer;Q.selected=null;Q.focus='overview';Q.stamp='';refresh(true);}
      if(b.dataset.focus){Q.focus=b.dataset.focus;Q.yaw=-.38;Q.zoom=1;Q.selected=Q.focus==='in'?'IN-L1':Q.focus==='out'?'OUT-L1':'L1-P01';Q.state.layer='all';resetFilters();follow();refresh(true);}
      if(b.dataset.object)select(b.dataset.object);
      if(b.dataset.dbObject)selectDatabaseObject(b.dataset.dbObject);
      if(b.dataset.chart){Q.chart=b.dataset.chart;refresh(true);}
    });
    for(const [id,k] of [['rdMetric','metric'],['rdType','type'],['rdOpen','open'],['rdLoad','load']])$(id).addEventListener('change',e=>{Q[k]=e.target.value;Q.message='';if(k!=='metric')Q.selected=null;refresh(true);});
    $('rdSearch').addEventListener('input',e=>{Q.search=e.target.value.trim().toUpperCase();Q.selected=null;refresh(true);});
    const searchGo=()=>{const q=Q.search.replace(/^R\d+:/,'');if(objects.has(q)){resetFilters();select(q);}else{Q.message='未找到此 ID；输入 L1-P01、IN-L1 或 TK-01。';refresh(true);}};
    $('rdSearchGo').onclick=searchGo;$('rdSearch').addEventListener('keydown',e=>{if(e.key==='Enter')searchGo();});
    root.addEventListener('change',e=>{if(e.target.dataset.show){Q.show[e.target.dataset.show]=e.target.checked;const o=objects.get(Q.selected);if(o&&!objectVisible(o))Q.selected=null;refresh(true);}});
    $('rdPrediction').onchange=e=>{Q.show.prediction=e.target.checked;refresh(true);};
    $('rdExpand').onclick=()=>{Q.expanded=!Q.expanded;refresh(true);};
    $('rdLabels').onclick=()=>{Q.show.labels=!Q.show.labels;refresh();};
    $('rdGuideDetail').onclick=()=>{Q.guideDetail=!Q.guideDetail;refresh();};
    $('rdReset').onclick=()=>{Q.yaw=-.38;Q.zoom=1;Q.focus='overview';refresh(true);};

    for(const [id,scale] of [['rdZoomIn',1.15],['rdZoomOut',1/1.15]])$(id).onclick=()=>{Q.zoom=limit(Q.zoom*scale,.65,2);drawScene();};
    $('rdPlay').onclick=()=>{Q.dbSimulationFollow=false;if(Q.state.t>=playbackEndSeconds()/60){Q.state.t=0;follow();}Q.state.playing=!Q.state.playing;Q.last=performance.now();refresh(true);};
    $('rdReplay').onclick=()=>{Q.dbSimulationFollow=false;Q.state.t=0;Q.state.playing=true;follow();Q.last=performance.now();refresh(true);};
    $('rdLatest').onclick=()=>{Q.dbSimulationFollow=true;Q.state.t=playbackEndSeconds()/60;Q.state.playing=false;follow();refresh(true);};
    $('rdTimeSlider').oninput=e=>{Q.dbSimulationFollow=false;Q.state.t=+e.target.value/60;Q.state.playing=false;if(seekFrame===null)seekFrame=requestAnimationFrame(()=>{seekFrame=null;if(Q.state.view==='region'){follow();refresh(true);}});};
    $('rdTimeSlider').onchange=()=>{if(seekFrame!==null){cancelAnimationFrame(seekFrame);seekFrame=null;}follow();refresh(true);};
    $('rdRate').onchange=e=>{Q.rate=+e.target.value;refresh(true);};
    $('rdSlideEnterCenter').onclick=()=>enterCenter?.();
    $('rdViewToggle').onclick=()=>presentation(!$('regionalDemo').classList.contains('rd-slide'));
    for(const [id,target] of [['rdSlideRegion','regionSelect'],['rdSlideScenario','scenario']]){
      $(id).replaceChildren(...[...$(target).options].map(o=>new Option(o.text,o.value)));
      $(id).onchange=()=>{$(target).value=$(id).value;$(target).dispatchEvent(new Event('change'));};
    }
    window.addEventListener('resize',()=>{if(root.classList.contains('rd-slide')){fitPresentation();resize();}});
    document.addEventListener('keydown',e=>{if(e.key==='Escape'&&root.classList.contains('rd-slide'))presentation(false);});
    const canvas=$('rdScene');
    canvas.onpointerdown=e=>{Q.drag={x:e.clientX,y:e.clientY,last:e.clientX,moved:false};canvas.setPointerCapture(e.pointerId);};
    canvas.onpointermove=e=>{const rect=canvas.getBoundingClientRect();if(Q.drag){const dx=e.clientX-Q.drag.last;Q.drag.last=e.clientX;if(Math.hypot(e.clientX-Q.drag.x,e.clientY-Q.drag.y)>4)Q.drag.moved=true;if(Q.drag.moved){Q.yaw+=dx*.008;drawScene();}return;}
      const x=(e.clientX-rect.left)*canvas.clientWidth/rect.width,y=(e.clientY-rect.top)*canvas.clientHeight/rect.height;
      const hit=hitAt(x,y),tip=$('rdTip');tip.style.display=hit?'block':'none';if(hit){if(hit.dbKey){const o=Q.dbTopology.objects.get(hit.dbKey),open=o.state?.isOpen!==false;tip.textContent=`${o.key} · ${o.type} · ${open?'开放':'关闭'}`;}else{const o=objects.get(hit.id);tip.textContent=`${o.id} · ${o.name} · 模拟`;}tip.style.left=Math.max(5,Math.min(x+10,canvas.clientWidth-230))+'px';tip.style.top=Math.max(5,y-30)+'px';}};
    canvas.onpointerup=e=>{if(Q.drag&&!Q.drag.moved){const r=canvas.getBoundingClientRect(),hit=hitAt((e.clientX-r.left)*canvas.clientWidth/r.width,(e.clientY-r.top)*canvas.clientHeight/r.height);if(hit){if(hit.dbKey)selectDatabaseObject(hit.dbKey);else select(hit.id);}}Q.drag=null;};
    canvas.onpointercancel=()=>Q.drag=null;canvas.onpointerleave=()=>{$('rdTip').style.display='none';};
    canvas.addEventListener('wheel',e=>{e.preventDefault();Q.zoom=limit(Q.zoom*Math.exp(-e.deltaY*.001),.65,2);drawScene();},{passive:false});
  }
  function render(state,goCenter){
    if(goCenter)enterCenter=goCenter;
    Q.state=state;if(!Q.ready)init();const key=state.region+':'+state.scenario;
    if(Q.key!==key){const old=Q.key;Q.key=key;Q.model=makeModel(state.region,state.scenario);Q.stamp='';Q.last=performance.now();Q.frames=[];
      if(old&&!old.startsWith(state.region+':')){Q.selected=null;Q.focus='overview';resetFilters();}follow();}
    $('rdSlideRegion').value=state.region;$('rdSlideScenario').value=state.scenario;
    $('pageTitle').textContent='区域数字孪生态势';$('enterRegion').textContent='返回中心 ↗';$('regionSelect').value=state.region;
    document.querySelectorAll('[data-view]').forEach(b=>{b.classList.toggle('active',b.dataset.view==='region');b.setAttribute('aria-pressed',String(b.dataset.view==='region'));});
    // Clear/schedule the selected region's external topology before painting,
    // so switching from R01–R35 to offline R36 cannot leave a stale map.
    Q.last=performance.now();loadDatabaseTopologyForRegion();loadDatabaseSimulationForRegion();refresh(true);
  }

  // REGION_DB_TOPOLOGY: regions 1–9 load their own Docker database. Regions
  // R01–R35 use Docker databases; R36 remains the deterministic offline demo.
  function loadDatabaseTopologyForRegion(){
    const regionId=Q.state.region+1;
    if(!RegionDbTopology.endpoints[regionId]){Q.dbTopology=null;Q.dbTopologyRegion=null;Q.dbTopologyError='';Q.dbSelected=null;return;}
    if(Q.dbTopologyRegion===regionId)return;
    if(Q.dbTopologyRegion!==regionId){Q.dbTopology=null;Q.dbSelected=null;}
    if(Q.dbTopologyLoading.has(regionId))return;
    Q.dbTopologyLoading.add(regionId);Q.dbTopologyError='';Q.message=`正在读取区域${regionId} Docker数据库路网…`;
    RegionDbTopology.load(regionId).then(topology=>{
      if(Q.state.region+1!==regionId)return;
      Q.dbTopology=topology;Q.dbTopologyRegion=regionId;
      Q.message=`已加载区域${regionId}数据库路网 · 拓扑版本 ${topology.topologyVersion}`;
      refresh(true);
    }).catch(error=>{
      if(Q.state.region+1!==regionId)return;
      Q.dbTopologyError=error.message;Q.message=`区域${regionId}数据库路网加载失败，暂时显示原离线演示：${error.message}`;
      refresh(true);
    }).finally(()=>{Q.dbTopologyLoading.delete(regionId);});
  }

  function playbackEndSeconds(){return Q.dbSimulation?.maxTime??C.duration;}
  function loadDatabaseSimulationForRegion(){
    const region=Q.state.region+1;
    if(!RegionDbTopology.endpoints[region]){Q.dbSimulationStream?.close();Q.dbSimulationStream=null;Q.dbSimulation=null;Q.dbSimulationSnapshot=null;Q.dbSimulationRegion=null;return;}
    if(Q.dbSimulationRegion!==region){Q.dbSimulationStream?.close();Q.dbSimulationStream=null;Q.dbSimulation=null;Q.dbSimulationSnapshot=null;Q.dbSimulationSnapshotKey='';Q.dbSimulationFollow=true;Q.dbSimulationSnapshotLoading=false;Q.dbSimulationSnapshotPending=false;Q.dbSimulationStatsHistory.clear();}
    if(Q.dbSimulationLoading)return;
    Q.dbSimulationLoading=true;Q.dbSimulationError='';
    RegionSimulationResults.latest(region).then(result=>{
      if(Q.state.region+1!==region)return;
      Q.dbSimulation=result;Q.dbSimulationRegion=region;
      if(Q.state.t*60>result.maxTime)Q.state.t=result.maxTime/60;
      $('rdTimeSlider').max=result.maxTime;
      loadDatabaseSimulationSnapshot(true);
    }).catch(error=>{
      if(Q.state.region+1!==region)return;
      Q.dbSimulation=null;Q.dbSimulationSnapshot=null;
      if(!String(error.message).includes('HTTP 404'))Q.dbSimulationError=error.message;
    }).finally(()=>{Q.dbSimulationLoading=false;if(Q.state.region+1!==region)loadDatabaseSimulationForRegion();else startDatabaseSimulationStream(region);});
  }
  function startDatabaseSimulationStream(region){
    Q.dbSimulationStream?.close();
    Q.dbSimulationStream=RegionSimulationResults.stream(region,{
      runId:Q.dbSimulation?.runId,attemptId:Q.dbSimulation?.attemptId,after:Q.dbSimulation?.maxTime,
      onRun:metadata=>{
        if(Q.state.region+1!==region)return;
        const changed=!Q.dbSimulation||Q.dbSimulation.runId!==metadata.runId||Q.dbSimulation.attemptId!==metadata.attemptId;
        Q.dbSimulation={...metadata};Q.dbSimulationRegion=region;
        if(changed){Q.dbSimulationFollow=true;Q.dbSimulationSnapshot=null;Q.dbSimulationSnapshotKey='';Q.dbSimulationStatsHistory.clear();}
      },
      onSnapshot:snapshot=>{
        if(Q.state.region+1!==region)return;
        Q.dbSimulation={...(Q.dbSimulation||{}),runId:snapshot.runId,attemptId:snapshot.attemptId,regionId:snapshot.regionId,minTime:Q.dbSimulation?.minTime??0,maxTime:Math.max(Q.dbSimulation?.maxTime??0,snapshot.frameTime)};
        Q.dbSimulationSnapshot=snapshot;Q.dbSimulationSnapshotKey=`${snapshot.runId}:${snapshot.attemptId}:${snapshot.frameTime}`;rememberDatabaseStatistics(snapshot);
        if(Q.dbSimulationFollow){Q.state.t=snapshot.frameTime/60;Q.state.playing=false;}
        refresh(true);
      },
      onError:()=>{Q.dbSimulationError='推演实时事件流连接中断，浏览器将自动重连';}
    });
  }
  function loadDatabaseSimulationSnapshot(force=false){
    if(!Q.dbSimulation||Q.dbSimulationRegion!==Q.state.region+1)return;
    const time=Math.max(Q.dbSimulation.minTime,Math.min(Q.dbSimulation.maxTime,Math.floor(Q.state.t*60))),key=`${Q.dbSimulation.runId}:${Q.dbSimulation.attemptId}:${time}`;
    if(!force&&key===Q.dbSimulationSnapshotKey)return;
    // Keep one request in flight and coalesce faster playback ticks into the
    // newest requested time. The former fire-and-discard approach could keep
    // discarding every response at 15x/30x/60x, so only 1x appeared to update.
    if(Q.dbSimulationSnapshotLoading){Q.dbSimulationSnapshotPending=true;return;}
    Q.dbSimulationSnapshotLoading=true;const region=Q.state.region+1,runId=Q.dbSimulation.runId,attemptId=Q.dbSimulation.attemptId;
    RegionSimulationResults.snapshot(region,runId,attemptId,time).then(snapshot=>{
      if(Q.state.region+1!==region||Q.dbSimulation?.runId!==runId||Q.dbSimulation?.attemptId!==attemptId)return;
      if(Q.dbSimulationFollow&&Q.dbSimulationSnapshot?.frameTime>snapshot.frameTime)return;
      Q.dbSimulationSnapshot=snapshot;Q.dbSimulationSnapshotKey=key;rememberDatabaseStatistics(snapshot);refresh(true);
    }).catch(error=>{if(Q.state.region+1===region)Q.dbSimulationError=error.message;}).finally(()=>{
      Q.dbSimulationSnapshotLoading=false;if(Q.dbSimulationSnapshotPending){Q.dbSimulationSnapshotPending=false;loadDatabaseSimulationSnapshot();}
    });
  }
  function refresh(charts=false){
    if(!Q.ready||Q.state.view!=='region')return;const start=performance.now(),t=Q.state.t*60;RegionProjection.update(Q.state.region,Q.state.scenario,t);const projected=RegionProjection.snapshot();Q.snap=projected||snapshot(t);
    if(Q.dbTopology&&Q.dbSimulation)loadDatabaseSimulationSnapshot();
    const list=filteredPipes(),a=aggregateScope(list,Q.snap),done=Q.snap.completion||completion(t),selected=objects.get(Q.selected);
    if(selected&&selected.kind==='pipe'&&!list.includes(selected))Q.selected=null;
    $('rdCount').textContent=nfmt(a.n);$('rdCount').previousElementSibling.textContent='主干内飞行器数量';$('rdSpeed').textContent=nfmt(a.speed,1);$('rdUtil').textContent=a.u==null?'暂无数据':nfmt(a.u*100,1);$('rdPass').textContent=done.in+' / '+done.out;let displayedUtilization=a.u;
    // REGION_DB_TOPOLOGY: topology GET has no runtime UAV/statistics payload.
    // Show topology counts and explicit unavailable values instead of mixing in
    // the mock traffic metrics while the real database network is displayed.
    if(Q.dbTopology){const summary=databaseRegionSummary();
      $('rdCount').previousElementSibling.textContent='主干内飞行器数量';$('rdCount').textContent=summary?nfmt(summary.n):'暂无数据';
      $('rdSpeed').textContent=summary?nfmt(summary.speed,1):'暂无数据';$('rdUtil').textContent=summary?.u==null?'暂无数据':nfmt(summary.u*100,1);$('rdPass').textContent='暂无数据';displayedUtilization=summary?.u??null;
      $('rdPass').previousElementSibling.textContent='连接观察 · 过去 60 秒';$('rdPass').nextElementSibling.nextElementSibling.textContent='汇入主干完成 / 降落完成';
    }
    const utilLevel=!Number.isFinite(displayedUtilization)?'unknown':displayedUtilization>=.85?'high':displayedUtilization>=.65?'warning':'normal',utilText={high:'高负载',warning:'负载关注',normal:'运行正常',unknown:'无数据'}[utilLevel];
    $('rdUtilState').dataset.level=utilLevel;$('rdUtilState').textContent=utilText;
    $('rdScope').textContent=Q.dbTopology?`${regionId()} · 整个区域当前推演帧 · 拓扑 v${Q.dbTopology.topologyVersion}`:`${regionId()} · ${list.length}根筛选主干 · 缺失${a.missingCount}根已排除`;
    $('rdMapTitle').textContent=Q.dbTopology?`${regionId()} · PostgreSQL区域路网`:`${regionId()} · 自由飞与分层主干连接`;
    $('rdCameraTitle').textContent=Q.dbTopology?'数据库拓扑 / 节点—管道—匝道—航道':(projected?RegionProjection.label()+' / ':'')+({overview:'区域总览 / 六层结构',in:'汇入示例 / 自由飞 → 主干',out:'汇出示例 / 主干 → 自由飞'}[Q.focus]);
    const end=playbackEndSeconds();$('rdTimeSlider').max=end;$('rdTime').textContent=clock(t);$('clock').textContent=clock(t);$('rdTimeSlider').value=t;$('rdPlay').textContent=Q.state.playing?'Ⅱ 暂停':t>=end?'↻ 重播':'▶ 播放';
    $('rdTimeMeta').textContent=Q.dbTopology?(Q.dbSimulation?`区域${Q.dbTopology.regionId} DT推演 ${Q.dbSimulation.runId} / attempt ${Q.dbSimulation.attemptId} · 帧 ${nfmt(Q.dbSimulationSnapshot?.frameTime)}s · 统计 ${nfmt(Q.dbSimulationSnapshot?.statisticsTime)}s`:`区域${Q.dbTopology.regionId} PostgreSQL路网 · 当前没有可回放的DT推演CSV`):`仿真 ${C.date} ${clock(t)} 北京时间 · 模拟采样 ${clock(t)} / 1s口径 · 动画 ${Q.rate}× · ${Q.state.playing?'播放':'暂停 / 回放'} · ${C.version} · 本地生成`;
    $('rdExpand').classList.toggle('active',Q.expanded);$('rdExpand').setAttribute('aria-pressed',Q.expanded);
    $('rdLabels').textContent=Q.show.labels?'隐藏总览文字':'显示总览文字';$('rdLabels').classList.toggle('active',Q.show.labels);$('rdLabels').setAttribute('aria-pressed',String(Q.show.labels));
    $('rdGuideDetail').textContent=Q.guideDetail?'引路：全部':'引路：重点';$('rdGuideDetail').classList.toggle('active',Q.guideDetail);$('rdGuideDetail').setAttribute('aria-pressed',String(Q.guideDetail));
    for(const [selector,key] of [['[data-rdlayer]','rdlayer'],['[data-focus]','focus'],['[data-chart]','chart']])document.querySelectorAll(selector).forEach(b=>{const v=key==='rdlayer'?String(Q.state.layer):Q[key];b.classList.toggle('active',b.dataset[key]===v);b.setAttribute('aria-pressed',String(b.dataset[key]===v));});
    const units={utilization:'容量利用率 %',count:'数量 架',speed:'平均速度 m/s',density:'密度 架/km管道'},ranges=Q.metric==='count'?['<11','11–15','16–20','≥21']:Q.metric==='speed'?['>11','7–11','3–7','≤3']:Q.metric==='density'?['<5.4','5.4–7.8','7.8–10.2','≥10.2']:['<45','45–65','65–85','≥85'];
    $('rdLegend').innerHTML=Q.dbTopology?'<b>数据库拓扑 · 沿用离线样式</b><span><i style="background:#4E8398"></i>主用 / 备用管道</span><span><i style="background:#A27B29"></i>层间上行</span><span><i style="background:#377F94"></i>层间下行</span>':`<b>${units[Q.metric]} · 模拟阈值</b>`+C.colors.map((c,i)=>`<span><i style="background:${c}"></i>${ranges[i]}</span>`).join('');
    if(projected){$('clock').textContent=clock(projected.t);$('rdTimeMeta').textContent=`原始快照 ${clock(t)} · ${RegionProjection.label()} · 画面/详情/管道着色均为当前推演帧 · 历史趋势与原回放已暂停`;}
    $('rdPass').previousElementSibling.textContent=projected?'推演累计汇入 / 降落':'连接观察 · 过去 60 秒';
    $('rdPass').nextElementSibling.nextElementSibling.textContent=projected?'从捕获时刻起累计 · 模拟':'汇入主干完成 / 降落完成';
    $('rdMessage').textContent=Q.message;renderDetail();renderRanks();drawScene();RegionEvaluation.refresh();
    const stamp=`${Q.key}|${Q.selected}|${Q.dbSelected}|${Q.chart}|${Math.floor(t/5)}|${Q.show.prediction}|${Q.state.layer}|${Q.type}|${Q.open}|${Q.load}|${Q.search}`;
    if(!projected&&(charts||Q.stamp!==stamp)){Q.stamp=stamp;if(Q.dbTopology)drawDatabaseCharts();else drawCharts();}Q.updateMs=performance.now()-start;
  }
  function field(label,value,wide=false){return `<div${wide?' class="rd-wide"':''}><dt>${label}</dt><dd>${value}</dd></div>`;}
  function databaseLiveEdgeCounts(){const counts=new Map();if(!Q.dbSimulationSnapshot)return counts;for(const uav of Q.dbSimulationSnapshot.uavs||[]){if(!uav.active)continue;const id=uav.edgeType==='pipe'?uav.pipeId:uav.rampId,key=`${uav.edgeType}:${Number(id)}`;counts.set(key,(counts.get(key)||0)+1);}return counts;}
  function databaseRegionSummary(){if(!Q.dbTopology||!Q.dbSimulationSnapshot)return null;const active=(Q.dbSimulationSnapshot.uavs||[]).filter(uav=>uav.active&&uav.edgeType==='pipe'),counts=databaseLiveEdgeCounts(),n=active.length,speed=n?active.reduce((sum,uav)=>sum+(uav.speed??0),0)/n:null;let capacity=0;for(const pipe of Q.dbTopology.pipes){const key=`pipe:${Number(pipe.pipeId)}`,occupied=counts.get(key)||0;if(pipe.state?.isOpen!==false||occupied>0)capacity+=Number(pipe.state?.maxCapacity)||0;}return{n,speed,u:capacity?n/capacity:null,capacity};}
  function renderDetail(){const o=objects.get(Q.selected),box=$('rdDetail');
    // REGION_DB_TOPOLOGY: no mock facility detail is shown beside a real DB
    // topology. Object picking will be added with the dynamic-state phase.
    if(Q.dbTopology){renderDatabaseDetail(box);return;}
    if(!o){box.innerHTML='<h3>所选对象</h3><div class="rd-empty">点击场景或排序选择对象。</div>';return;}
    const a=Q.snap.states.get(o.id),pipe=o.kind==='pipe',type={pipe:o.reserve?'备用主干':'主用主干',free:'自由飞空域',takeoff:'起飞漏斗',landing:'降落漏斗',in:'自由飞至主干引路',out:'主干至自由飞引路',ramp:'主干层间引路'}[o.kind];
    let connection=o.connection;if(pipe)connection=`${o.startNode} → ${o.endNode} · ${o.p<5?'正':'反'}向 · 邻区 ${o.p<5?'R东':'R西'}（模拟）`;
    if(o.kind==='free')connection='FF-01 → TK-01；LD-01 → FF-01。灵活末端路径，排除禁飞区。';
    box.dataset.object=regionId()+':'+o.id;
    box.innerHTML=`<div class="rd-object">${o.id}</div><span class="rd-status">${type} · ${a.status}${a.missing?' / 数据缺失':a.u>=.85?' / 高负载':''}</span><dl class="rd-dl">`+
      field('区域 / 层',`${regionId()} / ${o.l==null?'连接空间':'L'+(o.l+1)}`)+field('原始高度 · m',String(o.height))+
      field('长度 · m',o.kind==='free'?'非固定路径':nfmt(o.length,1))+field('数量 · 架',a.missing?'暂无数据（模拟缺失）':nfmt(a.n))+
      (RegionProjection.active()?field('入口等待 · 架',nfmt(a.queue)): '')+field('平均速度 · m/s',nfmt(a.speed,1))+field('60秒窗出口流量 · 架/分钟',nfmt(a.flow))+
      field('密度 · 架/km管道',pipe?nfmt(a.density,1):'不适用')+field('最大同时容纳 · 架',nfmt(a.capacity))+
      field('速度上限 · m/s',pipe?nfmt(a.speedLimit):'示例段速 5–12')+field('安全距离 · m',pipe?nfmt(a.safeDistance):'未设正式参数')+
      (pipe?field('航道 / 管径',`${C.laneCount}条 / ${C.pipeDiameter}m`):field('几何口径',o.kind==='free'?'水平 2900×2900m；0–120m':o.kind==='takeoff'||o.kind==='landing'?'下口直径180m / 上口44m':'三次贝塞尔 / 10m管径'))+
      field('连接关系',connection,true)+field('采样时间',clock(Q.snap.t)+' 北京时间',true)+'</dl><div class="rd-small">'+
      (pipe?'密度按管道长度；空对象均速为 null。流量按过去60秒离开本对象的去重个体事件计数。':'高度、口径与连接坐标为展示假设；漏斗属于自由飞子网。')+' 全部数值为模拟。</div>';
  }
  // REGION_DB_TOPOLOGY: show only values actually returned by PostgreSQL.
  // Traffic performance fields remain unavailable until a state/statistics
  // endpoint is connected; mock values are never mixed into this card.
  function renderDatabaseDetail(box){const db=Q.dbTopology,item=db.objects.get(Q.dbSelected);
    const rid=`R${String(db.regionId).padStart(2,'0')}`;
    if(!item){box.dataset.object=`${rid}:database-topology`;box.innerHTML=`<div class="rd-object">区域${db.regionId}数据库拓扑</div><span class="rd-status">路网与最新状态 · 版本 ${db.topologyVersion}</span><dl class="rd-dl">${field('节点',db.nodes.size+' 个')}${field('管道',db.pipes.length+' 根')}${field('匝道',db.ramps.length+' 条')}${field('漏斗',db.funnels.length+' 个')}${field('航道',db.lanes.length+' 条')}${field('邻域区域',db.neighbourhood.join('、')||'无')}${field('查询时刻',db.stateTime,true)}</dl><div class="rd-small">点击画布中的管道、匝道或漏斗查看与离线模式相同口径的详情；无时序统计记录时显示暂无数据。</div>`;return;}
    if(item.key.startsWith('funnel:')){const takeoff=item.type==='takeoff',height=`${nfmt(item.bottom[2],1)}–${nfmt(item.top[2],1)}`;box.dataset.object=`${rid}:${item.key}`;box.innerHTML=`<div class="rd-object">${takeoff?'TK':'LD'}-${item.nodeId}</div><span class="rd-status">${takeoff?'起飞漏斗':'降落漏斗'} · 开放</span><dl class="rd-dl">`+
      field('区域 / 层',`${rid} / 连接空间`)+field('原始高度 · m',height)+field('长度 · m',nfmt(dist(item.bottom,item.top),1))+field('数量 · 架','暂无数据')+
      field('平均速度 · m/s','暂无数据')+field('60秒窗出口流量 · 架/分钟','暂无数据')+field('密度 · 架/km管道','不适用')+field('最大同时容纳 · 架','暂无数据')+
      field('速度上限 · m/s','暂无数据')+field('安全距离 · m','暂无数据')+field('几何口径',`下口直径${C.funnelBottomRadius*2}m / 上口${C.funnelTopRadius*2}m`)+
      field('连接关系',takeoff?`自由飞 → 节点 ${item.nodeId} → 漏斗汇入引路`:`漏斗汇出引路 → 节点 ${item.nodeId} → 自由飞`,true)+field('采样时间',db.stateTime,true)+'</dl><div class="rd-small">漏斗节点和有向连接来自数据库；口径沿用离线展示参数。</div>';return;}
    const pipe=item.key.startsWith('pipe:'),state=item.state||{},edgeId=Number(pipe?item.pipeId:item.rampId),edgeKey=`${pipe?'pipe':'ramp'}:${edgeId}`,performance=Q.dbSimulationSnapshot?.statisticsMap.get(edgeKey)||null,start=db.nodes.get(String(item.startNodeId)),end=db.nodes.get(String(item.endNodeId)),open=state.isOpen!==false,n=Q.dbSimulationSnapshot?(databaseLiveEdgeCounts().get(edgeKey)||0):(performance?.instantUavCount??null),speed=n===0?null:performance?.windowSpaceMeanSpeed??null,capacity=performance?.maxCapacity??state.maxCapacity??null,utilization=n!=null&&capacity?n/capacity:null,status=open?'开放':n>0?'排空':'关闭';
    const startHeight=start?.position?.[2],endHeight=end?.position?.[2],height=startHeight==null?'暂无数据':endHeight!=null&&Math.abs(endHeight-startHeight)>.01?`${nfmt(startHeight,1)}–${nfmt(endHeight,1)}`:nfmt(startHeight,1);
    const speedLimit=performance?.speedLimit??state.speedLimit??null;
    box.dataset.object=`${rid}:${item.key}`;
    // Keep the same fields, order, labels and units as the offline detail card.
    // REGION_DB_SIMULATION: windowMinSpacingM/windowMaxRunningSpeed belong to
    // the passage-constraint and online-vs-runtime comparison chart only;
    // deliberately do not expose them in the clicked facility detail card.
    box.innerHTML=`<div class="rd-object">${pipe?'P'+item.pipeId:'RAMP'+item.rampId}</div><span class="rd-status">${pipe?(item.type==='reserve'?'备用主干':'主用主干'):'主干连接匝道'} · ${status}${utilization!=null&&utilization>=.85?' / 高负载':''}</span><dl class="rd-dl">`+
      field('区域 / 层',`${rid} / ${pipe?'L'+item.layerId:(start?.layerId==null?'连接空间':'L'+start.layerId)}`)+
      field('原始高度 · m',height)+
      field('长度 · m',nfmt(item.length,1))+field('数量 · 架',nfmt(n))+
      field('平均速度 · m/s',nfmt(speed,1))+field('60秒窗出口流量 · 架/分钟',nfmt(performance?.outFlowPerMinute))+
      field('密度 · 架/km管道',pipe?nfmt(performance?.windowDensityPerKm,1):'不适用')+
      field('最大同时容纳 · 架',nfmt(capacity))+
      field('速度上限 · m/s',nfmt(speedLimit,1))+
      field('安全距离 · m',pipe?nfmt(state.safeDistance,1):'未设正式参数')+
      (pipe?field('航道 / 管径',`${item.laneNumber??'暂无数据'}条 / ${nfmt(item.pipeWidth,1)}m`):field('几何口径',`数据库路径 / ${nfmt(item.rampWidth,1)}m管径`))+
      field('连接关系',`${item.startNodeId} → ${item.endNodeId} · 有向`,true)+
      field('采样时间',Q.dbSimulationSnapshot?.statisticsTime==null?(state.sampleTime??db.stateTime):`仿真 ${Q.dbSimulationSnapshot.statisticsTime}s`,true)+'</dl><div class="rd-small">密度按管道长度；空对象均速为 null。流量按过去60秒离开本对象的个体事件计数。性能来自本次区域DT推演的60秒滑动窗口，每5秒更新；无统计记录时保持“暂无数据”，不补零。</div>';
  }
  function renderRanks(){const list=filteredPipes();$('rdRankScope').textContent=`当前筛选 ${list.length} / 60根 · 点击定位`;
    // REGION_DB_SIMULATION: match the offline utilization convention: current
    // instantaneous UAV count divided by maximum spatial capacity.
    if(Q.dbTopology){const db=Q.dbTopology,stats=Q.dbSimulationSnapshot?.statisticsMap,liveCounts=databaseLiveEdgeCounts(),layers=[...db.layers].sort((a,b)=>Number(a.height)-Number(b.height)),selectedLayer=Q.state.layer==='all'?null:String(layers[+Q.state.layer]?.layerId??'');
      const rows=db.pipes.map(pipe=>{const key=`pipe:${Number(pipe.pipeId)}`,p=stats?.get(key),capacity=p?.maxCapacity??pipe.state?.maxCapacity??null,n=Q.dbSimulationSnapshot?(liveCounts.get(key)||0):(p?.instantUavCount??null),ratio=n!=null&&capacity?n/capacity:null,open=pipe.state?.isOpen!==false,status=open?'开放':n>0?'排空':'关闭';return{pipe,p,ratio,status};}).filter(row=>{
        const {pipe,ratio,status}=row,id=`P${pipe.pipeId}`.toUpperCase();return (selectedLayer===null||String(pipe.layerId)===selectedLayer)&&(Q.type==='all'||Q.type===pipe.type)&&(Q.open==='all'||Q.open===status)&&(Q.load==='all'||Q.load==='high'&&ratio!=null&&ratio>=.85||Q.load==='low'&&ratio!=null&&ratio<.85)&&(Q.search===''||id.includes(Q.search)||pipe.key.toUpperCase().includes(Q.search));
      }).sort((a,b)=>(b.ratio??-1)-(a.ratio??-1));
      $('rdRankScope').textContent=`瞬时容量占用率 · ${rows.length} / ${db.pipes.length}根`;
      $('rdRanks').innerHTML=rows.length?rows.map(({pipe,ratio,status})=>{const value=ratio==null?null:ratio*100,col=ratio==null?'#93928C':C.colors[ratio<.45?0:ratio<.65?1:ratio<.85?2:3];return `<button class="rd-rank ${Q.dbSelected===pipe.key?'active':''}" data-db-object="${pipe.key}" aria-label="定位 P${pipe.pipeId}"><span class="rd-rank-line"><span>P${pipe.pipeId} · ${pipe.type==='reserve'?'备':'主'} · ${status}</span><span>${value==null?'暂无数据':nfmt(value,1)+'%'}</span></span><span class="rd-rank-bar"><i style="width:${Math.min(100,value||0)}%;background:${col}"></i></span></button>`;}).join(''):'<div class="rd-empty">筛选范围内暂无管道</div>';return;}
    const rank=list.map(o=>({o,a:Q.snap.states.get(o.id)})).sort((x,y)=>(y.a.u??-1)-(x.a.u??-1));
    $('rdRanks').innerHTML=rank.length?rank.map(({o,a})=>`<button class="rd-rank ${Q.selected===o.id?'active':''}" data-object="${o.id}" aria-label="定位 ${o.id}"><span class="rd-rank-line"><span>${o.id} · ${o.reserve?'备':'主'} · ${a.missing?'缺失':a.status}</span><span>${a.u==null?'暂无数据':nfmt(a.u*100)+'%'}</span></span><span class="rd-rank-bar"><i style="width:${Math.min(100,(a.u||0)*100)}%;background:${color(a,'utilization')}"></i></span></button>`).join(''):'<div class="rd-empty">筛选范围内暂无对象</div>';
  }
  function objectVisible(o){
    const target=objects.get(Q.selected),targetLayer=['in','out'].includes(target?.kind)?target.l??0:0,targetPipe=['in','out'].includes(target?.kind)?target.p??0:0;
    if(o.kind==='free')return Q.show.free;
    if(o.kind==='takeoff'||o.kind==='landing')return Q.show.funnel&&(Q.focus==='overview'||(Q.focus==='in')===(o.kind==='takeoff'));
    if(o.kind==='in'||o.kind==='out')return Q.show.guide&&(Q.focus==='overview'||(Q.focus==='in')===(o.kind==='in'))&&(o.l==null||Q.state.layer==='all'||o.l===+Q.state.layer)&&(Q.focus==='overview'||o.l==null||o.l===targetLayer&&o.p===targetPipe);
    if(o.kind==='ramp')return Q.show.ramp&&(Q.state.layer==='all'||o.l===+Q.state.layer||o.targetLayer===+Q.state.layer)&&(Q.focus==='overview'||Q.focus==='in'&&targetLayer===0&&targetPipe===0&&o.id==='X-L1-L2');
    return filteredPipes().includes(o)&&(Q.focus==='overview'||o.l===targetLayer&&o.p===targetPipe||Q.focus==='in'&&targetLayer===0&&targetPipe===0&&o.id==='L2-P01');
  }
  function canvasScale(el){const root=$('regionalDemo'),inside=root&&root.contains(el),outer=inside&&root.classList.contains('rd-slide')?(parseFloat(root.style.getPropertyValue('--rd-slide-scale'))||1):1;return Math.min(4,(devicePixelRatio||1)*outer);}
  function ctx(id){const el=$(id),r={width:el.clientWidth,height:el.clientHeight},dpr=canvasScale(el);if(!r.width||!r.height)return null;
    const w=Math.round(r.width*dpr),h=Math.round(r.height*dpr);if(el.width!==w||el.height!==h){el.width=w;el.height=h;}const c=el.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,r.width,r.height);return {c,w:r.width,h:r.height};}
  function viewPoint(q){const ca=Math.cos(Q.yaw),sa=Math.sin(Q.yaw);return [q[0]*ca-q[1]*sa,-(q[0]*sa+q[1]*ca)*.34-(q[2]+(Q.expanded?limit((q[2]-C.firstHeight)/C.layerPitch,0,5)*C.expansion:0))*.7];}
  function project(p,w,h){const a=viewPoint(p),camera=Q.camera||{cx:0,cy:0,scale:Math.min(w/4000,h/1850)};return {x:w*.5+(a[0]-camera.cx)*camera.scale*Q.zoom,y:h*.54+(a[1]-camera.cy)*camera.scale*Q.zoom};}
  function fitCamera(w,h){const points=[...objects.values()].filter(o=>o.kind!=='free'&&objectVisible(o)).flatMap(o=>o.samples);
    if(Q.focus!=='out')points.push(...freeIn.samples);if(Q.focus!=='in')points.push(...freeOut.samples);
    if(Q.focus==='overview')points.push([-1450,-1450,0],[1450,1450,850],[-1450,1450,0],[1450,-1450,0]);
    if(!points.length)points.push([-1000,-1000,0],[1000,1000,800]);const ps=points.map(viewPoint),xs=ps.map(p=>p[0]),ys=ps.map(p=>p[1]),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
    const padding=$('regionalDemo').classList.contains('rd-slide')?60:120;
    Q.camera={cx:(minX+maxX)/2,cy:(minY+maxY)/2,scale:Math.min((w-60)/(maxX-minX||1),(h-padding)/(maxY-minY||1))};}
  function lineDistance(p,a,b){const x=b.x-a.x,y=b.y-a.y,u=limit(((p.x-a.x)*x+(p.y-a.y)*y)/(x*x+y*y||1),0,1);return Math.hypot(p.x-a.x-u*x,p.y-a.y-u*y);}
  function hitAt(x,y){let best=null,d=11;for(const hit of Q.hits){const dd=hit.point?Math.hypot(x-hit.point.x,y-hit.point.y):lineDistance({x,y},hit.a,hit.b);if(dd<d){best=hit;d=dd;}}return best;}
  function drawScene(){if(!Q.snap)return;const start=performance.now(),v=ctx('rdScene');if(!v)return;const {c,w,h}=v,P=p=>project(p,w,h);fitCamera(w,h);Q.hits=[];Q.drawnPaths=[];
    // REGION_DB_TOPOLOGY: draw the database network with its own data-driven
    // camera because the legacy camera assumes a fixed six-layer mock model.
    if(Q.dbTopology){drawDatabaseTopology(c,w,h,start);return;}
    function poly(points,fill,stroke,width=1,dash=[]){const ps=points.map(P);c.beginPath();ps.forEach((p,i)=>i?c.lineTo(p.x,p.y):c.moveTo(p.x,p.y));c.closePath();if(fill){c.fillStyle=fill;c.fill();}if(stroke){c.strokeStyle=stroke;c.lineWidth=width;c.setLineDash(dash);c.stroke();c.setLineDash([]);}}
    const sceneLabels=[],occupied=[],sceneRect=$('rdScene').getBoundingClientRect(),scale=sceneRect.width/w;
    for(const overlay of $('rdScene').parentElement.querySelectorAll('.rd-overlay,.rd-scene-hint')){
      const r=overlay.getBoundingClientRect();occupied.push({x:(r.x-sceneRect.x)/scale,y:(r.y-sceneRect.y)/scale,width:r.width/scale,height:r.height/scale});
    }
    function label(p,s,col='#607365',size=10,dx=0,dy=0){
      if(Q.focus==='overview'&&!Q.show.labels)return;
      sceneLabels.push({point:P(p),text:s,col,size:$('regionalDemo').classList.contains('rd-slide')?Math.max(12,size):size,dx,dy});
    }
    function paintLabels(){
      Q.sceneLabels=[];
      sceneLabels.sort((a,b)=>Number(b.text.includes(Q.selected))-Number(a.text.includes(Q.selected)));
      for(const item of sceneLabels){
        const {point:q,text:s,col,size,dx,dy}=item;c.font=size+'px "Segoe UI","Microsoft YaHei",sans-serif';
        const width=c.measureText(s).width+6,height=size+6;let rect;
        for(const offsetY of [0,-20,20,-40,40,-60,60,-80,80,-100,100]){
          for(const offsetX of [0,-36,36,-72,72]){
            const candidate={x:limit(q.x+dx-3+offsetX,8,w-width-8),y:limit(q.y+dy-size-2+offsetY,8,h-height-8),width,height};
            if(!occupied.some(r=>candidate.x<r.x+r.width+4&&candidate.x+width+4>r.x&&candidate.y<r.y+r.height+4&&candidate.y+height+4>r.y)){rect=candidate;break;}
          }
          if(rect)break;
        }
        if(!rect)continue;
        occupied.push(rect);Q.sceneLabels.push({...rect,text:s});
        const target={x:limit(q.x,rect.x,rect.x+width),y:limit(q.y,rect.y,rect.y+height)};
        if(Math.hypot(q.x-target.x,q.y-target.y)>14){c.save();c.globalAlpha=.45;c.strokeStyle=col;c.lineWidth=.8;c.beginPath();c.moveTo(q.x,q.y);c.lineTo(target.x,target.y);c.stroke();c.restore();}
        c.fillStyle='#FFFEFAEB';c.fillRect(rect.x,rect.y,width,height);c.fillStyle=col;c.fillText(s,rect.x+3,rect.y+size+2);
      }
    }

    function dot(p,r,col){const q=P(p);c.beginPath();c.arc(q.x,q.y,r,0,Math.PI*2);c.fillStyle=col;c.fill();}
    function stroke(points,col,width,dash=[],id=null,opacity=1){const ps=points.map(P);c.save();c.globalAlpha=opacity;c.strokeStyle=col;c.lineWidth=width;c.lineJoin='round';c.lineCap='round';c.setLineDash(dash);c.beginPath();ps.forEach((p,i)=>i?c.lineTo(p.x,p.y):c.moveTo(p.x,p.y));c.stroke();c.restore();if(id){for(let i=1;i<ps.length;i++)Q.hits.push({id,a:ps[i-1],b:ps[i]});Q.drawnPaths.push({id,start:ps[0],end:ps.at(-1),opacity,width,color:col});}}
    function arrow(g,u,col,size=5){const p=P(g.at(u)),a=P(g.at(Math.min(1,u+.018))),ang=Math.atan2(a.y-p.y,a.x-p.x);c.save();c.translate(p.x,p.y);c.rotate(ang);c.fillStyle=col;c.beginPath();c.moveTo(size,0);c.lineTo(-size,-size*.65);c.lineTo(-size,size*.65);c.closePath();c.fill();c.restore();}
    const b=C.bounds,ground=[[b[0],b[2],0],[b[1],b[2],0],[b[1],b[3],0],[b[0],b[3],0]];
    poly(ground,'#E3E8DA20','#7A8C6A60',1,[5,5]);
    for(let x=-1500;x<=1500;x+=300){stroke([[x,-1500,0],[x,1500,0]],'#B6C3AC38',.6);stroke([[-1500,x,0],[1500,x,0]],'#B6C3AC38',.6);}
    if(Q.focus==='overview'){label([-1450,1400,0],'模拟区域边界 / 邻区 R北','#6C7C63');label([-1400,-1400,0],'底部参照网格（非自由飞空域）','#6C7C63',9);}
    if(Q.show.free){const lo=[[-1450,-1450,0],[1450,-1450,0],[1450,1450,0],[-1450,1450,0]],hi=lo.map(p=>[p[0],p[1],C.freeTop]);
      poly([lo[0],lo[1],hi[1],hi[0]],'#66A5A112','#57958A4D');poly([lo[0],lo[3],hi[3],hi[0]],'#66A5A112','#57958A4D');poly(hi,'#66A5A110','#57958A70',1,[5,3]);
      label([-1380,-1380,C.freeTop],'FF-01 自由飞空域 · 0–120m','#387B72',11);Q.hits.push({id:'FF-01',point:P([-1380,-1380,C.freeTop])});}
    if(Q.show.noFly){const nf=C.noFly,base=[[nf[0],nf[2],nf[4]],[nf[1],nf[2],nf[4]],[nf[1],nf[3],nf[4]],[nf[0],nf[3],nf[4]]],roof=base.map(p=>[p[0],p[1],nf[5]]);
    poly([base[0],base[1],roof[1],roof[0]],'#B5665522','#A8615080',1,[3,3]);poly([base[1],base[2],roof[2],roof[1]],'#B5665519','#A8615080',1,[3,3]);poly(roof,'#B566551A','#A8615080',1,[3,3]);if(Q.focus==='overview')label([nf[0],nf[2],nf[5]],'× 模拟禁飞区 · 0–850m','#9B5445',10);}
    const visible=new Set(filteredPipes().filter(objectVisible).map(o=>o.id)),contextPipes=new Set();
    if(Q.show.ramp)for(const r of ramps.filter(objectVisible)){for(const l of [r.l,r.targetLayer]){const id=`L${l+1}-P${r.p?'06':'01'}`;if(!visible.has(id))contextPipes.add(id);}}
    for(const branch of [...branchesIn,...branchesOut].filter(objectVisible))if(!visible.has(branch.pipe))contextPipes.add(branch.pipe);
    const compact=Q.focus==='overview'&&!Q.guideDetail,selected=objects.get(Q.selected),linkedPipes=new Set();
    if(selected?.kind==='pipe')linkedPipes.add(selected.id);
    else if(selected?.pipe)linkedPipes.add(selected.pipe);
    else if(selected?.kind==='ramp')for(const l of [selected.l,selected.targetLayer])linkedPipes.add(`L${l+1}-P${selected.p?'06':'01'}`);
    const related=o=>o.kind==='ramp'?[o.l,o.targetLayer].some(l=>linkedPipes.has(`L${l+1}-P${o.p?'06':'01'}`)):linkedPipes.has(o.pipe)||selected?.kind==='takeoff'&&o.kind==='in'||selected?.kind==='landing'&&o.kind==='out';
    const drawObjects=[...pipes.filter(o=>visible.has(o.id)||contextPipes.has(o.id)),...ramps.filter(objectVisible),...branchesIn.filter(objectVisible),...branchesOut.filter(objectVisible)].sort((a,b)=>(a.id===Q.selected)-(b.id===Q.selected)||a.samples[50][2]-b.samples[50][2]);
    for(const o of drawObjects){const a=Q.snap.states.get(o.id),isPipe=o.kind==='pipe',chosen=Q.selected===o.id,context=contextPipes.has(o.id),down=o.kind==='ramp'&&o.targetLayer<o.l,emphasized=chosen||related(o),col=isPipe?color(a):o.kind==='ramp'?(down?'#377F94':'#A27B29'):o.kind==='in'?'#7556A0':'#7C5895';
      const dash=a.missing?[2,5]:isPipe&&a.status==='关闭'?[4,5]:a.status==='排空'?[11,3,2,3]:o.kind==='ramp'?(down?[9,4,2,4]:[7,5]):o.kind==='out'?[10,3,2,3]:[];
      const alpha=context?.2:chosen?1:isPipe?(a.status==='关闭'?.25:o.reserve?.65:1):compact?(emphasized?.9:.35):.85,width=isPipe?(a.status==='关闭'?.7:o.reserve?2:4):compact&&!emphasized?1:1.8;
      if(chosen)stroke(o.samples,'#FFFEFA',width+7,[],null,1);stroke(o.samples,col,width+(chosen?2:0),dash,o.id,alpha);
      if(a.status!=='关闭'&&(isPipe||!compact||emphasized)){c.save();c.globalAlpha=alpha;arrow(o,.58,col,chosen?6:isPipe?3:4);c.restore();}else if(chosen&&a.status==='关闭')label(o.at(.5),'关闭 · 无个体','#70776C');
      if(isPipe){dot(o.at(0),chosen?4:2,'#FFFEFA');dot(o.at(1),chosen?4:2,col);
        if(o.p===0&&(Q.focus==='overview'||o.l===0))label(o.at(.08),`L${o.l+1} · ${C.layerAngles[o.l]}° · ${o.height}m${context?' 连接上下文':''}`,'#4F695A',10,0,-8);
      }
      if(chosen)label(o.at(.53),`${o.id} · ${a.missing?'缺失':a.status} · ${nfmt(a.n)}架`,col,11,7,-12);
    }
    for(const [id,center,col] of [['TK-01',C.takeoff,'#2C827B'],['LD-01',C.landing,'#B16B4F']]){const o=objects.get(id);if(!objectVisible(o))continue;const bottom=Array.from({length:33},(_,i)=>[center[0]+Math.cos(i/32*2*Math.PI)*C.funnelBottomRadius,center[1]+Math.sin(i/32*2*Math.PI)*C.funnelBottomRadius,C.funnelBottom]),top=bottom.map((p,i)=>[center[0]+Math.cos(i/32*2*Math.PI)*C.funnelTopRadius,center[1]+Math.sin(i/32*2*Math.PI)*C.funnelTopRadius,C.funnelTop]);
      for(let i=0;i<32;i++)poly([bottom[i],bottom[i+1],top[i+1],top[i]],col+'10');stroke(bottom,col,1.7,[],id);stroke(top,col,1.7,[],id);for(const i of [0,8,16,24])stroke([bottom[i],top[i]],col+'A0',1,[],id);stroke(o.samples,col,Q.selected===id?5:2,[],id);arrow(o,.5,col,6);label([...center,C.funnelBottom],`${id} ${o.name}`,col,11,-34,23);Q.hits.push({id,point:P([...center,75])});}
    if(Q.show.free){if(Q.focus!=='out')stroke(freeIn.samples,'#2C827B8C',1.4,[2,5]);if(Q.focus!=='in')stroke(freeOut.samples,'#B16B4F8C',1.4,[2,5]);}
    if(Q.show.guide){if(Q.focus!=='out')label(takeTop,'汇入引路 · 直达目标层','#7556A0',10,-45,-9);if(Q.focus!=='in')label(landTop,'汇出引路 · 目标层直达漏斗','#7556A0',10,-35,-9);}
    let drawn=0,eligible=0;
    for(const v of Q.snap.vehicles){const o=objects.get(v.objectId);if(!objectVisible(o))continue;if(Q.focus==='in'&&v.stream==='SIM-OUT'||Q.focus==='out'&&v.stream==='SIM-IN')continue;eligible++;
      const p=P(v.position);if(p.x<0||p.x>w||p.y<0||p.y>h)continue;drawn++;
      if(v.waiting){dot(v.position,3.5,'#BF7435');continue;}
      const tracked=(v.stream==='SIM-IN'&&v.k===Q.watched.in)||(v.stream==='SIM-OUT'&&v.k===Q.watched.out),a=P(v.ahead),ang=Math.atan2(a.y-p.y,a.x-p.x),size=tracked?6:2.7,col=v.stage==='起飞'||v.stage==='自由飞'?'#267D75':v.stage==='降落'?'#B16B4F':v.stage==='层间转移'?(o.targetLayer<o.l?'#377F94':'#9D7721'):v.stage==='汇出'||v.stage==='汇入'?'#7556A0':'#305F50';
      c.save();c.translate(p.x,p.y);c.rotate(ang);c.beginPath();c.moveTo(size+2,0);c.lineTo(-size,-size*.65);c.lineTo(-size*.4,0);c.lineTo(-size,size*.65);c.closePath();c.fillStyle=col;c.strokeStyle='#FFFEFA';c.lineWidth=tracked?1.5:.6;c.fill();c.stroke();c.restore();if(tracked)label(v.position,`${v.stream==='SIM-IN'?'汇入组':'汇出组'} · ${v.stage}`,col,10,10,-9);
    }
    // Enlarged local inset explains the funnel proportions without altering world heights.
    if(Q.focus!=='overview'&&Q.show.funnel){const incoming=Q.focus==='in',id=incoming?'TK-01':'LD-01',col=incoming?'#2C827B':'#B16B4F',bw=Math.min(205,w-30),bh=128,x=14,y=h-bh-37;
      occupied.push({x,y,width:bw,height:bh});
      c.fillStyle='#FFFEFAF5';c.strokeStyle='#D7DECF';c.lineWidth=1;c.fillRect(x,y,bw,bh);c.strokeRect(x,y,bw,bh);c.fillStyle=col;c.font='11px "Microsoft YaHei"';c.fillText(`${id} ${incoming?'起飞汇聚':'降落分散'} · 局部放大`,x+9,y+17);
      const cx=x+bw*.37,top=y+35,bottom=y+97;c.beginPath();c.moveTo(cx-13,top);c.lineTo(cx+13,top);c.lineTo(cx+47,bottom);c.lineTo(cx-47,bottom);c.closePath();c.fillStyle=col+'18';c.fill();c.strokeStyle=col;c.stroke();
      c.beginPath();c.ellipse(cx,top,13,4,0,0,Math.PI*2);c.stroke();c.beginPath();c.ellipse(cx,bottom,47,8,0,0,Math.PI*2);c.stroke();c.fillStyle=col;c.font='10px "Microsoft YaHei"';c.fillText('110m · Φ44m',x+bw*.63,y+40);c.fillText('40m · Φ180m',x+bw*.63,y+94);c.fillText('展示假设 · 非固定参数',x+9,y+119);
      const fy=incoming?bottom-25:top+25;c.beginPath();c.moveTo(cx,fy+(incoming?-15:15));c.lineTo(cx-4,fy+(incoming?-8:8));c.lineTo(cx+4,fy+(incoming?-8:8));c.closePath();c.fill();c.beginPath();c.moveTo(cx,fy-10);c.lineTo(cx,fy+10);c.stroke();
    }
    paintLabels();Q.visible=drawn;Q.eligible=eligible;
    const desc=`当前可见 ${drawn} / 显示开关内 ${eligible} / 区域完整 ${Q.snap.vehicles.length} 架；未抽样，裁剪与隐藏不改变完整统计。`;
    $('rdScene').setAttribute('aria-description',desc);$('rdVisible').textContent=(Q.countText||'')+' '+desc;
    Q.renderMs=performance.now()-start;
  }

  // REGION_DB_TOPOLOGY: topology-only renderer for the region-1 database test.
  // It intentionally does not manufacture UAVs, utilization or traffic stats.
  function drawDatabaseTopology(c,w,h,start){
    const db=Q.dbTopology,layers=[...db.layers].sort((a,b)=>Number(a.height)-Number(b.height));
    const layerIndex=new Map(layers.map((layer,index)=>[String(layer.layerId),index]));
    const selectedLayer=Q.state.layer==='all'?null:String(layers[+Q.state.layer]?.layerId??'');
    const pipes=db.pipes.filter(pipe=>selectedLayer===null||String(pipe.layerId)===selectedLayer);
    const ramps=db.ramps.filter(ramp=>{if(ramp.type==='funnel')return Q.show.guide;if(!Q.show.ramp)return false;if(selectedLayer===null)return true;const a=db.nodes.get(String(ramp.startNodeId)),b=db.nodes.get(String(ramp.endNodeId));return String(a?.layerId)===selectedLayer||String(b?.layerId)===selectedLayer;});
    const all=[...pipes,...ramps].flatMap(o=>o.points);
    if(!all.length){c.fillStyle='#6B746B';c.font='14px "Microsoft YaHei"';c.fillText('当前层没有可绘制的数据库路网',24,42);return;}

    // Match the legacy scene's yaw, pseudo-3D inclination and layer expansion.
    const ca=Math.cos(Q.yaw),sa=Math.sin(Q.yaw),expandedHeight=z=>{if(!Q.expanded)return z;let nearest=0,best=Infinity;for(let i=0;i<layers.length;i++){const d=Math.abs(z-Number(layers[i].height));if(d<best){best=d;nearest=i;}}return z+nearest*C.expansion;};
    const raw=p=>[p[0]*ca-p[1]*sa,-(p[0]*sa+p[1]*ca)*.34-expandedHeight(p[2])*.7];
    const view=all.map(raw),xs=view.map(p=>p[0]),ys=view.map(p=>p[1]),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),pad=Q.focus==='overview'?70:55,scale=Math.min((w-pad*2)/(maxX-minX||1),(h-pad*2)/(maxY-minY||1))*Q.zoom,cx=(minX+maxX)/2,cy=(minY+maxY)/2;
    const P=p=>{const q=raw(p);return{x:w/2+(q[0]-cx)*scale,y:h/2+(q[1]-cy)*scale};};
    const stroke=(points,color,width,dash=[],opacity=1)=>{const ps=points.map(P);c.save();c.globalAlpha=opacity;c.beginPath();ps.forEach((p,i)=>i?c.lineTo(p.x,p.y):c.moveTo(p.x,p.y));c.strokeStyle=color;c.lineWidth=width;c.lineJoin='round';c.lineCap='round';c.setLineDash(dash);c.stroke();c.restore();};
    const polygon=(points,fill,outline)=>{const ps=points.map(P);c.beginPath();ps.forEach((p,i)=>i?c.lineTo(p.x,p.y):c.moveTo(p.x,p.y));c.closePath();if(fill){c.fillStyle=fill;c.fill();}if(outline){c.strokeStyle=outline;c.lineWidth=1;c.stroke();}};
    const addHit=(key,points)=>{const ps=points.map(P);for(let i=1;i<ps.length;i++)Q.hits.push({dbKey:key,a:ps[i-1],b:ps[i]});};
    const dot=(point,radius,color)=>{const p=P(point);c.beginPath();c.arc(p.x,p.y,radius,0,Math.PI*2);c.fillStyle=color;c.fill();};
    const arrow=(points,color,opacity=1)=>{if(points.length<2)return;const index=Math.max(1,Math.floor(points.length*.58)),a=P(points[index-1]),b=P(points[Math.min(points.length-1,index)]),angle=Math.atan2(b.y-a.y,b.x-a.x),x=(a.x+b.x)/2,y=(a.y+b.y)/2;c.save();c.globalAlpha=opacity;c.translate(x,y);c.rotate(angle);c.fillStyle=color;c.beginPath();c.moveTo(4,0);c.lineTo(-3,-2.3);c.lineTo(-3,2.3);c.closePath();c.fill();c.restore();};

    // Same subtle reference grid as the offline overview. Lanes are deliberately
    // not drawn individually: the offline scene also renders one line per pipe.
    const range=db.range?.length===4?db.range:[Math.min(...all.map(p=>p[0])),Math.max(...all.map(p=>p[0])),Math.min(...all.map(p=>p[1])),Math.max(...all.map(p=>p[1]))],z0=Math.min(0,...all.map(p=>p[2])),ground=[[range[0],range[2],z0],[range[1],range[2],z0],[range[1],range[3],z0],[range[0],range[3],z0]];
    stroke([...ground,ground[0]],'#7A8C6A60',1,[5,5]);
    const gridStep=Math.max(100,Math.ceil(Math.max(range[1]-range[0],range[3]-range[2])/10/100)*100);
    for(let x=Math.ceil(range[0]/gridStep)*gridStep;x<=range[1];x+=gridStep)stroke([[x,range[2],z0],[x,range[3],z0]],'#B6C3AC38',.6);
    for(let y=Math.ceil(range[2]/gridStep)*gridStep;y<=range[3];y+=gridStep)stroke([[range[0],y,z0],[range[1],y,z0]],'#B6C3AC38',.6);

    Q.hits=[];Q.drawnPaths=[];
    const ordered=[...pipes].sort((a,b)=>a.points[0][2]-b.points[0][2]);
    const simulated=Q.dbSimulationSnapshot,liveCounts=databaseLiveEdgeCounts(),metricState=(type,id,fallbackCapacity)=>{if(!simulated)return null;const key=`${type}:${Number(id)}`,s=simulated.statisticsMap.get(key),n=liveCounts.get(key)||0,capacity=s?.maxCapacity??fallbackCapacity??null;return {n,speed:s?.windowSpaceMeanSpeed??null,density:s?.windowDensityPerKm??null,u:capacity?n/capacity:null,status:'开放',missing:false};};
    for(const pipe of ordered){const open=pipe.state?.isOpen!==false,reserve=pipe.type==='reserve',state=metricState('pipe',pipe.pipeId,pipe.state?.maxCapacity),pipeColor=open?(state?color(state):'#93928C'):'#A4AAA0',width=open?(reserve?2:4):.7,alpha=open?(reserve?0.65:1):0.25,dash=open?[]:[4,5],chosen=Q.dbSelected===pipe.key;if(chosen)stroke(pipe.points,'#FFFEFA',width+7);stroke(pipe.points,pipeColor,width+(chosen?2:0),dash,alpha);if(open)arrow(pipe.points,pipeColor,alpha);dot(pipe.points[0],chosen?4:2,'#FFFEFA');dot(pipe.points.at(-1),chosen?4:2,pipeColor);addHit(pipe.key,pipe.points);}
    for(const ramp of ramps){const start=ramp.points[0],end=ramp.points.at(-1),down=end[2]<start[2],funnel=ramp.type==='funnel',same=ramp.type==='same-layer-reverse',color=funnel?(ramp.funnelFlow==='landing'?'#7C5895':'#7556A0'):down?'#377F94':same?'#7C5895':'#A27B29',open=ramp.state?.isOpen!==false,dash=open?(funnel?(ramp.funnelFlow==='landing'?[10,3,2,3]:[]):down?[9,4,2,4]:same?[10,3,2,3]:[7,5]):[4,5],alpha=open?0.85:0.25,chosen=Q.dbSelected===ramp.key;if(chosen)stroke(ramp.points,'#FFFEFA',8);stroke(ramp.points,color,1.8+(chosen?2:0),dash,alpha);if(open)arrow(ramp.points,color,alpha);addHit(ramp.key,ramp.points);}
    if(Q.show.funnel)for(const funnel of db.funnels){const color=funnel.type==='takeoff'?'#2C827B':'#B16B4F',bottom=Array.from({length:33},(_,i)=>[funnel.bottom[0]+Math.cos(i/32*2*Math.PI)*C.funnelBottomRadius,funnel.bottom[1]+Math.sin(i/32*2*Math.PI)*C.funnelBottomRadius,funnel.bottom[2]]),top=Array.from({length:33},(_,i)=>[funnel.top[0]+Math.cos(i/32*2*Math.PI)*C.funnelTopRadius,funnel.top[1]+Math.sin(i/32*2*Math.PI)*C.funnelTopRadius,funnel.top[2]]),chosen=Q.dbSelected===funnel.key;
      for(let i=0;i<32;i++)polygon([bottom[i],bottom[i+1],top[i+1],top[i]],color+'10',chosen?'#FFFEFA':null);stroke(bottom,color,chosen?4:1.7);stroke(top,color,chosen?4:1.7);for(const i of [0,8,16,24])stroke([bottom[i],top[i]],color+'A0',1);const center=P([(funnel.bottom[0]+funnel.top[0])/2,(funnel.bottom[1]+funnel.top[1])/2,(funnel.bottom[2]+funnel.top[2])/2]);Q.hits.push({dbKey:funnel.key,point:center});if(Q.show.labels){c.fillStyle=color;c.font='10px "Microsoft YaHei"';c.fillText(`${funnel.type==='takeoff'?'TK':'LD'}-${funnel.nodeId}`,center.x+8,center.y-8);}}
    // REGION_DB_TOPOLOGY: shared boundary nodes remain in the topology model
    // for portal/handoff semantics, but are intentionally not rendered.
    // REGION_DT_SIMULATION_RESULT: render real UAVs with the same directional
    // arrow glyph used by the offline scene. Direction follows the database
    // edge's start-to-end geometry. All active UAVs use one arrow shape,
    // including UAVs whose current speed is zero.
    const edgeDirection=(position,points)=>{let best=null,bestDistance=Infinity;for(let i=1;i<points.length;i++){const a=points[i-1],b=points[i],d=b.map((value,j)=>value-a[j]),length2=d.reduce((sum,value)=>sum+value*value,0);if(length2<=1e-9)continue;const u=limit(position.reduce((sum,value,j)=>sum+(value-a[j])*d[j],0)/length2,0,1),nearest=a.map((value,j)=>value+d[j]*u),distance=position.reduce((sum,value,j)=>sum+(value-nearest[j])**2,0);if(distance<bestDistance){bestDistance=distance;const pa=P(a),pb=P(b);best=Math.atan2(pb.y-pa.y,pb.x-pa.x);}}return best;};
    const drawnEdgeKeys=new Set([...pipes,...ramps].map(edge=>edge.key));let uavCount=0;
    for(const uav of simulated?.uavs||[]){if(!uav.active||uav.position.some(value=>!Number.isFinite(value)))continue;const edgeId=uav.edgeType==='pipe'?uav.pipeId:uav.rampId,edge=db.objects.get(`${uav.edgeType}:${edgeId}`);if(!edge||!drawnEdgeKeys.has(edge.key))continue;const p=P(uav.position);if(p.x<0||p.x>w||p.y<0||p.y>h)continue;uavCount++;
      const angle=edgeDirection(uav.position,edge.points)??0;
      const down=edge.type==='inter-layer'&&edge.points.at(-1)[2]<edge.points[0][2],color=uav.edgeType==='pipe'?'#305F50':edge.type==='funnel'?(edge.funnelFlow==='landing'?'#B16B4F':'#267D75'):edge.type==='inter-layer'?(down?'#377F94':'#9D7721'):'#7556A0',size=2.7;
      c.save();c.translate(p.x,p.y);c.rotate(angle);c.beginPath();c.moveTo(size+2,0);c.lineTo(-size,-size*.65);c.lineTo(-size*.4,0);c.lineTo(-size,size*.65);c.closePath();c.fillStyle=color;c.strokeStyle='#FFFEFA';c.lineWidth=.6;c.fill();c.stroke();c.restore();
    }

    if(Q.show.labels){const labeled=new Set();for(const pipe of ordered){if(labeled.has(pipe.layerId))continue;labeled.add(pipe.layerId);const layer=layers.find(value=>String(value.layerId)===String(pipe.layerId)),p=P(pipe.points[0]);c.fillStyle='#4F695A';c.font='10px "Microsoft YaHei"';c.fillText(`L${pipe.layerId} · ${nfmt(layer?.angle)}° · ${nfmt(layer?.height)}m`,p.x+5,p.y-7);}}
    c.fillStyle='#40584B';c.font='600 13px "Microsoft YaHei"';c.fillText(`${regionId()} 数据库路网 · topology v${db.topologyVersion}`,18,26);c.font='11px "Microsoft YaHei"';c.fillStyle='#66736B';c.fillText(`${db.nodes.size}节点 · ${db.pipes.length}管道 · ${db.ramps.length}匝道 · ${db.funnels.length}漏斗 · ${db.lanes.length}航道`,18,44);
    Q.visible=0;Q.eligible=0;Q.sceneLabels=[];
    const desc=`区域${db.regionId}数据库真实拓扑：${db.nodes.size}节点、${db.pipes.length}管道、${db.ramps.length}匝道、${db.funnels.length}漏斗、${db.lanes.length}航道。`,performanceCount=simulated?.statistics.length||0;
    $('rdScene').setAttribute('aria-description',desc);$('rdVisible').textContent=simulated?`DT推演 ${simulated.runId} · ${simulated.frameTime}s · 当前区域 ${uavCount} 架 · ${performanceCount}个路段具有60秒滑动统计。`:desc+' 当前没有可回放的DT推演CSV。';Q.renderMs=performance.now()-start;
  }
  const metricInfo={count:['数量','架',a=>a.n],speed:['平均速度','m/s',a=>a.speed],flow:['出口流量','架/分钟',a=>a.flow],density:['密度','架/km管道',a=>a.density]};
  function rememberDatabaseStatistics(snapshot){
    if(snapshot?.statisticsTime==null||!snapshot.statisticsMap)return;
    Q.dbSimulationStatsHistory.set(Number(snapshot.statisticsTime),snapshot.statisticsMap);
    // A 15-minute run sampled every five seconds needs only 181 entries. Keep
    // a modest safety margin if a longer test is opened in the same page.
    while(Q.dbSimulationStatsHistory.size>400)Q.dbSimulationStatsHistory.delete(Q.dbSimulationStatsHistory.keys().next().value);
  }
  function chart(id,series,{from,to,unit='',rightUnit='',max=null,rightMax=null,forecastAt=null,axes=false,xLabel='',empty=''}={}){
    const el=$(id),stamp=JSON.stringify([from,to,unit,rightUnit,max,rightMax,forecastAt,axes,xLabel,el?.clientWidth,el?.clientHeight,devicePixelRatio||1]),cached=chartCache.get(id);if(cached?.series===series&&cached.stamp===stamp)return;chartCache.set(id,{series,stamp});
    const v=ctx(id);if(!v)return;const {c,w,h}=v,L=44,R=rightUnit?40:xLabel?32:15,T=23,B=xLabel?44:29,ww=w-L-R,hh=h-T-B;
    const values=series.filter(s=>!s.right).flatMap(s=>s.data.map(p=>p.y).filter(y=>y!=null)),rvalues=series.filter(s=>s.right).flatMap(s=>s.data.map(p=>p.y).filter(y=>y!=null));
    max=max??Math.max(1,...values)*1.18;rightMax=rightMax??Math.max(1,...rvalues)*1.18;
    const X=t=>L+(t-from)/(to-from||1)*ww,Y=(y,right)=>T+hh-(y/(right?rightMax:max))*hh;
    c.font=($('regionalDemo').classList.contains('rd-slide')?12:10)+'px "Segoe UI","Microsoft YaHei",sans-serif';c.fillStyle='#647169';c.fillText(unit,7,12);if(rightUnit){c.textAlign='right';c.fillText(rightUnit,w-7,12);c.textAlign='left';}
    if(empty&&!values.length&&!rvalues.length){c.textAlign='center';c.fillText(empty,w/2,h/2);c.textAlign='left';return;}
    for(let i=0;i<4;i++){const y=T+hh*i/3;c.beginPath();c.moveTo(L,y);c.lineTo(w-R,y);c.strokeStyle='#E5E5DA';c.lineWidth=1;c.stroke();c.fillStyle='#647169';c.textAlign='right';c.fillText(nfmt(max*(1-i/3),max<10?1:0),L-7,y+3);if(rightUnit){c.textAlign='left';c.fillText(nfmt(rightMax*(1-i/3),0),w-R+5,y+3);}}
    if(forecastAt!=null){c.fillStyle='#7556A009';c.fillRect(X(forecastAt),T,X(to)-X(forecastAt),hh);c.strokeStyle='#A79CB9';c.setLineDash([3,4]);c.beginPath();c.moveTo(X(forecastAt),T);c.lineTo(X(forecastAt),T+hh);c.stroke();c.setLineDash([]);}
    if(axes){c.beginPath();c.moveTo(L,T);c.lineTo(L,T+hh);c.lineTo(w-R,T+hh);c.strokeStyle='#9BAA9F';c.lineWidth=1;c.stroke();}
    for(const s of series){c.beginPath();c.strokeStyle=s.color;c.lineWidth=s.width||1.9;c.setLineDash(s.dash||[]);let pen=false;for(const p of s.data){if(p.y==null){pen=false;continue;}const x=X(p.t),y=Y(p.y,s.right);if(!pen){c.moveTo(x,y);pen=true;}else c.lineTo(x,y);}c.stroke();c.setLineDash([]);if(s.points){c.fillStyle=s.color;for(const p of s.data){if(p.y==null)continue;c.beginPath();c.arc(X(p.t),Y(p.y,s.right),1.7,0,Math.PI*2);c.fill();}}}
    c.fillStyle='#647169';c.textAlign='center';for(const t of [from,(from+to)/2,to])c.fillText(t<0?'开始前':clock(t).slice(0,8),X(t),h-(xLabel?23:7));if(xLabel)c.fillText(xLabel,L+ww/2,h-5);c.textAlign='left';
  }
  function drawDatabaseCharts(){
    const item=Q.dbTopology?.objects.get(Q.dbSelected),pipe=item?.key?.startsWith('pipe:'),ramp=item?.key?.startsWith('ramp:'),edgeType=pipe?'pipe':ramp?'ramp':null,edgeId=Number(pipe?item.pipeId:ramp?item.rampId:NaN),state=item?.state||{},now=Number(Q.dbSimulationSnapshot?.statisticsTime??Q.dbSimulationSnapshot?.frameTime??0),historyFrom=now-180,historyFloor=Math.max(0,historyFrom),historyTo=now+180;
    const observations=[];
    if(edgeType)for(const [time,map] of [...Q.dbSimulationStatsHistory].sort((a,b)=>a[0]-b[0])){if(time<historyFloor||time>now)continue;const value=map.get(`${edgeType}:${edgeId}`);if(value)observations.push({t:time,value});}
    const metric={count:['数量','架',v=>v.instantUavCount],speed:['平均速度','m/s',v=>v.windowSpaceMeanSpeed],flow:['出口流量','架/分钟',v=>v.outFlowPerMinute],density:['密度','架/km管道',v=>pipe?v.windowDensityPerKm:null]}[Q.chart];
    const history=observations.map(point=>({t:point.t,y:metric[2](point.value)})),current=history.at(-1)?.y??null,prediction=Array.from({length:7},(_,i)=>({t:now+i*30,y:current==null?null:Math.max(0,current*(1+.07*Math.sin(i*.6)))}));
    $('rdChartTitle').textContent=`${item?(pipe?'P'+item.pipeId:ramp?'RAMP'+item.rampId:'所选对象'):'未选对象'} · ${metric[0]}历史 / 预测`;
    chart('rdHistory',[{data:history,color:'#387D69',points:true},{data:Q.show.prediction?prediction:[],color:'#7556A0',dash:[6,4]}],{from:historyFrom,to:historyTo,unit:metric[1],forecastAt:now,empty:'当前对象暂无DT历史统计'});
    $('rdForecastNote').textContent=`起报 ${clock(now)} · 历史 / 预测各3分钟 · 历史为区域DT统计，预测暂沿用离线演示`;
    const from=Math.max(0,now-600);
    const safeDistance=pipe?state.safeDistance:null,speedLimit=observations.map(p=>p.value.speedLimit).findLast?.(v=>v!=null)??observations.map(p=>p.value.speedLimit).filter(v=>v!=null).at(-1)??state.speedLimit??null;
    const runtime=observations.map(point=>({t:point.t,gap:point.value.windowMinSpacingM,maxSpeed:point.value.windowMaxRunningSpeed}));
    const distanceMax=Math.ceil(Math.max(1,safeDistance??0,...runtime.map(v=>v.gap??0))*1.18/20)*20,speedMax=Math.ceil(Math.max(1,speedLimit??0,...runtime.map(v=>v.maxSpeed??0))*1.18/5)*5;
    chart('rdLimits',[
      {color:'#AC7B2C',dash:[5,4],data:safeDistance==null?[]:[{t:from,y:safeDistance},{t:Math.max(now,from+1),y:safeDistance}]},
      {color:'#AC7B2C',data:runtime.map(v=>({t:v.t,y:v.gap})),points:true},
      {color:'#316E91',dash:[5,4],right:true,data:speedLimit==null?[]:[{t:from,y:speedLimit},{t:Math.max(now,from+1),y:speedLimit}]},
      {color:'#316E91',right:true,data:runtime.map(v=>({t:v.t,y:v.maxSpeed})),points:true}
    ],{from,to:Math.max(now,from+1),unit:'纵向间距 m',rightUnit:'速度 m/s',max:distanceMax,rightMax:speedMax,empty:'当前对象暂无通行约束统计'});
    const last=runtime.at(-1);$('rdLimitsNote').textContent=`当前间距 ${nfmt(last?.gap,1)} m · 最高速度 ${nfmt(last?.maxSpeed,1)} m/s`;
  }
  function constraintObservation(o,snap){
    if(!o||o.kind!=='pipe'||snap.states.get(o.id)?.missing)return {gap:null,maxSpeed:null};
    const vs=snap.vehicles.filter(v=>v.objectId===o.id&&!v.waiting),a=o.at(0),b=o.at(1),d=b.map((v,i)=>v-a[i]),len=Math.hypot(...d);
    const distances=vs.map(v=>v.position.reduce((sum,x,i)=>sum+(x-a[i])*d[i]/len,0)).sort((x,y)=>x-y);
    return {gap:distances.length<2?null:Math.min(...distances.slice(1).map((v,i)=>v-distances[i])),maxSpeed:vs.length?Math.max(...vs.map(v=>v.speed)):null};
  }
  function drawCharts(){const start=performance.now(),t=Q.snap.t,from=t-180,to=t+180,o=objects.get(Q.selected),info=metricInfo[Q.chart];
    $('rdChartTitle').textContent=(o?o.id:'未选对象')+' · '+info[0]+'历史 / 预测（模拟）';
    if(o){const hist=[];for(let tt=Math.max(0,from);tt<t;tt+=C.historyStep)hist.push({t:tt,y:info[2](stats(o,tt,activeVehicles(tt)))});hist.push({t,y:info[2](Q.snap.states.get(o.id))});
      const current=hist.at(-1).y,pred=Array.from({length:7},(_,i)=>({t:t+i*30,y:current==null?null:Math.max(0,current*(1+.07*Math.sin(i*.6)))}));
      chart('rdHistory',[{data:hist,color:'#387D69'},{data:Q.show.prediction?pred:[],color:'#7556A0',dash:[6,4]}],{from,to,unit:info[1],forecastAt:t});
    }else chart('rdHistory',[],{from,to,unit:info[1],forecastAt:t});
    $('rdForecastNote').textContent=`起报 ${clock(t)} · 历史 / 预测各3分钟`;
    const limitFrom=Math.max(0,t-600),times=[];for(let tt=limitFrom;tt<t;tt+=30)times.push(tt);times.push(t);
    const observed=times.map(tt=>({t:tt,...(!o||o.kind!=='pipe'?{gap:null,maxSpeed:null}:constraintObservation(o,objectSnapshot(o,tt)))}));
    const runtime=observed;
    // Round each physical axis in its own units so constant bounds do not normalize onto one line.
    const distanceMax=Math.ceil(Math.max(1,...times.map(tt=>limits(tt).safeDistance),...runtime.map(v=>v.gap??0))*1.18/20)*20;
    const speedMax=Math.ceil(Math.max(1,...times.map(tt=>limits(tt).speedLimit),...runtime.map(v=>v.maxSpeed??0))*1.18/5)*5;
    chart('rdLimits',[
      {color:'#AC7B2C',dash:[5,4],data:o?.kind==='pipe'?times.map(tt=>({t:tt,y:limits(tt).safeDistance})):[]},
      {color:'#AC7B2C',data:runtime.map(v=>({t:v.t,y:v.gap}))},
      {color:'#316E91',dash:[5,4],right:true,data:o?.kind==='pipe'?times.map(tt=>({t:tt,y:limits(tt).speedLimit})):[]},
      {color:'#316E91',right:true,data:runtime.map(v=>({t:v.t,y:v.maxSpeed}))}
    ],{from:limitFrom,to:t||1,unit:'纵向间距 m',rightUnit:'速度 m/s',max:distanceMax,rightMax:speedMax});
    const last=runtime.at(-1);
    $('rdLimitsNote').textContent=`当前间距 ${nfmt(last?.gap,1)} m · 最高速度 ${nfmt(last?.maxSpeed,1)} m/s`;
    Q.chartMs=performance.now()-start;
  }
  function frame(now,state){if(!Q.ready||Q.state!==state)render(state);const dt=Math.min(.1,Math.max(0,(now-Q.last)/1000));Q.last=now;
    if(document.hidden)return;if(RegionProjection.active())state.playing=false;
    if(state.playing){const end=playbackEndSeconds()/60;state.t=Math.min(end,state.t+dt*Q.rate/60);if(state.t===end)state.playing=false;
      if(Q.dbTopology&&Q.dbSimulation)loadDatabaseSimulationSnapshot();
      if(now-(Q.lastDraw||0)>=32){Q.lastDraw=now;Q.snap=snapshot(state.t*60);drawScene();Q.frames.push(now);while(Q.frames.length&&now-Q.frames[0]>2000)Q.frames.shift();Q.fps=Q.frames.length>2?(Q.frames.length-1)*1000/(now-Q.frames[0]):null;}if(!state.playing){Q.lastUI=now;refresh(true);}}
    if(state.playing&&now-Q.lastUI>=250){Q.lastUI=now;refresh();}
  }
  function fitPresentation(){
    const scale=Math.min(innerWidth/1920,innerHeight/1080),el=$('regionalDemo');
    el.style.setProperty('--rd-slide-scale',scale);el.style.setProperty('--rd-slide-left',(innerWidth-1920*scale)/2+'px');el.style.setProperty('--rd-slide-top',(innerHeight-1080*scale)/2+'px');chartCache.clear();
  }
  function presentation(enabled){
    $('regionalDemo').classList.toggle('rd-slide',enabled);document.body.classList.toggle('region-slide',enabled);
    $('rdViewToggle').textContent=enabled?'返回详情视图':'切换到16:9全览';$('rdViewToggle').setAttribute('aria-pressed',String(enabled));
    if(enabled)fitPresentation();requestAnimationFrame(()=>{resize();drawCharts();});
  }
  function resize(){if(Q.ready&&Q.state.view==='region'){drawScene();if(!RegionProjection.active())drawCharts();RegionEvaluation.refresh();}}
  // Read-only hooks inspect the same model used by the scene and charts.
  function inspect(t=Q.state.t*60){const snap=arguments.length?snapshot(t):(RegionProjection.snapshot()||snapshot(t)),list=filteredPipes(snap);return {t:snap.t,currentTime:Q.state.t*60,region:regionId(),scenario:Q.state.scenario,selected:Q.selected,labels:Q.sceneLabels||[],vehicles:snap.vehicles,states:Object.fromEntries(snap.states),scope:aggregateScope(list,snap),completion:snap.completion||completion(t),visible:Q.visible,paths:Q.drawnPaths,config:C};}
  const centerModels=new Map(),centerCache=new Map();
  function centerSample(r,l,t,scenario){const key=r+':'+scenario,time=t*60,cacheKey=key+':'+time;let groups=centerCache.get(cacheKey);
    if(!groups){let model=centerModels.get(key);if(!model){model=makeModel(r,scenario);centerModels.set(key,model);}const vs=activeVehicles(time,model);groups=[];
      for(let layer=0;layer<6;layer++){let n=0,weighted=0,capacity=0,flow=0,open=0;for(const o of pipes.filter(p=>p.l===layer)){const a=stats(o,time,vs,model);if(a.status!=='关闭')open++;if(a.missing)continue;if(a.status!=='关闭')capacity+=o.capacity;n+=a.n;weighted+=(a.speed??0)*a.n;flow+=a.flow??0;}groups.push({n,capacity,u:capacity?n/capacity:0,speed:n?weighted/n:0,flow:time<60?null:flow,reserve:open>2,open});}
      if(centerCache.size>256)centerCache.clear();centerCache.set(cacheKey,groups);
    }return groups[l];
  }
  function captureBranch(horizon,{preserveView=false,time=Q.state.t*60}={}){if(!preserveView){Q.state.playing=false;Q.focus='overview';Q.state.layer='all';resetFilters();}const source=snapshot(time);return BranchSimulation.capture({time:source.t,region:Q.state.region,scenario:Q.state.scenario,streams:Q.model.streams,objects,vehicles:source.vehicles,states:Object.fromEntries(source.states),config:C},horizon);}
  function captureRuntime(){const source=snapshot(0);return BranchSimulation.captureRuntime({time:0,region:Q.state.region,scenario:Q.state.scenario,streams:Q.model.streams,objects,vehicles:source.vehicles,states:Object.fromEntries(source.states),config:C});}
  return {drawComparison:chart,render,frame,resize,inspect,presentation,context:()=>({view:Q.state.view,currentTime:Q.state.t*60,region:regionId(),scenario:Q.state.scenario,config:C}),geometry,objects,centerSample,captureBranch,captureRuntime,refreshProjection:()=>refresh(true),selectDatabaseObject,streams:()=>Q.model.streams,project:p=>{const el=$('rdScene'),r=el.getBoundingClientRect(),q=project(p,el.clientWidth,el.clientHeight);return {x:q.x*r.width/el.clientWidth,y:q.y*r.height/el.clientHeight};}};
})();
