/* Deterministic, local-only fixtures for the center design review. */
(function(root){
  'use strict';
  const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
  const round=(v,n=1)=>Number(v.toFixed(n));
  const seed=n=>{const x=Math.sin(n*127.1+311.7)*43758.5453;return x-Math.floor(x);};
  const base=Date.parse('2026-10-20T02:00:00Z');
  const iso=t=>new Date(base+t*60000).toISOString();
  const layers=Array.from({length:6},(_,i)=>({id:'L'+(i+1),index:i,angle:i*30,height:135+i*130,capacity:780+i*45,criticalDensity:22+i,length:18+i*1.5}));
  const vertices=Array.from({length:7},(_,y)=>Array.from({length:7},(_,x)=>({x:(x-3)*900+(seed(y*17+x+1)-.5)*330,y:(y-3)*900+(seed(y*13+x+90)-.5)*330})));
  const regions=Array.from({length:36},(_,i)=>{const x=i%6,y=Math.floor(i/6),polygon=[vertices[y][x],vertices[y][x+1],vertices[y+1][x+1],vertices[y+1][x]];return {id:'R'+String(i+1).padStart(2,'0'),index:i,name:['西北','北部','东北','西部','中部','东部'][x]+'空域 '+String(i+1).padStart(2,'0'),polygon,center:{x:polygon.reduce((s,p)=>s+p.x,0)/4,y:polygon.reduce((s,p)=>s+p.y,0)/4},neighbors:[],area:round(0.6+seed(i+5)*.4,2)};});
  const connections=[];
  regions.forEach((r,i)=>{for(const j of [i%6<5?i+1:null,i<30?i+6:null])if(j!==null){r.neighbors.push(regions[j].id);regions[j].neighbors.push(r.id);connections.push({id:r.id+'-'+regions[j].id,from:r.id,to:regions[j].id,path:[r.center,regions[j].center]});}});
  const zones=[{id:'NFZ-01',label:'模拟禁飞区 · 医疗保护区',polygon:[{x:-950,y:150},{x:-450,y:260},{x:-350,y:700},{x:-900,y:680}],bottom:0,top:950},{id:'NFZ-02',label:'模拟禁飞区 · 设施保护区',polygon:[{x:1300,y:-1700},{x:1800,y:-1600},{x:1700,y:-1100},{x:1250,y:-1200}],bottom:0,top:1050}];
  const pads=[{id:'PAD-01',region:'R08',x:-1700,y:-1500,type:'起飞'},{id:'PAD-02',region:'R15',x:-550,y:-600,type:'起降'},{id:'PAD-03',region:'R28',x:700,y:1300,type:'降落'},{id:'PAD-04',region:'R23',x:1600,y:450,type:'起降'}];
  function topology(t=30,variant='normal'){
    const changed=variant==='version'||t>=40;
    return {id:'MOCK-CENTER-01',coordinateSystem:'LOCAL_ENU',origin:{longitude:104.06,latitude:30.67,altitude:0},horizontalUnit:'m',verticalUnit:'m',heightReference:'模拟地面相对高度',regions,layers,connections:connections.map(e=>({...e,open:!(changed&&e.id==='R15-R16')})),zones,pads,versions:Object.fromEntries(regions.map(r=>[r.id,changed&&['R15','R16'].includes(r.id)?'C2':'C1'])),changed};
  }
  function cell(r,l,t=30,scenario='peak',quality='normal'){
    const ri=typeof r==='number'?r:regions.findIndex(v=>v.id===r),li=typeof l==='number'?l:layers.findIndex(v=>v.id===l),region=regions[ri],layer=layers[li];
    const absent=quality==='missing'&&ri===8||quality==='missingLayer'&&ri===11&&li===3,stale=quality==='stale'&&ri===26;
    const sampled=stale?Math.max(0,t-4):t;
    const peak=Math.exp(-Math.pow((sampled-27)/15,2)),hot=[8,14,15,20,21].includes(ri)?1.4:.76;
    const factor=scenario==='normal'?.63:scenario==='reserve'?.9:1;
    const capacity=Math.round(layer.capacity*(.8+seed(ri+31)*.4)*(scenario==='reserve'?1.24:1));
    const u=clamp((.23+.18*seed(ri*6+li)+.42*peak*hot+.09*Math.sin(sampled/8+li+ri/6))*factor,.04,1.12);
    const n=quality==='empty'&&ri===3?0:Math.round(capacity*u),length=layer.length*(.8+seed(ri+77)*.5),density=n/length,speed=clamp(22*(1-.72*Math.pow(Math.min(u,1),2)),3,24),flow=n*speed*60/(length*1000);
    return {regionId:region.id,layerId:layer.id,n:absent?null:n,capacity:absent?null:capacity,u:absent?null:n/capacity,speed:absent||n===0?null:round(speed),density:absent?null:round(density),flow:absent?null:round(flow),length,sampleTime:iso(sampled),age:stale?240:12,missing:absent,stale,reason:absent?(quality==='missingLayer'?'模拟方向层采样缺失':'模拟区域采样缺失'):stale?'模拟超过2个采样周期':null,source:'本地中心宏观样例'};
  }
  function aggregate(cells){
    const valid=cells.filter(v=>!v.missing&&v.n!==null),n=valid.reduce((s,v)=>s+v.n,0),capacity=valid.reduce((s,v)=>s+v.capacity,0),length=valid.reduce((s,v)=>s+v.length,0);
    return {n:valid.length?n:null,capacity:capacity||null,u:capacity?n/capacity:null,speed:n?round(valid.reduce((s,v)=>s+v.n*v.speed,0)/n):null,density:valid.length?round(n/length):null,flow:valid.length?round(valid.reduce((s,v)=>s+v.flow,0)):null,length,missing:cells.length-valid.length,valid:valid.length,total:cells.length,stale:valid.some(v=>v.stale)};
  }
  function state(t,scenario='peak',quality='normal',selectedLayer='all'){
    const cells=regions.flatMap(r=>layers.map(l=>cell(r.index,l.index,t,scenario,quality)));
    const selected=cells.filter(v=>selectedLayer==='all'||v.layerId===selectedLayer);
    return {sampleTime:iso(t),cells,summary:aggregate(selected),regions:regions.map(r=>({...r,...aggregate(selected.filter(v=>v.regionId===r.id))})),layers:layers.map(l=>({...l,...aggregate(cells.filter(v=>v.layerId===l.id))})),quality,source:'本地确定性宏观样例 · 非真实采集'};
  }
  function transfers(regionId,t,scenario='peak',quality='normal'){
    const ri=regions.findIndex(r=>r.id===regionId),missing=quality==='missing'&&ri===8;
    const observed=[],forecast=[],guide=[];
    for(let i=0;i<6;i++){
      const weights=Array.from({length:6},(_,j)=>i===j?0:(Math.abs(i-j)===1?4:1)*(1+.5*seed(ri+i*7+j)));
      const sum=weights.reduce((a,b)=>a+b,0);
      observed[i]=weights.map((w,j)=>i===j?0:missing?null:Math.max(0,Math.round(w*(3+Math.sin(t/6+i+j))*((i+j+ri)%7===0?0:1)*Math.min(5,t)/5)));
      forecast[i]=observed[i].map((v,j)=>v===null?null:i===j?0:Math.max(0,Math.round(v*(1+.12*Math.sin(i+j+1)))));
      guide[i]=weights.map(w=>missing?null:w/sum);
    }
    return {regionId,from:iso(Math.max(0,t-5)),to:iso(t),windowMinutes:Math.min(5,t),observed,forecast,guide,source:'模拟完成转移汇总 · 5分钟窗口',missing,scenario};
  }
  function prediction(regionId,t,horizon=15,scenario='peak',quality='normal',metric='n',layerId='all'){
    const ri=regions.findIndex(r=>r.id===regionId),selected=layers.filter(l=>layerId==='all'||l.id===layerId);
    const value=at=>aggregate(selected.map(l=>cell(ri,l.index,at,scenario,quality)))[metric];
    const history=Array.from({length:31},(_,i)=>{const at=t-30+i;return {t:at,value:at<0?null:value(at)};});
    const future=Array.from({length:horizon+1},(_,i)=>{const actual=value(t+i),valueForecast=actual===null?null:Math.max(0,actual*(1+.035*Math.sin(i*.6+ri)*Math.min(1,i/4)));return {t:t+i,value:valueForecast,low:valueForecast===null?null:valueForecast*.93,high:valueForecast===null?null:valueForecast*1.07};});
    const evaluation=Array.from({length:16},(_,i)=>{const at=t-15+i,actual=at<15||at>t?null:value(at);return {t:at,actual,predicted:actual===null?null:actual*(1+.035*Math.sin(i*.6+ri)),issueTime:at-15};});
    const valid=evaluation.filter(v=>v.actual!==null),mae=valid.length?valid.reduce((s,v)=>s+Math.abs(v.actual-v.predicted),0)/valid.length:null,rmse=valid.length?Math.sqrt(valid.reduce((s,v)=>s+Math.pow(v.actual-v.predicted,2),0)/valid.length):null;
    return {history,future,evaluation,mae,rmse,metric,regionId,layerId,issueTime:iso(t),horizon,algorithm:'MOCK-MACRO-v1',source:'本地关联样例，非独立准确性验证',band:'±7%展示带，非统计置信区间'};
  }
  function experiments(t,scenario){
    const group='MOCK-DEMAND-'+scenario+'-'+t,common={group,seed:20261025,demand:12000,duration:30,snapshot:iso(t)};
    return [{...common,id:'baseline',name:'维持现状',throughput:9100,congestion:.24,delay:84,conflicts:null,compute:148},{...common,id:'balance',name:'域间均衡导引',throughput:9860,congestion:.16,delay:65,conflicts:null,compute:176},{...common,id:'reserve',name:'均衡导引＋备用容量',throughput:10340,congestion:.11,delay:53,conflicts:null,compute:203}];
  }
  root.CenterModel={layers,regions,connections,zones,pads,topology,cell,aggregate,state,transfers,prediction,experiments,iso,clamp,round,seed};
  if(typeof module!=='undefined')module.exports=root.CenterModel;
})(typeof window!=='undefined'?window:globalThis);
