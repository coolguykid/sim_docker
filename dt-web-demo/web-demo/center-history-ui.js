/* Historical observations are an independent baseline simulation until real data is connected. */
window.CenterHistoryUI=(()=>{
  'use strict';
  const $=id=>document.getElementById(id),PU=window.CenterProjectionUI,cache=new Map();
  let hooks,runtime=null,policy='balance',windowSeconds=600,history=null,stamp='',series=[],queued=false;
  function cardHTML(){return '<div class="cd-toolbar"><label>查看策略 <select id="cdHistoryPolicy" aria-label="中心历史DT策略"><option value="balance">域间均衡导引</option><option value="reserve">均衡＋备用容量</option><option value="baseline">维持现状</option></select></label><label>窗口 <select id="cdHistoryWindow" aria-label="中心历史对比窗口"><option value="300">近5分钟</option><option value="600" selected>近10分钟</option><option value="900">近15分钟</option></select></label></div>'+PU.chartsHTML('cdHistoryPlot',true)+'<div class="cd-policy-summary cd-history-progress" id="cdHistoryMessage" role="status" aria-live="polite"></div>';}
  function mount(config){hooks=config;for(const [id,set]of [['cdHistoryPolicy',v=>policy=v],['cdHistoryWindow',v=>windowSeconds=+v]])$(id).onchange=e=>{set(e.target.value);refresh();};}
  function repaint(entry){if(entry!==runtime||queued||hooks.state().view!=='center')return;queued=true;requestAnimationFrame(()=>{queued=false;if(hooks.state().view==='center')refresh();});}
  function create(key){
    const entry={key,to:-5,target:0,runs:[],worker:null,timer:null,mode:'worker',disposed:false,error:null},scenario=hooks.state().scenario,quality=hooks.quality();
    const receive=batch=>{if(entry.disposed)return;if(batch.error){fallback(batch.error);return;}if(!entry.runs.length)entry.runs=batch.runs;else batch.runs.forEach((r,i)=>entry.runs[i].timeline.push(...r.timeline));entry.to=batch.to;repaint(entry);};
    function fallback(reason){if(entry.disposed||entry.mode==='cooperative')return;entry.worker?.terminate();entry.worker=null;entry.mode='cooperative';entry.error=reason;entry.to=-5;entry.runs=[];
      const engine=CenterProjection.createHistory(scenario,quality);
      entry.pump=()=>{entry.timer=null;if(entry.disposed||entry.to>=entry.target)return;try{receive(engine.advance(Math.min(entry.target,entry.to+5)));}catch(error){entry.error=error.message;return;}if(entry.to<entry.target)entry.timer=setTimeout(entry.pump,0);};
      entry.timer=setTimeout(entry.pump,0);
    }
    entry.request=to=>{entry.target=to;if(entry.worker)entry.worker.postMessage({type:'target',to});else if(entry.pump&&entry.timer===null&&entry.to<to)entry.timer=setTimeout(entry.pump,0);};
    try{entry.worker=new Worker('center-history-worker.js');entry.worker.onmessage=e=>{if(entry.mode==='worker')receive(e.data);};entry.worker.onerror=e=>{e.preventDefault();fallback(e.message||'历史后台计算失败');};entry.worker.postMessage({type:'init',scenario,quality,to:0});}catch(error){fallback(error.message);}
    return entry;
  }
  function refresh(){
    if(!hooks||hooks.state().view!=='center')return;
    const key=hooks.state().scenario+'|'+hooks.quality(),to=Math.floor(Math.max(0,Math.min(60,hooks.time()))*60/5)*5,from=Math.max(0,to-windowSeconds);
    if(runtime?.key!==key){runtime=cache.get(key)||create(key);cache.delete(key);cache.set(key,runtime);while(cache.size>2){const oldest=cache.keys().next().value,entry=cache.get(oldest);entry.disposed=true;entry.worker?.terminate();clearTimeout(entry.timer);cache.delete(oldest);}}
    if(runtime.target!==to||runtime.to<0)runtime.request(to);
    const next=[key,from,to,runtime.to,policy].join('|');
    if(next!==stamp){stamp=next;const select=points=>points.filter(p=>p.t>=from&&p.t<=to),observed=select(runtime.runs[0]?.timeline||[]),dt=select(runtime.runs.find((r,i)=>i>0&&r.id===policy)?.timeline||[]);
      history={from,to,policy,scenario:hooks.state().scenario,ready:runtime.to>=to,timeline:observed,dt};
      series=Object.keys(PU.metrics).map(k=>[{name:'RMSE',color:'#7556A0',points:RegionEvaluation.historyRmse(observed,dt,k).map(p=>({x:p.t/60,y:p.y})),dots:observed.length===1}]);
    }
    const message=$('cdHistoryMessage');message.replaceChildren();
    if(history.ready){message.textContent=hooks.clock(from/60)+'–'+hooks.clock(to/60)+' · RMSE'+(to===0?' · 仅有初始样本':'');message.classList.remove('is-loading');}
    else{
      const done=Math.max(0,Math.min(to,runtime.to)),maximum=Math.max(1,to),percent=to?Math.floor(done/maximum*100):0,status=document.createElement('span'),progress=document.createElement('progress'),label=document.createElement('b');
      status.textContent='补算到 '+hooks.clock(to/60)+' · '+(runtime.mode==='worker'?'后台线程':'本地分批计算')+'已至 '+hooks.clock(done/60);
      progress.max=maximum;progress.value=done;progress.setAttribute('aria-label','历史态势计算进度');label.textContent=percent+'%';message.append(status,progress,label);message.classList.add('is-loading');
    }
    Object.entries(PU.metrics).forEach(([k,item],i)=>hooks.plot(PU.chartId('cdHistoryPlot',k),series[i],{unit:'RMSE · '+item.unit,xMin:from/60,xMax:to/60,axes:true,xLabel:'时间',legend:false}));
  }
  return {cardHTML,mount,refresh,inspect:()=>({history,runtime:runtime?{key:runtime.key,to:runtime.to,target:runtime.target,mode:runtime.mode,cacheSize:cache.size,error:runtime.error}:null})};
})();
