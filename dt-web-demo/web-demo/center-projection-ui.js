/* Center-only projection controls. Regional state and original snapshots stay independent. */
window.CenterProjectionUI=(()=>{
  'use strict';
  const $=id=>document.getElementById(id),model=()=>window.CenterProjection;
  const metrics={meanWait:{label:'平均等待',unit:'s/架'},queue:{label:'当前排队数量',unit:'架'},highLoadSeconds:{label:'高负载累计时长',unit:'宏观单元·秒'},diverted:{label:'累计分流',unit:'架'},n:{label:'数量',unit:'架'},speed:{label:'平均速度',unit:'m/s'}};
  const colors=['#7556A0','#316E91','#AC7B2C','#448A84','#AD6D56','#78568B','#286452','#91803D'];
  let hooks,result=null,mode='live',index=0,playing=false,lastTick=0,busy=false,worker=null,job=0,sourceKey='',message='',records=[],group='',number=0;
  let horizon=600,policy='balance',plotStamp='',plotSeries=[];
  const active=()=>mode!=='live'&&!!result,current=()=>active()?result[mode].frames[index]:null;
  const signature=()=>{const s=hooks.state();return [s.t,s.scenario,hooks.quality()].join('|');};
  const fmt=(value,digits=0)=>value==null?'暂无数据':value.toLocaleString('zh-CN',{maximumFractionDigits:digits});
  function toolsHTML(){return '<strong>全域推演</strong><select id="cdProjectionHorizon" aria-label="中心推演时长"><option value="300">未来5分钟</option><option value="600" selected>未来10分钟</option><option value="900">未来15分钟</option></select><select id="cdPolicy" aria-label="中心推演策略"><option value="baseline">维持现状</option><option value="balance" selected>域间均衡导引</option><option value="reserve">均衡＋备用容量</option></select><button id="cdProjectionRun">从当前全域推演</button><button id="cdProjectionBase" disabled>基线</button><button id="cdProjectionPolicy" disabled>策略</button><button id="cdProjectionExit" hidden>返回当前态势</button><span id="cdProjectionSource" role="status"></span>';}
  const chartId=(prefix,key)=>prefix+(key==='meanWait'?'':key);
  const chartsHTML=(prefix,rmse=false)=>'<div class="cd-comparison-charts">'+Object.entries(metrics).map(([key,item])=>'<figure><figcaption>'+item.label+(rmse?' RMSE':'')+' · '+item.unit+'</figcaption><div class="cd-mini-chart"><canvas id="'+chartId(prefix,key)+'" role="img" aria-label="'+item.label+(rmse?' RMSE':'')+'随时间变化"></canvas></div></figure>').join('')+'</div>';
  function cardHTML(){return '<div class="cd-toolbar"><label>比较组 <select id="cdProjectionGroup" aria-label="中心推演比较组"></select></label><button id="cdProjectionClear">清空记录</button><span class="cd-note">最近8次</span></div><div class="cd-run-legend" id="cdPolicyLegend"></div>'+chartsHTML('cdPolicyPlot')+'<div class="cd-policy-summary" id="cdPolicySummary" role="status" aria-live="polite"></div><details class="cd-method"><summary>查看指标口径</summary><div class="cd-metadata" id="cdPolicyMeta"></div></details>';}
  function mount(config){
    hooks=config;$('cdProjectionTools').innerHTML=toolsHTML();
    $('cdProjectionRun').onclick=run;$('cdProjectionExit').onclick=()=>exit('');
    $('cdProjectionBase').onclick=()=>choose('baseline');$('cdProjectionPolicy').onclick=()=>choose('strategy');
    $('cdProjectionHorizon').onchange=e=>{horizon=Number(e.target.value);exit('时域已改变，请重新推演');};
    $('cdPolicy').onchange=e=>{policy=e.target.value;exit('策略已改变，请从同一快照再次运行以比较');};
    $('cdProjectionGroup').onchange=e=>{group=e.target.value;hooks.refresh();};
    $('cdProjectionClear').onclick=()=>{records=[];group='';hooks.refresh();};
    document.addEventListener('visibilitychange',()=>{if(document.hidden){playing=false;lastTick=0;if(hooks.state().view==='center')hooks.refresh();}});
  }
  function stop(){playing=false;lastTick=0;}
  function discardWorker(){if(worker){worker.terminate();worker=null;}}
  function exit(note=message){stop();mode='live';if(busy){job++;discardWorker();}busy=false;message=note;hooks.refresh();}
  function sync(){if((active()||busy)&&sourceKey!==signature()){stop();mode='live';busy=false;job++;discardWorker();message='原始时刻或场景已改变，请从新快照推演';}}
  function choose(value){if(!result||busy)return;mode=value;stop();hooks.refresh();}
  function save(computed,started,fallback){
    result=computed;index=0;mode='strategy';busy=false;stop();
    const thin=frames=>frames.map(f=>({time:f.time,elapsed:f.elapsed,metrics:{...f.metrics}}));
    const record={number:++number,group:result.input.id,policy:result.policy.id,label:result.policy.label,color:colors[(number-1)%colors.length],time:result.input.time,horizon:result.input.horizon,scenario:result.input.scenario,frames:thin(result.strategy.frames),baseline:thin(result.baseline.frames)};
    records.push(record);records=records.slice(-8);group=record.group;
    message=fallback?'后台不可用，已在本地完成计算':'';
    hooks.refresh();
  }
  function run(){
    if(busy)return;stop();hooks.state().playing=false;const id=++job,started=performance.now();sourceKey=signature();busy=true;mode='live';message='正在计算同快照、同需求的基线与策略…';hooks.refresh();
    let input;try{input=model().capture(hooks.state().t,hooks.state().scenario,horizon,hooks.quality());}catch(error){busy=false;message='推演失败：'+error.message;hooks.refresh();return;}
    const accept=computed=>{if(job!==id)return;if(signature()!==sourceKey||hooks.state().view!=='center'){busy=false;stop();mode='live';message='视图或源数据已改变，请重新推演';return;}save(computed,started,false);};
    const fail=error=>{if(job!==id)return;discardWorker();busy=false;mode='live';message='推演失败：'+(error.message||error);hooks.refresh();};
    const fallback=()=>{discardWorker();requestAnimationFrame(()=>{setTimeout(()=>{if(job!==id)return;try{const computed=model().compare(input,policy);if(signature()===sourceKey&&hooks.state().view==='center')save(computed,started,true);else{busy=false;mode='live';stop();}}catch(error){fail(error);}},0);});};
    try{if(typeof Worker==='undefined'){fallback();return;}worker=new Worker('center-projection-worker.js');worker.onmessage=e=>{if(job!==id||e.data.id!==id)return;discardWorker();if(e.data.error){fail(e.data.error);return;}accept(e.data.result);};worker.onerror=e=>{e.preventDefault();if(job===id)fallback();};worker.postMessage({id,input,policy});}catch(error){fallback();}
  }
  function seek(seconds){if(!active())return;stop();index=Math.max(0,Math.min(result[mode].frames.length-1,Math.round(seconds/result.input.step)));hooks.refresh();}
  function toggle(){if(!active()||busy)return;if(index===result[mode].frames.length-1)index=0;playing=!playing;lastTick=performance.now();hooks.refresh();}
  function tick(now,rate){
    if(!active()||!playing||document.hidden){lastTick=now;return false;}
    const interval=result.input.step*1000/rate;if(!lastTick)lastTick=now;
    if(now-lastTick<interval)return false;
    const steps=Math.floor((now-lastTick)/interval);lastTick+=steps*interval;index=Math.min(result[mode].frames.length-1,index+steps);if(index===result[mode].frames.length-1)stop();hooks.refresh();return true;
  }
  function render(){
    const on=active();$('cdProjectionRun').disabled=busy;$('cdProjectionRun').textContent=busy?'计算中…':'从当前全域推演';$('cdProjectionExit').hidden=!on&&!busy;$('cdProjectionExit').textContent=busy?'取消计算':'返回当前态势';
    for(const [id,value]of [['cdProjectionBase','baseline'],['cdProjectionPolicy','strategy']]){const el=$(id);el.disabled=!on;el.classList.toggle('active',mode===value);el.setAttribute('aria-pressed',String(mode===value));}
    $('cdProjectionSource').textContent=message;$('cdProjectionSource').title='宏观库存流模拟，固定起报拓扑；只分配新需求，不模拟真实个体路线与避碰。';
    $('cdProjectionHorizon').value=horizon;$('cdPolicy').value=policy;
    const groups=[...new Map(records.map(r=>[r.group,r])).values()];if(!groups.some(r=>r.group===group))group=groups.at(-1)?.group||'';
    $('cdProjectionGroup').innerHTML=groups.length?groups.map(r=>'<option value="'+r.group+'">'+hooks.time(r.time)+' / '+r.horizon/60+'分钟'+'</option>').join(''):'<option value="">尚无记录</option>';$('cdProjectionGroup').value=group;$('cdProjectionGroup').disabled=!groups.length;$('cdProjectionClear').disabled=!records.length;
    const list=records.filter(r=>r.group===group),last=list.at(-1),a=on&&group===result.input.id?current():null;
    $('cdPolicyLegend').innerHTML=last?'<span><i style="border-color:#647169;border-top-style:dashed"></i>基线 · 维持现状</span>'+list.filter(r=>r.policy!=='baseline').map(r=>'<span><i style="border-color:'+r.color+'"></i>#'+r.number+' '+r.label+'</span>').join(''):'';
    const hasRecords=records.length>0;
    document.querySelector('#centerDemo .cd-policy-card').classList.toggle('has-records',hasRecords);
    if(!hasRecords)for(const key of Object.keys(metrics))$(chartId('cdPolicyPlot',key)).setAttribute('aria-label','尚无推演记录');
    $('cdPolicySummary').textContent=a?'当前'+(mode==='baseline'?'基线':result.policy.label)+' · 等待 '+fmt(a.metrics.meanWait,1)+'s/架 · 排队 '+fmt(a.metrics.queue)+'架 · 完成 '+fmt(a.metrics.completed)+'架':last?'最近 '+records.length+'/8 次 · 本组 '+list.length+' 次 · #'+last.number+' '+last.label:'选择策略后启动推演';
    $('cdPolicyMeta').textContent=last?'全域 · 起报 '+hooks.time(last.time)+' · '+last.horizon/60+'分钟 · 高负载按区域×层宏观单元累计':'六项指标与区域DT对应；高负载粒度为区域×层，全部为本地模拟。';
    $('cdPolicyMeta').title='平均等待=起报后累计排队秒/初始在途及累计新需求；数量与均速不含入口排队；高负载≥85%，宏观单元·秒不等于细管道·秒；累计指标从起报归零。';
  }
  function draw(plot){
    const list=records.filter(r=>r.group===group),first=list[0],stamp=group+'|'+list.map(r=>r.number).join(',');
    if(stamp!==plotStamp){plotStamp=stamp;plotSeries=Object.keys(metrics).map(key=>{const points=frames=>frames.map(f=>({x:f.time,y:f.metrics[key]}));return first?[{name:'基线',color:'#647169',dash:[5,4],points:points(first.baseline)},...list.filter(r=>r.policy!=='baseline').map(r=>({name:'#'+r.number,color:r.color,points:points(r.frames)}))]:[];});}
    Object.entries(metrics).forEach(([key,item],i)=>{
      const id=chartId('cdPolicyPlot',key);plot(id,plotSeries[i],{unit:item.unit,xMin:first?.time??0,xMax:first?first.time+first.horizon/60:10,vertical:first&&active()&&result.input.id===group?current().time:null,verticalLabel:'回放',axes:true,xLabel:'时间',legend:false,empty:'尚无推演记录'});
      $(id).title='基线：维持现状\n'+list.map(r=>'#'+r.number+' '+r.label).join('\n');
    });
  }
  return {mount,cardHTML,chartsHTML,chartId,metrics,sync,render,draw,active,busy:()=>busy,current,input:()=>result?.input,frames:()=>active()?result[mode].frames:[],snapshot:layer=>active()?model().snapshot(result.input,current(),layer):null,playing:()=>playing,stop,seek,toggle,tick,inspect:()=>({result,mode,index,elapsed:current()?.elapsed??0,playing,busy,horizon,policy,group,records})};
})();
