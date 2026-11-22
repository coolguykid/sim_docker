/* Regional-only offline presentation. Reference observations are synthetic. */
(function(root){
  'use strict';
  const finite=x=>typeof x==='number'&&Number.isFinite(x),optional=x=>x==null||finite(x)&&x>=0;
  function validate(data,knownIds){
    if(!data||typeof data!=='object'||typeof data.run_id!=='string'||!data.run_id.trim()||!/^R\d+$/.test(data.region_id)||typeof data.topology_version!=='string'||typeof data.scenario!=='string')throw Error('缺少运行、区域、版本或场景信息');
    if(!['baseline','all','layer'].includes(data.policy_id)||data.policy_id==='layer'&&(!Array.isArray(data.policy_layers)||data.policy_layers.length!==1||!Number.isInteger(data.policy_layers[0])||data.policy_layers[0]<0||data.policy_layers[0]>5))throw Error('策略标识或作用层不合法');
    if(data.policy_id!=='layer'&&data.policy_layers!=null)throw Error('非分层策略的policy_layers应为null或省略');
    if(!finite(data.snapshot_time)||data.snapshot_time<0||!Number.isInteger(data.horizon)||data.horizon<60||data.horizon>900||!Array.isArray(data.samples)||!data.samples.length||data.samples.length>50000)throw Error('起报时间、推演时域或观测样本无效（最多50000点）');
    const seen=new Set(),known=new Set(knownIds);
    for(const s of data.samples){
      if(!s||!finite(s.sim_time)||s.sim_time<=data.snapshot_time||s.sim_time>data.snapshot_time+data.horizon||!known.has(s.object_id)||!Number.isSafeInteger(s.n)||s.n<0||!optional(s.speed)||!optional(s.max_speed)||!optional(s.min_gap))throw Error('观测点时间、设施ID或数值无效；缺失用null，数量须为非负整数');
      if(s.n===0&&s.speed!=null||s.n>0&&s.speed==null)throw Error('零数量时均速应为null，非零数量须提供均速');
      if(s.max_speed!=null&&(s.speed==null||s.max_speed<s.speed)||s.min_gap!=null&&s.n<2)throw Error('最高速度不能低于均速；少于2架不能提供最小间距');
      const key=s.sim_time+'|'+s.object_id;if(seen.has(key))throw Error('同一时刻同一设施的观测点重复');seen.add(key);
    }
    return data;
  }
  function matches(data,result){const i=result.input,p=result.policy;return data.region_id==='R'+String(i.region+1).padStart(2,'0')&&data.topology_version===i.version&&data.scenario===i.scenario&&data.snapshot_time===i.time&&data.horizon===i.horizon&&data.policy_id===p.id&&JSON.stringify(data.policy_layers??null)===JSON.stringify(p.layers??null);}
  function evaluate(data,result){
    if(!matches(data,result))throw Error('观测与本次DT推演的区域、版本、场景、策略、起报或时域不一致');
    const frames=new Map(result.strategy.frames.map(f=>[result.input.time+f.elapsed,f]));
    let absN=0,sqN=0,absSpeed=0,sqSpeed=0,count=0,speedCount=0;const pairs=[],byTime=new Map();
    for(const s of data.samples){const predicted=frames.get(s.sim_time)?.states[s.object_id];if(!predicted||predicted.missing||!finite(predicted.n))continue;
      const point=byTime.get(s.sim_time)??{absN:0,sqN:0,absSpeed:0,sqSpeed:0,count:0,speedCount:0};byTime.set(s.sim_time,point);
      const dn=predicted.n-s.n;absN+=Math.abs(dn);sqN+=dn*dn;count++;
      point.absN+=Math.abs(dn);point.sqN+=dn*dn;point.count++;
      if(finite(predicted.speed)&&finite(s.speed)){const ds=predicted.speed-s.speed;absSpeed+=Math.abs(ds);sqSpeed+=ds*ds;speedCount++;point.absSpeed+=Math.abs(ds);point.sqSpeed+=ds*ds;point.speedCount++;}
      pairs.push({t:s.sim_time,id:s.object_id,predicted:predicted.n,observed:s.n,predictedSpeed:predicted.speed,observedSpeed:s.speed});
    }
    const timeline=[...frames.keys()].filter(t=>t>result.input.time).sort((a,b)=>a-b).map(t=>{const p=byTime.get(t);return {t,count:p?.count??0,speedCount:p?.speedCount??0,mae:p?.count?p.absN/p.count:null,rmse:p?.count?Math.sqrt(p.sqN/p.count):null,speedMae:p?.speedCount?p.absSpeed/p.speedCount:null,speedRmse:p?.speedCount?Math.sqrt(p.sqSpeed/p.speedCount):null};});
    return {count,speedCount,total:data.samples.length,mae:count?absN/count:null,rmse:count?Math.sqrt(sqN/count):null,speedMae:speedCount?absSpeed/speedCount:null,speedRmse:speedCount?Math.sqrt(sqSpeed/speedCount):null,pairs,timeline};
  }
  // Stable phase offsets make the same snapshot reproduce the same demonstration.
  // This perturbs the DT output; it is not an independent validation dataset.
  function offlineObservations(result){
    const i=result.input,samples=[];
    for(const f of result.strategy.frames){if(f.elapsed<=0)continue;
      for(const [id,s] of Object.entries(f.states)){if(s.missing||!finite(s.n))continue;
        const phase=[...id].reduce((n,c)=>n+c.charCodeAt(0),0)*.37+i.region*.23+i.time*.001;
        const wave=Math.sin(f.elapsed/39+phase)*.7+Math.sin(f.elapsed/87+phase*.6)*.3;
        const n=s.n===0?0:Math.max(0,s.n+Math.round(wave*Math.max(2,s.n*.065)));
        const speed=n===0?null:Math.max(0,Math.min(i.speedLimit??18,(s.speed??0)+.42*Math.sin(f.elapsed/53+phase)));
        samples.push({sim_time:i.time+f.elapsed,object_id:id,n,speed});
      }
    }
    return {run_id:'OFFLINE-DEMO-'+(i.region+1)+'-'+i.time,source:'offline-synthetic',region_id:'R'+String(i.region+1).padStart(2,'0'),topology_version:i.version,scenario:i.scenario,policy_id:result.policy.id,policy_layers:result.policy.layers??null,snapshot_time:i.time,horizon:i.horizon,samples};
  }
  // Both panels consume the same regional aggregates; queued aircraft are counted separately.
  const metrics=[
    {key:'meanWait',suffix:'Wait',label:'平均等待',unit:'s/架'},
    {key:'queue',suffix:'Queue',label:'当前排队数量',unit:'架'},
    {key:'highLoadSeconds',suffix:'HighLoad',label:'高负载管道累计时长',unit:'管道·秒'},
    {key:'diverted',suffix:'Diverted',label:'累计分流',unit:'架'},
    {key:'n',suffix:'Count',label:'数量',unit:'架'},
    {key:'speed',suffix:'Speed',label:'平均速度',unit:'m/s'}
  ];
  function timeline(input,frames){return frames.map(f=>{
    const valid=Object.values(f.states).filter(s=>!s.missing&&finite(s.n)),n=valid.reduce((a,s)=>a+s.n,0);
    const speedCount=valid.reduce((a,s)=>a+(finite(s.speed)?s.n:0),0),weighted=valid.reduce((a,s)=>a+(finite(s.speed)?s.speed*s.n:0),0);
    return {t:input.time+f.elapsed,meanWait:f.meanWait,queue:f.queue,highLoadSeconds:f.highLoadSeconds,diverted:f.diverted,n:valid.length?n:null,speed:speedCount?weighted/speedCount:null};
  });}
  function historyRmse(observed,predicted,key){
    const byTime=new Map(predicted.map(p=>[p.t,p]));let sum=0,count=0;
    return [...observed].sort((a,b)=>a.t-b.t).map(p=>{
      const q=byTime.get(p.t);if(!finite(p[key])||!finite(q?.[key]))return {t:p.t,y:null};
      sum+=(q[key]-p[key])**2;return {t:p.t,y:Math.sqrt(sum/++count)};
    });
  }
  let latest=null,history=null,runtime=null,records=[],next=1,selectedGroup=0;
  const runtimeCache=new Map();let runtimeId=0,paintQueued=false,historyStamp='',historySeries=null,effectStamp='',effectSeries=null;
  let historyPolicy='all',historyLayer=0,historyWindow=600;
  const policies=[{id:'baseline',label:'维持现状',layers:null},{id:'all',label:'开放全部备用',layers:null},...Array.from({length:6},(_,l)=>({id:'layer',label:'开放L'+(l+1)+'备用',layers:[l]}))];
  const $=id=>document.getElementById(id),fmt=x=>x==null?'暂无数据':x.toFixed(2);
  const colors=['#387D69','#7556A0','#B87232','#4286A0','#AB596D','#79863E','#5869AC','#87725C'];
  const color=r=>colors[(r.number-1)%colors.length];
  const region=i=>'R'+String(i.region+1).padStart(2,'0');
  const clock=t=>[10+Math.floor(t/3600),Math.floor(t/60)%60,Math.floor(t)%60].map(v=>String(v).padStart(2,'0')).join(':');
  const charts=prefix=>metrics.map(p=>'<figure class="rd-error-chart"><figcaption>'+p.label+(prefix==='rdHistory'?' RMSE':'')+' · '+p.unit+'</figcaption><div class="rd-eval-plot"><canvas id="'+prefix+p.suffix+'" role="img" aria-label="'+p.label+(prefix==='rdHistory'?' RMSE':'')+'随时间变化"></canvas></div></figure>').join('');
  function mount(){
    $('rdEvaluations').innerHTML=
      '<section class="panel rd-history-panel"><div class="rd-head"><h3>区域DT · 历史观测与当前推演对比</h3><span class="badge">历史观测：模拟替代</span></div><div class="rd-eval-body">'+
      '<div class="rd-eval-actions"><label>查看策略 <select id="rdCompareStrategy" aria-label="历史DT推演策略"><option value="all">开放全部备用</option><option value="layer">开放指定层备用</option><option value="baseline">维持现状</option></select></label><label>作用层 <select id="rdCompareLayer" aria-label="历史对比中的策略作用层">'+Array.from({length:6},(_,l)=>'<option value="'+l+'">L'+(l+1)+'</option>').join('')+'</select></label><label>窗口 <select id="rdHistoryWindow" aria-label="历史对比时间窗口"><option value="300">近5分钟</option><option value="600" selected>近10分钟</option><option value="900">近15分钟</option></select></label></div>'+
      '<div id="rdHistoryContext" class="rd-eval-summary"></div><div id="rdHistoryLegend" class="rd-effect-legend"><span><i style="border-color:#7556A0"></i>RMSE：历史运行值（模拟替代）与历史DT推演</span></div>'+
      '<div class="rd-comparison-charts">'+charts('rdHistory')+'</div><div id="rdHistoryMessage" class="rd-eval-message"></div><p class="rd-small rd-metric-note">系统自动运行全部8种策略，截止当前运行时刻。RMSE=√(Σ(DT−运行值)²/N)，自所选窗口起点逐点累计，按5秒采样及当前秒端点配对；缺失不补零。运行值暂由维持现状模型替代。累计指标自仿真起点计数；自动推演不计入右侧记录。</p></div></section>'+
      '<section class="panel rd-effect-panel"><div class="rd-head"><h3>区域推演DT策略依次对比 · 有效性</h3><span class="rd-note">同快照 / 同需求</span></div><div class="rd-eval-body"><div class="rd-eval-actions"><label>比较组 <select id="rdEffectGroup" aria-label="有效性比较组"></select></label><button id="rdEffectClear">清空记录</button><span class="rd-note">最近8次</span></div><div id="rdEffectContext" class="rd-eval-summary"></div><div id="rdEffectLegend" class="rd-effect-legend" role="list" aria-label="策略运行图例"></div><div class="rd-comparison-charts">'+charts('rdEffect')+'</div><div id="rdEffectSummary" class="rd-eval-message"></div><p class="rd-small rd-metric-note">两侧使用相同六项指标：左侧为RMSE，右侧为运行值。数量和均速统计全部有效设施内个体，排队另计。高负载为管道利用率≥85%；同组共用时间范围、独立Y轴。不同组不直接推断策略收益。全部为本地模拟。</p></div></section>';
    const effectPanel=$('rdEvaluations').querySelector('.rd-effect-panel');effectPanel.id='rdEffectPanel';
    const effectCharts=effectPanel.querySelector('.rd-comparison-charts');effectCharts.id='rdEffectCharts';
    const historyMessage=$('rdHistoryMessage');historyMessage.setAttribute('role','status');historyMessage.setAttribute('aria-live','polite');
    for(const note of $('rdEvaluations').querySelectorAll('.rd-metric-note')){
      const disclosure=document.createElement('details'),summary=document.createElement('summary'),copy=document.createElement('p');
      disclosure.className='rd-metric-note';summary.textContent='指标口径';copy.className='rd-small';copy.textContent=note.textContent;disclosure.append(summary,copy);note.replaceWith(disclosure);
    }
    $('rdCompareStrategy').onchange=e=>{historyPolicy=e.target.value;refresh();};
    $('rdCompareLayer').onchange=e=>{historyLayer=+e.target.value;refresh();};
    $('rdHistoryWindow').onchange=e=>{historyWindow=+e.target.value;refresh();};
    $('rdEffectClear').onclick=()=>{records=[];selectedGroup=0;paintEffect();};
    $('rdEffectGroup').onchange=e=>{selectedGroup=+e.target.value;paintEffect();};
    paintEffect();
  }
  function scheduleHistoryPaint(entry){if(entry!==runtime||paintQueued||RegionDemo.context().view!=='region')return;paintQueued=true;requestAnimationFrame(()=>{paintQueued=false;if(runtime&&RegionDemo.context().view==='region')refresh();});}
  function createRuntime(context,key){
    const input=RegionDemo.captureRuntime(),entry={id:++runtimeId,key,to:-1,target:0,runs:[policies[0],...policies].map(policy=>({policy,points:[]})),worker:null,timer:null,mode:'worker',batches:0,computeMs:0,error:null};
    const receive=batch=>{if(entry.disposed)return;if(batch.error){fallback(batch.error);return;}for(let i=0;i<batch.runs.length;i++)entry.runs[i].points.push(...batch.runs[i].timeline);entry.to=batch.to;entry.batches++;entry.computeMs+=batch.computeMs||0;scheduleHistoryPaint(entry);};
    function fallback(reason){if(entry.disposed||entry.mode==='cooperative')return;entry.worker?.terminate();entry.worker=null;entry.mode='cooperative';entry.error=reason;entry.to=-1;for(const run of entry.runs)run.points=[];
      const engine=RegionHistory.create(input,[policies[0],...policies]);receive(engine.advance(0));
      entry.pump=()=>{entry.timer=null;if(entry.disposed||entry.to>=entry.target)return;const start=performance.now(),batch=engine.advance(Math.min(entry.target,entry.to+1));receive({...batch,computeMs:performance.now()-start});entry.timer=setTimeout(entry.pump,0);};
      entry.timer=setTimeout(entry.pump,0);
    }
    entry.request=to=>{if(entry.target===to)return;entry.target=to;if(entry.worker)entry.worker.postMessage({type:'target',to});else if(entry.pump&&entry.timer===null)entry.timer=setTimeout(entry.pump,0);};
    try{entry.worker=new Worker('region-history-worker.js');entry.worker.onmessage=e=>{if(entry.mode==='worker')receive(e.data);};entry.worker.onerror=e=>{e.preventDefault();fallback(e.message||'后台计算不可用');};entry.worker.postMessage({type:'init',input,policies:[policies[0],...policies],to:0});}catch(error){fallback(error.message);}
    return entry;
  }
  function updateHistory(context){
    const end=Math.max(0,Math.min(context.config.duration,Math.floor(context.currentTime))),key=context.region+'|'+context.scenario+'|'+context.config.version;
    if(!runtime||runtime.key!==key){runtime=runtimeCache.get(key)||createRuntime(context,key);runtimeCache.delete(key);runtimeCache.set(key,runtime);while(runtimeCache.size>2){const oldest=runtimeCache.keys().next().value,entry=runtimeCache.get(oldest);entry.disposed=true;entry.worker?.terminate();clearTimeout(entry.timer);runtimeCache.delete(oldest);}}
    runtime.request(end);
    const from=Math.max(0,end-historyWindow),dataTo=Math.min(end,runtime.to),stamp=[runtime.id,end,dataTo,historyWindow,historyPolicy,historyLayer].join('|');
    if(stamp===historyStamp)return;historyStamp=stamp;
    const select=points=>points.slice(from,Math.max(from,dataTo+1)).filter(p=>p.t%5===0||p.t===end);
    const runs=runtime.runs.slice(1).map(r=>({policy:r.policy,timeline:select(r.points)})),selected=runs.find(r=>r.policy.id===historyPolicy&&(historyPolicy!=='layer'||r.policy.layers[0]===historyLayer));
    history={from,to:end,region:context.region,scenario:context.scenario,automatic:true,ready:runtime.to>=end,policy:selected.policy,timeline:select(runtime.runs[0].points),dt:selected.timeline,runs};
    historySeries=metrics.map(p=>[{color:'#7556A0',points:history.timeline.length===1,data:historyRmse(history.timeline,history.dt,p.key)}]);
  }
  function record(result){
    latest=result;
    const signature=JSON.stringify(result.input),existing=records.find(r=>r.signature===signature),group=existing?.group??next;
    const points=timeline(result.input,result.strategy.frames);
    records.push({number:next++,group,signature,policy:result.policy.label,policyId:result.policy.id,input:{region:result.input.region,time:result.input.time,horizon:result.input.horizon,scenario:result.input.scenario},metrics:points.at(-1),baseline:timeline(result.input,result.baseline.frames).at(-1),timeline:points});
    if(records.length>8)records.shift();selectedGroup=group;paintEffect();refresh();
  }
  function paintEffect(){
    const groups=[...new Set(records.map(r=>r.group))];$('rdEffectGroup').replaceChildren(...groups.map(g=>new Option('组 '+g,g)));$('rdEffectGroup').value=selectedGroup;$('rdEffectGroup').disabled=!groups.length;
    const list=records.filter(r=>r.group===selectedGroup),i=list[0]?.input;
    $('rdEffectContext').textContent=i?region(i)+' · '+'起报 '+clock(i.time)+' · 时域 '+i.horizon+'s':'选择策略后启动推演';
    $('rdEffectLegend').innerHTML=list.map(r=>'<span role="listitem"><i style="border-color:'+color(r)+';border-top-style:'+(r.policyId==='baseline'?'dashed':'solid')+'"></i>'+r.number+' · '+r.policy+'</span>').join('');
    const last=list.at(-1);$('rdEffectSummary').textContent=last?'第'+last.number+'次相对同输入基线：平均等待变化 '+fmt(last.metrics.meanWait-last.baseline.meanWait)+' s/架；当前排队变化 '+(last.metrics.queue-last.baseline.queue)+'架。':'';
    const hasRecords=list.length>0;
    $('rdEffectContext').hidden=false;$('rdEffectLegend').hidden=false;$('rdEffectCharts').hidden=false;
    $('rdEffectSummary').hidden=false;$('rdEffectPanel').dataset.empty=String(!hasRecords);
    drawEffect();
  }
  function drawEffect(){
    if(!root.RegionDemo||!$('rdEffectWait'))return;const list=records.filter(r=>r.group===selectedGroup);const i=list[0]?.input,from=i?.time??RegionDemo.context().currentTime;
    const stamp=selectedGroup+'|'+list.map(r=>r.number).join(',');if(stamp!==effectStamp){effectStamp=stamp;effectSeries=metrics.map(p=>list.map(r=>({color:color(r),dash:r.policyId==='baseline'?[5,3]:[],data:r.timeline.map(f=>({t:f.t,y:f[p.key]}))})));}
    for(let k=0;k<metrics.length;k++){const p=metrics[k];RegionDemo.drawComparison('rdEffect'+p.suffix,effectSeries[k],{from,to:from+(i?.horizon??600),unit:p.unit,axes:true,xLabel:'时间',empty:'尚无推演记录'});$('rdEffect'+p.suffix).setAttribute('aria-label',p.label+(list.length?'随时间变化':'：尚无推演记录'));}
  }
  function drawHistory(){
    for(let k=0;k<metrics.length;k++){const p=metrics[k];RegionDemo.drawComparison('rdHistory'+p.suffix,historySeries[k],{from:history.from,to:history.to,unit:'RMSE · '+p.unit,axes:true,xLabel:'时间'});}
  }
  function refresh(){
    if(!$('rdHistoryContext'))return;const context=RegionDemo.context();
    if(latest&&(region(latest.input)!==context.region||latest.input.scenario!==context.scenario))latest=null;
    updateHistory(context);$('rdCompareLayer').disabled=historyPolicy!=='layer';
    $('rdHistoryContext').textContent=clock(history.from)+'–'+clock(history.to)+' · RMSE';
    const historyMessage=$('rdHistoryMessage');
    historyMessage.dataset.state=history.ready?'ready':'pending';
    historyMessage.textContent=history.ready?(history.to===0?'仅有初始样本':''):'正在计算至 '+clock(history.to)+' · 已完成至 '+clock(Math.max(0,runtime.to));
    drawHistory();drawEffect();
  }
  const api={validate,matches,evaluate,offlineObservations,metrics,timeline,historyRmse,mount,record,refresh,inspect:()=>({latest,history,records,runtime:runtime?{id:runtime.id,computedTo:runtime.to,target:runtime.target,mode:runtime.mode,batches:runtime.batches,computeMs:runtime.computeMs,cacheSize:runtimeCache.size,error:runtime.error}:null})};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.RegionEvaluation=api;
})(typeof window==='undefined'?globalThis:window);
