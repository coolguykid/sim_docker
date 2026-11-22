/* Whole-region projection controls embedded in the existing six-layer scene. */
window.RegionProjection=(()=>{
  'use strict';
  let result=null,frame=0,mode='live',timer=null,cached=null;
  const $=id=>document.getElementById(id),fmt=(v,d=0)=>v==null?'—':v.toFixed(d);
  function mount(){
    $('rdProjectionTools').innerHTML=`<strong>区域推演</strong><select id="rdProjectionHorizon" aria-label="全区域推演时长"><option value="300">未来5分钟</option><option value="600" selected>未来10分钟</option><option value="900">未来15分钟</option></select><select id="rdProjectionStrategy" aria-label="区域推演策略"><option value="all">开放全部备用</option><option value="layer">开放指定层备用</option><option value="baseline">维持现状</option></select><select id="rdProjectionLayer" aria-label="备用策略作用层"><option value="0">L1</option><option value="1">L2</option><option value="2">L3</option><option value="3">L4</option><option value="4">L5</option><option value="5">L6</option></select><button id="rdProjectionRun">从当前区域推演</button><button id="rdProjectionBase" disabled>基线</button><button id="rdProjectionPolicy" disabled>开放全区域备用</button><button id="rdProjectionExit" hidden>返回当前态势</button><span id="rdProjectionSource" class="rd-note" aria-live="polite"></span>
      <div id="rdProjectionPlayback" hidden><button id="rdProjectionPlay">播放推演</button><input id="rdProjectionTime" type="range" min="0" value="0" step="1" aria-label="全区域两方案共同推演时间"><strong id="rdProjectionClock"></strong></div>`;
    $('rdProjectionCompare').innerHTML='<div class="rd-projection-summary" id="rdProjectionSummary"></div><div class="rd-small">同一快照与需求；仅新进入的同层同向整段任务允许分流，原有在途及接入任务保留路线。未模拟避碰。每次结果记入下方有效性对比。</div>';
    $('rdProjectionStrategy').onchange=()=>{exit();$('rdProjectionLayer').disabled=$('rdProjectionStrategy').value!=='layer';};
    $('rdProjectionLayer').disabled=true;
    $('rdProjectionLayer').onchange=exit;
    $('rdProjectionRun').onclick=run;
    $('rdProjectionHorizon').onchange=()=>{if(mode!=='live'){exit();$('rdProjectionSource').textContent='时长已改变，请从当前区域重新推演。';}};
    $('rdProjectionBase').onclick=()=>choose('baseline');$('rdProjectionPolicy').onclick=()=>choose('strategy');$('rdProjectionExit').onclick=exit;
    $('rdProjectionTime').oninput=()=>{stop();frame=+$('rdProjectionTime').value;cached=null;paint();};
    $('rdProjectionPlay').onclick=()=>{if(timer){stop();return;}if(frame>=result.baseline.frames.length-1)frame=0;timer=setInterval(()=>{if(document.hidden)return;frame++;cached=null;paint();if(frame>=result.baseline.frames.length-1)stop();},150);$('rdProjectionPlay').textContent='暂停推演';};
  }
  function stop(){clearInterval(timer);timer=null;if($('rdProjectionPlay'))$('rdProjectionPlay').textContent='播放推演';}
  function choose(value){mode=value;cached=null;paint();}
  function run(){
    stop();const start=performance.now();
    try{const input=RegionDemo.captureBranch(+$('rdProjectionHorizon').value);const id=$('rdProjectionStrategy').value,layer=+$('rdProjectionLayer').value,policy={id,label:id==='all'?'开放全部备用':id==='layer'?`开放L${layer+1}备用`:'维持现状',layers:id==='layer'?[layer]:null};result=BranchSimulation.compare(input,policy);result.runMs=performance.now()-start;mode='strategy';frame=0;cached=null;RegionEvaluation.record(result);$('rdProjectionPolicy').textContent=result.policy.label;
      $('rdProjectionSource').textContent='';
      $('rdProjectionTime').max=result.baseline.frames.length-1;paint();
    }catch(error){$('rdProjectionSource').textContent=error.message;}
  }
  function exit(){stop();mode='live';cached=null;paint();}
  function update(region,scenario,time){if(mode!=='live'&&result&&(region!==result.input.region||scenario!==result.input.scenario||Math.abs(time-result.input.time)>.01)){stop();mode='live';cached=null;controls();$('rdProjectionSource').textContent='区域或原始时刻已改变，请从当前区域重新推演。';}}
  function controls(){const active=mode!=='live';$('regionalDemo').dataset.projection=active?'true':'false';$('rdProjectionCompare').hidden=!active;$('rdProjectionPlayback').hidden=!active;$('rdProjectionExit').hidden=!active;$('rdProjectionRun').textContent=active?'重新从快照推演':'从当前区域推演';
    for(const id of ['rdProjectionBase','rdProjectionPolicy'])$(id).disabled=!active;
    for(const [id,value] of [['rdProjectionBase','baseline'],['rdProjectionPolicy','strategy']]){$(id).classList.toggle('active',mode===value);$(id).setAttribute('aria-pressed',String(mode===value));}
    for(const id of ['rdPlay','rdReplay','rdLatest','rdTimeSlider'])$(id).disabled=active;
  }
  function paint(){controls();if(mode!=='live'){
    const a=result.baseline.frames[frame],b=result.strategy.frames[frame],pair=key=>`${a[key]} / ${b[key]}`;
    $('rdProjectionTime').value=frame;$('rdProjectionClock').textContent=`+${a.elapsed}秒 / ${result.input.horizon}秒 · ${mode==='baseline'?'基线':'策略'}`;
    $('rdProjectionSummary').textContent=`当前画面：${mode==='baseline'?'基线 · 维持现状':'策略 · '+result.policy.label} | 在途 ${a.positions.length-a.queue} / ${b.positions.length-b.queue} 架 | 等待 ${pair('queue')} 架 | 完成 ${pair('completed')} 架（基线 / 策略）`;
    }RegionDemo.refreshProjection();
  }
  function snapshot(){if(mode==='live'||!result)return null;if(!cached)cached=BranchSimulation.snapshot(result.input,result[mode],frame);return cached;}
  function label(){return mode==='live'?'':`${mode==='baseline'?'基线':'策略'}推演 · +${result[mode].frames[frame].elapsed}秒`;}
  return {mount,stop,exit,update,snapshot,label,active:()=>mode!=='live',inspect:()=>({result,mode,frame})};
})();
