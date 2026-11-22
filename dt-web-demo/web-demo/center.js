/* Center-only presentation. All fixtures are local and explicitly simulated. */
(function(){
  'use strict';
  const M=window.CenterModel,PU=window.CenterProjectionUI,HU=window.CenterHistoryUI,$=id=>document.getElementById(id);
  const colors=['#4E8398','#51846C','#AC7B2C','#B56655'],layerColors=['#286452','#316E91','#94621E','#78568B','#448A84','#AD6D56'];
  const metrics={u:{name:'容量利用率',unit:'%',scale:100},n:{name:'飞行器数量',unit:'架',scale:1},density:{name:'密度',unit:'架/km',scale:1},speed:{name:'平均速度',unit:'m/s',scale:1},flow:{name:'断面流率合计',unit:'架/分钟',scale:1}};
  const C={metric:'u',quality:'normal',view:'layers',horizon:15,predMetric:'n',yaw:-.55,zoom:1,labels:true,zones:true,pads:true,arrows:true,forecast:true,simulationMode:'history',rate:15,revision:0,events:[],hits:[],movers:[],pickRegion:null};
  let S,enterRegion,initialized=false,lastTick=0,lastFrame=0,animation=0,drag=null,resizeObserver,drawQueued=false,snapshot;
  const f=(v,n=0)=>v===null||v===undefined||!Number.isFinite(v)?'暂无数据':v.toLocaleString('zh-CN',{maximumFractionDigits:n,minimumFractionDigits:n});
  const time=t=>new Date(M.iso(t)).toLocaleTimeString('zh-CN',{timeZone:'Asia/Shanghai',hour:'2-digit',minute:'2-digit',hour12:false});
  const lid=()=>S.layer==='all'?'all':'L'+(Number(S.layer)+1);
  const rid=()=>M.regions[S.region].id;
  const fmtMetric=(v,metric=C.metric)=>v===null?'暂无数据':f(v*metrics[metric].scale,metric==='n'?0:1)+' '+metrics[metric].unit;
  const event=(text)=>{C.events.unshift({time:time(S.t),text});C.events=C.events.slice(0,12);};
  function valueColor(v,metric=C.metric){if(v===null||v===undefined)return '#B8BDB4';const q=metric==='u'?v:metric==='speed'?1-v/24:metric==='density'?v/36:metric==='n'?v/850:v/450;return colors[q<.45?0:q<.65?1:q<.85?2:3];}
  const option=(value,text)=>'<option value="'+value+'">'+text+'</option>';
  const card=(title,note,body,actions='',kind='')=>'<section class="cd-card '+kind+'"><div class="cd-head"><div><h3>'+title+'</h3><p class="cd-note">'+note+'</p></div><div class="cd-actions">'+actions+'</div></div>'+body+'</section>';
  const plot=(id)=>'<div class="cd-chart"><canvas id="'+id+'" role="img"></canvas></div>';
  function init(){
    const el=$('centerDemo');
    el.innerHTML=`
      <div class="cd-intro"><div><h2 class="cd-title">中心数字孪生态势</h2><p class="cd-note">36 个区域 · 6 个方向层 · 本地模拟样例</p></div><div class="cd-chips" id="cdQualityChips"></div><div class="cd-actions cd-view-actions"><div id="cdSlideControls"><label>场景 <select id="cdSlideScenario" aria-label="中心全览模拟场景"></select></label><label>区域 <select id="cdSlideRegion" aria-label="中心全览所选区域"></select></label><button id="cdSlideEnterRegion">查看区域 ↗</button></div><button id="cdViewToggle" aria-pressed="false">切换到16:9全览</button></div></div>
      <div class="cd-kpis" id="cdKpis"></div>
      <div class="cd-dashboard">
        <div class="cd-overview">
          ${card('全域分层航路态势','全区域、全层移动示意 · 首次选区域，再次选层',`<div class="cd-toolbar"><label>视图 <select id="cdView">${option('layers','六层展开')+option('plan','全域俯视')+option('layer','当前单层')}</select></label><label>方向层 <select id="cdLayer">${option('all','全部方向层')+M.layers.map(l=>option(l.index,l.id+' · '+l.angle+'°')).join('')}</select></label><label>着色 <select id="cdMetric">${Object.entries(metrics).filter(([k])=>k!=='flow').map(([k,v])=>option(k,v.name)).join('')}</select></label><button id="cdReset">复位视角</button><button id="cdLocate">定位所选区域</button></div><div class="cd-toolbar"><label><input type="checkbox" id="cdZones" checked>禁飞区</label><label><input type="checkbox" id="cdPads" checked>起降点</label><label><input type="checkbox" id="cdArrows" checked>无人机移动</label><label><input type="checkbox" id="cdLabels" checked>区域标注</label><span class="cd-note">拖动旋转 · 滚轮缩放</span></div><div class="cd-toolbar cd-projection-tools" id="cdProjectionTools"></div><div class="cd-stage"><canvas id="cdScene" aria-label="36区域6方向层模拟拓扑，首次点击选择区域，再次点击选择层"></canvas><div class="cd-overlay" id="cdSceneOverlay"></div><div class="cd-scene-hint">抽样无人机移动示意 · 非全量真实轨迹</div><div class="cd-tooltip" id="cdTooltip"></div></div><div class="cd-legend" id="cdLegend"></div>`,'','cd-scene-card')}
          ${card('区域 × 方向层态势','36 区域 × 6 层 · 点击同步全页',`<div class="cd-heat-scroll"><table class="cd-heat" id="cdHeat" aria-label="36区域乘6方向层态势"></table></div>`,'','cd-heat-card')}
          <aside class="cd-aside">
            ${card('所选区域','',`<div class="cd-body" id="cdDetails"></div><div class="cd-alert-strip" id="cdAlerts"></div>`,`<button id="cdEnterRegion">查看区域 ↗</button>`,'cd-selection-card')}
            ${card('区域利用率排序','点击定位',`<div class="cd-rank" id="cdRanks"></div>`,'','cd-rank-card')}
          </aside>
        </div>
        <div class="cd-analysis">
          ${card('历史态势与预测','所选区域/层 · 实线历史，虚线预测',`<div class="cd-toolbar"><label>时域 <select id="cdHorizon">${[5,10,15,20,30].map(n=>option(n,n+'分钟')).join('')}</select></label><label>指标 <select id="cdPredMetric">${['n','flow','speed','density'].map(k=>option(k,metrics[k].name)).join('')}</select></label><label><input id="cdForecast" type="checkbox" checked>预测</label></div>${plot('cdForecastPlot')}<div class="cd-inline-stats" id="cdAccuracyStats"></div><div id="cdForecastMeta" hidden></div>`,'','cd-forecast-card')}
          ${card('中心DT · 历史观测与当前推演对比','历史运行值：模拟替代 · 全域六项RMSE',HU.cardHTML(),'','cd-history-card')}
          ${card('中心推演 · 策略有效性','同快照 / 同需求',PU.cardHTML(),'','cd-policy-card')}
        </div>
      </div>
      <section class="cd-card cd-timeline"><button id="cdPlay">Ⅱ 暂停</button><label>倍率 <select id="cdRate">${[1,15,30,60].map(n=>option(n,n+'×')).join('')}</select></label><span id="cdClock"></span><input id="cdTime" type="range" min="0" max="60" step="1" aria-label="中心模拟时间，分钟"><button id="cdLive">返回当前样例</button><span class="cd-note" id="cdTimeMode"></span></section>
      <div class="cd-foot">中心 DT · 态势、预测与策略效果均为本地模拟样例 · 未运行科研算法或下发控制</div>`;
    el.addEventListener('click',onClick);el.addEventListener('change',onChange);
    PU.mount({state:()=>S,quality:()=>C.quality,refresh:update,time});
    HU.mount({state:()=>S,quality:()=>C.quality,time:sampleTime,clock:time,plot:(...args)=>window.CenterDemo.drawComparison(...args)});
    $('cdSlideScenario').innerHTML=$('scenario').innerHTML;
    $('cdSlideRegion').innerHTML=M.regions.map(r=>option(r.index,r.id+' · '+r.name)).join('');
    window.addEventListener('resize',resize);
    document.addEventListener('keydown',e=>{if(e.key==='Escape'&&S.view==='center'&&el.classList.contains('cd-slide'))presentation(false);});
    $('cdTime').addEventListener('input',e=>{if(PU.active()){PU.seek(Number(e.target.value));return;}S.t=Number(e.target.value);S.playing=false;C.simulationMode='replay';update();});
    const canvas=$('cdScene');
    canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,lastX:e.clientX,moved:false};canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(drag){if(Math.hypot(e.clientX-drag.x,e.clientY-drag.y)>5)drag.moved=true;if(drag.moved){C.yaw+=(e.clientX-drag.lastX)*.008;drawScene();}drag.lastX=e.clientX;return;}const rect=canvas.getBoundingClientRect(),hit=hitAt(e.clientX-rect.left,e.clientY-rect.top),tip=$('cdTooltip');tip.style.display=hit?'block':'none';if(hit){tip.textContent=C.pickRegion===hit.region?hit.label+' · 点击选择方向层':M.regions[hit.region].id+' · 点击选择区域';tip.style.left=Math.min(rect.width-230,Math.max(8,e.clientX-rect.left+12))+'px';tip.style.top=Math.max(8,e.clientY-rect.top-40)+'px';}});
    canvas.addEventListener('pointerup',e=>{if(drag&&!drag.moved){const rect=canvas.getBoundingClientRect();selectScene(hitAt(e.clientX-rect.left,e.clientY-rect.top));}drag=null;});
    canvas.addEventListener('pointercancel',()=>{drag=null;});canvas.addEventListener('pointerleave',()=>{$('cdTooltip').style.display='none';});
    canvas.addEventListener('wheel',e=>{e.preventDefault();C.zoom=M.clamp(C.zoom*Math.exp(-e.deltaY*.001),.55,2.4);drawScene();},{passive:false});
    resizeObserver=new ResizeObserver(()=>{if(!drawQueued){drawQueued=true;requestAnimationFrame(()=>{drawQueued=false;if(S&&S.view==='center')draw();});}});resizeObserver.observe(el);
    initialized=true;event('中心本地样例已加载');window.DTDesktop?.enhance(el);
  }
  function onClick(e){
    const b=e.target.closest('button');if(!b)return;
    if(b.dataset.region){C.pickRegion=null;S.region=Number(b.dataset.region);if(b.dataset.layer!==undefined)S.layer=b.dataset.layer;update();return;}
    switch(b.id){
      case 'cdViewToggle':presentation(!$('centerDemo').classList.contains('cd-slide'));return;
      case 'cdSlideEnterRegion':
      case 'cdEnterRegion':PU.stop();enterRegion($('centerDemo').classList.contains('cd-slide'));return;
      case 'cdReset':C.yaw=-.55;C.zoom=1;C.focus=false;C.pickRegion=null;S.layer='all';if(C.view==='layer')C.view='layers';update();return;
      case 'cdLocate':C.focus=!C.focus;drawScene();return;
      case 'cdPlay':if(PU.active()){PU.toggle();return;}if(S.t>=60)S.t=0;S.playing=!S.playing;lastTick=performance.now();break;
      case 'cdLive':S.t=30;S.playing=true;C.simulationMode='history';C.frozenTime=null;C.quality='normal';event('返回10:30当前样例，重新载入完整快照');break;
      default:return;
    }update();
  }
  function onChange(e){
    const el=e.target;
    const map={cdView:'view',cdMetric:'metric',cdHorizon:'horizon',cdPredMetric:'predMetric',cdRate:'rate'};
    const checks={cdZones:'zones',cdPads:'pads',cdArrows:'arrows',cdLabels:'labels',cdForecast:'forecast'};
    if(el.id==='cdSlideScenario'){S.scenario=el.value;S.pipe=null;$('scenario').value=el.value;}
    else if(el.id==='cdSlideRegion'){C.pickRegion=null;S.region=Number(el.value);S.pipe=null;}
    else if(el.id==='cdLayer'){C.pickRegion=null;S.layer=el.value;if(C.view==='layer'&&S.layer==='all')S.layer='0';}
    else if(map[el.id]){C[map[el.id]]=el.id==='cdHorizon'||el.id==='cdRate'?Number(el.value):el.value;if(el.id==='cdView'){C.pickRegion=null;if(C.view==='layer'&&S.layer==='all')S.layer='0';}}
    else if(checks[el.id])C[checks[el.id]]=el.checked;else return;
    update();
  }
  function sampleTime(){return C.quality==='offline'?C.frozenTime??S.t:S.t;}
  const displayTime=()=>PU.current()?.time??sampleTime();
  function update(){
    if(!S||S.view!=='center')return;
    PU.sync();if(PU.active()||PU.busy())S.playing=false;
    const t=sampleTime();snapshot=PU.snapshot(lid())||M.state(t,S.scenario,C.quality,lid());if(C.pickRegion!==S.region)C.pickRegion=null;
    const top=PU.active()?PU.input().topology:M.topology(t,C.quality),selected=snapshot.regions[S.region];
    $('pageTitle').textContent='中心数字孪生态势';$('enterRegion').textContent='查看区域 ↗';$('regionSelect').value=S.region;
    $('cdSlideRegion').value=S.region;$('cdSlideScenario').value=S.scenario;
    $('clock').textContent=time(displayTime());$('scopeText').textContent='全域 · '+(lid()==='all'?'六层聚合':lid())+' · 中心模拟样例';
    document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view==='center'));
    const summary=snapshot.summary;
    const utilizationState=summary.u===null?'unknown':summary.u>=.85?'danger':summary.u>=.65?'warning':'normal';
    const utilizationLabel={unknown:'暂无状态',danger:'高负载',warning:'负载上升',normal:'正常'}[utilizationState];
    const kpis=[['有效样本内飞行器',f(summary.n),'架 · '+(lid()==='all'?'六层':lid())],['断面流率合计',f(summary.flow),'架/分钟 · 非边界吞吐'],['平均密度',f(summary.density,1),'架/km · 按管道长度'],['加权平均速度',f(summary.speed,1),'m/s · 按数量加权'],['容量利用率',summary.u===null?'暂无数据':f(summary.u*100,1)+'%','有效数量 / 有效容量'],['数据完整性',f(summary.valid)+' / '+summary.total,'区域×层样本 · 缺失 '+summary.missing]];
    $('cdKpis').innerHTML=kpis.map(([a,b,c],i)=>{const state=i===4?' data-state="'+utilizationState+'"':'';const note=i===4?'<span class="cd-kpi-state">'+utilizationLabel+'</span> · ':'';return '<div class="cd-kpi"'+state+'><span>'+a+'</span><b>'+b+'</b><small>'+note+c+'</small></div>';}).join('');
    $('cdQualityChips').innerHTML='<span class="cd-chip">本地样例</span><span class="cd-chip '+(C.quality==='offline'?'warn':'')+'">'+(C.quality==='offline'?'模拟断线 · 保留最后快照':'本地离线演示')+'</span><span class="cd-chip '+(summary.missing||summary.stale?'warn':'')+'">'+(summary.missing?'缺失 '+summary.missing+' 个样本':summary.stale?'部分区域采样过期':'区域数据完整')+'</span><span class="cd-chip purple">'+(top.changed?'区域版本 C1 / C2':'区域版本 C1')+'</span>';
    for(const [id,val]of Object.entries({cdView:C.view,cdLayer:S.layer,cdMetric:C.metric,cdHorizon:PU.active()?PU.input().horizon/60:C.horizon,cdPredMetric:C.predMetric,cdRate:C.rate}))$(id).value=val;
    const projected=PU.active(),frame=PU.current();
    $('cdTime').max=projected?PU.input().horizon:60;$('cdTime').step=projected?PU.input().step:1;$('cdTime').value=projected?frame.elapsed:S.t;$('cdTime').disabled=PU.busy();$('cdTime').setAttribute('aria-label',projected?'中心两方案共同推演时间，秒':'中心模拟时间，分钟');
    $('cdClock').textContent='2026-10-20 '+time(displayTime())+' 北京时间'+(projected?' · 推演 +'+frame.elapsed+'秒':'');$('cdPlay').disabled=PU.busy();$('cdLive').disabled=projected||PU.busy();$('cdHorizon').disabled=projected;
    $('cdPlay').textContent=(projected?PU.playing():S.playing)?'Ⅱ 暂停':'▶ 播放';$('cdTimeMode').textContent=projected?(PU.inspect().mode==='baseline'?'基线':'策略')+'推演 / '+PU.input().horizon+'秒 · '+C.rate+'×':(C.simulationMode==='replay'?'历史回放':'本地模拟播放')+' · '+C.rate+'×';
    renderOverview(selected,top);renderPrediction();PU.render();
    draw();C.revision++;
  }
  function renderOverview(selected,top){
    const utilizationState=selected.u==null?'unknown':selected.u>=.85?'danger':selected.u>=.65?'warning':'normal';
    const fields=[['方向层',lid()==='all'?'全部六层':lid()],['业务高度',lid()==='all'?'135–785 m':M.layers[Number(S.layer)].height+' m'],['数量',f(selected.n)+' 架'],['均速',f(selected.speed,1)+' m/s'],['密度',f(selected.density,1)+' 架/km'],['利用率',fmtMetric(selected.u,'u'),'cd-value-'+utilizationState],['拓扑版本',top.versions[rid()]],['直接邻域',selected.neighbors.join('、')]];
    $('cdDetails').innerHTML='<div class="cd-detail-id">'+rid()+' <small>'+selected.name+'</small></div><dl class="cd-detail-grid">'+fields.map(([a,b,state=''])=>'<div><dt>'+a+'</dt><dd class="'+state+'">'+b+'</dd></div>').join('')+'</dl>';
    const sorted=[...snapshot.regions].sort((a,b)=>(b.u??-1)-(a.u??-1));
    $('cdRanks').innerHTML=sorted.map(r=>'<button data-region="'+r.index+'" class="'+(r.index===S.region?'active':'')+'"><span class="cd-rank-line"><span>'+r.id+(r.stale?' · 过期':'')+'</span><span>'+fmtMetric(r.u,'u')+'</span></span><span class="cd-bar"><i style="width:'+Math.min(100,(r.u??0)*100)+'%;background:'+valueColor(r.u,'u')+'"></i></span></button>').join('');
    $('cdHeat').innerHTML='<thead><tr><th>区域</th>'+M.layers.map(l=>'<th>'+l.id+' · '+l.angle+'°</th>').join('')+'</tr></thead><tbody>'+M.regions.map(r=>'<tr><th><button data-region="'+r.index+'">'+r.id+'</button></th>'+M.layers.map(l=>{const a=snapshot.cells[r.index*6+l.index],chosen=S.region===r.index&&(S.layer==='all'||Number(S.layer)===l.index),v=a[C.metric];return '<td><button data-region="'+r.index+'" data-layer="'+l.index+'" class="'+(chosen?'selected ':'')+(a.missing?'missing ':'')+(a.stale?'stale':'')+'" style="--cell:'+valueColor(v)+'" title="'+r.id+' / '+l.id+' · '+fmtMetric(v)+(a.reason?' · '+a.reason:'')+'">'+(v===null?(a.missing?'缺失':'无样本'):f(v*metrics[C.metric].scale,C.metric==='speed'||C.metric==='density'?1:0))+(a.stale?' ⏱':'')+'</button></td>';}).join('')+'</tr>').join('')+'</tbody>';
    const high=snapshot.regions.filter(r=>r.u>=.85),missing=snapshot.regions.filter(r=>r.valid===0);
    $('cdAlerts').textContent='高负载 ≥85%：'+high.length+' 个区域 · 缺失：'+missing.length+' 个区域';
    const ranges=C.metric==='u'?['<45%','45–65%','65–85%','≥85%']:C.metric==='speed'?['>13.2','8.4–13.2','3.6–8.4','≤3.6']:C.metric==='n'?['<383','383–553','553–723','≥723']:['<16.2','16.2–23.4','23.4–30.6','≥30.6'];
    $('cdLegend').innerHTML='<strong>'+metrics[C.metric].name+' / '+metrics[C.metric].unit+'</strong>'+ranges.map((v,i)=>'<span><i class="cd-swatch" style="background:'+colors[i]+'"></i>'+v+'</span>').join('')+'<span>▧ 缺失</span><span>⏱ 过期</span><span>紫色：单向层间连接</span>';
    $('cdSceneOverlay').innerHTML='<strong>'+rid()+' / '+(lid()==='all'?'六方向层':lid())+'</strong><span>'+time(displayTime())+' · '+(PU.active()?'宏观推演 +'+PU.current().elapsed+'秒':'中心本地样例')+'</span><br><span>'+(C.pickRegion===S.region?(lid()==='all'?'区域已选中 · 再次点击选择方向层':'已选择 '+lid()+' · 点击其他区域重新选层'):'首次点击选择区域 · 再次点击选择方向层')+'</span>';
  }
  function renderPrediction(){
    const p=M.prediction(rid(),sampleTime(),C.horizon,S.scenario,C.quality,C.predMetric,lid());
    $('cdForecastMeta').textContent=rid()+' / '+lid()+' · '+time(sampleTime())+' 起报（居中） · 左右各'+C.horizon+'分钟 · 起点前留白';
    if(PU.active())$('cdForecastMeta').textContent=rid()+' / '+lid()+' · '+time(sampleTime())+' 起报（居中） · 当前方案宏观推演 '+PU.input().horizon/60+'分钟';
    $('cdAccuracyStats').textContent=(PU.active()?'起报历史误差：MAE ':'历史误差：MAE ')+f(p.mae,2)+' / RMSE '+f(p.rmse,2)+' '+metrics[C.predMetric].unit+' · 样本 '+p.evaluation.filter(v=>v.actual!==null).length;
    $('cdAccuracyStats').title='预测与观测均为模拟样例，固定15分钟提前量，仅对已到达的历史观测计算误差；不评价本次未来推演。';
  }
  const canvasScales=new WeakMap();
  function displayScale(canvas){const el=$('centerDemo'),scale=Number.parseFloat(el?.style.getPropertyValue('--cd-slide-scale'));return el?.classList.contains('cd-slide')&&(!canvas||el.contains(canvas))&&Number.isFinite(scale)&&scale>0?scale:1;}
  const scaleFor=c=>canvasScales.get(c)||1;
  function context(id){
    const canvas=$(id);if(!canvas)return null;const box=canvas.getBoundingClientRect();if(box.width<5||box.height<5)return null;
    const dpr=Math.min(2,devicePixelRatio||1);if(canvas.width!==Math.round(box.width*dpr)||canvas.height!==Math.round(box.height*dpr)){canvas.width=Math.round(box.width*dpr);canvas.height=Math.round(box.height*dpr);}
    const c=canvas.getContext('2d');canvasScales.set(c,displayScale(canvas));c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,box.width,box.height);c.font='12px "Segoe UI","Microsoft YaHei",sans-serif';return {c,w:box.width,h:box.height,canvas};
  }
  function polygon(c,points,fill,stroke,width=1,dash=[]){const scale=scaleFor(c);c.beginPath();points.forEach((p,i)=>i?c.lineTo(p.x,p.y):c.moveTo(p.x,p.y));c.closePath();if(fill){c.fillStyle=fill;c.fill();}if(stroke){c.strokeStyle=stroke;c.lineWidth=width*scale;c.setLineDash(dash.map(v=>v*scale));c.stroke();c.setLineDash([]);}}
  function line(c,a,b,color,width=1,dash=[]){const scale=scaleFor(c);c.beginPath();c.moveTo(a.x,a.y);c.lineTo(b.x,b.y);c.strokeStyle=color;c.lineWidth=width*scale;c.setLineDash(dash.map(v=>v*scale));c.stroke();c.setLineDash([]);}
  function arrow(c,a,b,color,width=1){line(c,a,b,color,width);const angle=Math.atan2(b.y-a.y,b.x-a.x),head=7*scaleFor(c);polygon(c,[b,{x:b.x-head*Math.cos(angle-.45),y:b.y-head*Math.sin(angle-.45)},{x:b.x-head*Math.cos(angle+.45),y:b.y-head*Math.sin(angle+.45)}],color);}
  function text(c,s,x,y,color='#647169',size=12){c.fillStyle=color;c.font=(size*scaleFor(c))+'px "Segoe UI","Microsoft YaHei",sans-serif';c.fillText(s,x,y);}
  function dot(c,p,r,color){c.beginPath();c.arc(p.x,p.y,r*scaleFor(c),0,Math.PI*2);c.fillStyle=color;c.fill();}
  function inside(p,points){let yes=false;for(let i=0,j=points.length-1;i<points.length;j=i++){const a=points[i],b=points[j];if((a.y>p.y)!==(b.y>p.y)&&p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)yes=!yes;}return yes;}
  function hitAt(x,y){const hits=[...C.hits].reverse(),contains=v=>v.polygon?inside({x,y},v.polygon):Math.hypot(v.x-x,v.y-y)<v.radius;return (C.pickRegion===S.region&&hits.find(v=>v.region===S.region&&v.polygon&&contains(v)))||hits.find(contains);}
  function selectScene(hit){
    if(!hit){C.pickRegion=null;S.layer='all';if(C.view==='layer')C.view='layers';update();return;}
    if(C.pickRegion!==hit.region){S.region=hit.region;S.layer='all';C.pickRegion=hit.region;C.view='layers';}
    else {const layer=hit.layer??hit.pair?.[1];if(layer!==null&&layer!==undefined)S.layer=String(layer);}
    S.pipe=null;update();
  }
  function drawScene(){
    const ctx=context('cdScene');if(!ctx||!snapshot)return;const {c,w,h}=ctx,top=PU.active()?PU.input().topology:M.topology(sampleTime(),C.quality),selected=M.regions[S.region];C.hits=[];C.movers=[];
    const scale=Math.min(w/7100,h/(C.view==='layers'?5500:4300))*C.zoom,focus=C.focus?selected.center:{x:0,y:0};
    const sceneAnchor=.79;
    const P=(x,y,z=0)=>{const a=x-focus.x,b=y-focus.y,xx=a*Math.cos(C.yaw)-b*Math.sin(C.yaw),yy=a*Math.sin(C.yaw)+b*Math.cos(C.yaw);return {x:w/2+xx*scale,y:C.view==='layers'?h*sceneAnchor+(yy*.27-z*3.5)*scale:h*.53+yy*(C.view==='plan'?.72:.48)*scale};};
    const levelIds=C.view==='plan'?[-1]:lid()!=='all'?[Number(S.layer)]:[0,1,2,3,4,5];
    for(const li of levelIds){const z=li<0?0:top.layers[li].height;
      for(const r of top.regions){const a=li<0?snapshot.regions[r.index]:snapshot.cells[r.index*6+li],col=valueColor(a[C.metric]),points=r.polygon.map(p=>P(p.x,p.y,z));
        polygon(c,points,a.missing?'#E0E0D8':col+(li<0?'24':'19'),r.index===S.region?'#286452':col+(a.stale?'D0':'65'),r.index===S.region?2:1,a.stale?[4,3]:[]);
        if(a.missing){const center=P(r.center.x,r.center.y,z);text(c,'缺失',center.x-13*scaleFor(c),center.y,'#777D72',12);}
        C.hits.push({region:r.index,layer:li<0?null:li,polygon:points,label:r.id+(li<0?'':' / '+top.layers[li].id)+' · '+fmtMetric(a[C.metric])+(a.stale?' · 过期':'')});
        if(C.arrows&&!a.missing&&a.n>0)for(const layer of li<0?top.layers:[top.layers[li]]){
          const cell=snapshot.cells[r.index*6+layer.index];if(cell.missing||!cell.n)continue;
          const angle=layer.angle*Math.PI/180;
          for(let i=0;i<2;i++){
            const phase=(animation*(.08+cell.speed/240)+r.index*.173+layer.index*.113+i*.5)%1,offset=(i-.5)*.22,u=.5+Math.cos(angle)*(phase-.5)*.72+Math.sin(angle)*offset,v=.5+Math.sin(angle)*(phase-.5)*.72-Math.cos(angle)*offset;
            const [a,b,d,e]=r.polygon,point={x:(1-u)*(1-v)*a.x+u*(1-v)*b.x+u*v*d.x+(1-u)*v*e.x,y:(1-u)*(1-v)*a.y+u*(1-v)*b.y+u*v*d.y+(1-u)*v*e.y},p=P(point.x,point.y,z);
            dot(c,p,2,layerColors[layer.index]);C.movers.push({id:r.id+'-'+layer.id+'-'+i,regionId:r.id,layerId:layer.id,x:p.x,y:p.y});
          }
        }
      }
      if(C.arrows)for(const edge of top.connections){const from=top.regions.find(r=>r.id===edge.from),to=top.regions.find(r=>r.id===edge.to),a=P(from.center.x,from.center.y,z),b=P(to.center.x,to.center.y,z),isSelected=from.index===S.region||to.index===S.region;line(c,a,b,edge.open?(isSelected?'#527F7280':'#788B7730'):'#94621EA0',isSelected?1.2:.6,edge.open?[]:[4,4]);}
      if(C.labels&&(li===levelIds[levelIds.length-1]||li<0))for(const r of top.regions){const p=P(r.center.x,r.center.y,z),scale=scaleFor(c);text(c,r.id,p.x-10*scale,p.y-3*scale,r.index===S.region?'#234E3D':'#6B7E70',r.index===S.region?14:12);}
      if(li>=0){const p=P(-2900,2900,z);text(c,top.layers[li].id+' · '+top.layers[li].angle+'° · '+top.layers[li].height+'m',p.x,p.y,layerColors[li],12);}
    }
    if(C.view==='layers'&&lid()==='all')for(let li=0;li<5;li++){const x=selected.center.x+220,y=selected.center.y,a=P(x,y,top.layers[li].height),b=P(x+130,y+80,top.layers[li+1].height);arrow(c,a,b,'#78568BA0',2);const mid={x:(a.x+b.x)/2,y:(a.y+b.y)/2};C.hits.push({...mid,radius:12*displayScale(),region:S.region,layer:null,pair:[li,li+1],label:rid()+' / '+top.layers[li].id+' → '+top.layers[li+1].id+' · 模拟单向转移'});arrow(c,{x:b.x+8*scaleFor(c),y:b.y},{x:a.x+8*scaleFor(c),y:a.y},'#316E9170',1);}
    if(C.zones)for(const zone of top.zones){const bottom=zone.polygon.map(p=>P(p.x,p.y,0)),upper=zone.polygon.map(p=>P(p.x,p.y,C.view==='plan'?0:zone.top));polygon(c,bottom,'#B5665510','#AA514680',1,[4,3]);if(C.view!=='plan'){polygon(c,upper,'#B5665510','#AA514660',1,[4,3]);bottom.forEach((p,i)=>line(c,p,upper[i],'#AA514650',.8,[3,3]));}text(c,zone.id+' 禁飞',upper[0].x,upper[0].y-5*scaleFor(c),'#AA5146',12);}
    if(C.pads)for(const pad of top.pads){const p=P(pad.x,pad.y,0),scale=scaleFor(c);dot(c,p,5,'#FFFEFA');c.strokeStyle='#316E91';c.lineWidth=1.5*scale;c.beginPath();c.arc(p.x,p.y,5*scale,0,Math.PI*2);c.stroke();text(c,'H',p.x-3*scale,p.y+3*scale,'#316E91',12);text(c,pad.id,p.x+8*scale,p.y+3*scale,'#316E91',12);}
    text(c,'N ↑   E →',w-90*scaleFor(c),25*scaleFor(c),'#647169',12);
  }
  const chartCache=new Map();
  function chart(id,series,{unit='',xMin,xMax,yMin=0,yMax,band=null,vertical=null,verticalLabel='起报',percent=false,centered=false,axes=false,xLabel='',legend=true,empty='当前范围暂无有效样本'}={}){
    const el=$(id),box=el.getBoundingClientRect(),stamp=JSON.stringify([unit,xMin,xMax,yMin,yMax,band,vertical,verticalLabel,percent,centered,axes,xLabel,legend,empty,box.width,box.height,devicePixelRatio,displayScale(el)]),cached=chartCache.get(id);
    if(cached?.series===series&&cached.stamp===stamp)return;chartCache.set(id,{series,stamp});
    const ctx=context(id);if(!ctx)return;const {c,w,h,canvas}=ctx,scale=scaleFor(c),p={left:44*scale,right:(centered?44:24)*scale,top:28*scale,bottom:(xLabel?34:20)*scale};
    const valid=series.flatMap(s=>s.points).filter(v=>v.y!==null&&Number.isFinite(v.y));
    if(!valid.length){c.textAlign='center';text(c,empty,w/2,h/2,'#5D6C63',13);c.textAlign='left';canvas.setAttribute('aria-label',empty);return;}
    xMin=xMin??Math.min(...valid.map(v=>v.x));xMax=xMax??Math.max(...valid.map(v=>v.x));yMax=yMax??Math.max(1,...valid.map(v=>v.y))*1.18;
    const X=x=>p.left+(x-xMin)/(xMax-xMin||1)*(w-p.left-p.right),Y=y=>h-p.bottom-(y-yMin)/(yMax-yMin||1)*(h-p.top-p.bottom);
    const yTicks=h/scale<130?3:4;for(let i=0;i<=yTicks;i++){const v=yMin+(yMax-yMin)*i/yTicks,y=Y(v);line(c,{x:p.left,y},{x:w-p.right,y},'#E3E6DC');text(c,f(v,percent?0:v<10?1:0),5*scale,y+3*scale);}
    for(let i=0;i<3;i++){const v=xMin+(xMax-xMin)*i/2;c.textAlign='center';text(c,time(v),X(v),h-(xLabel?18:9)*scale);}if(xLabel)text(c,xLabel,(p.left+w-p.right)/2,h-3*scale);c.textAlign='left';
    text(c,unit,5*scale,13*scale);
    if(axes){line(c,{x:p.left,y:p.top},{x:p.left,y:h-p.bottom},'#9BAA9F');line(c,{x:p.left,y:h-p.bottom},{x:w-p.right,y:h-p.bottom},'#9BAA9F');}

    if(vertical!==null){const x=X(vertical);line(c,{x,y:p.top},{x,y:h-p.bottom},'#9AAFA1',1,[3,3]);if(verticalLabel)text(c,verticalLabel,Math.min(w-30*scale,x+4*scale),p.top+12*scale);}
    if(band){let segment=[];const flush=()=>{if(segment.length>1)polygon(c,[...segment.map(v=>({x:X(v.x),y:Y(v.high)})),...segment.slice().reverse().map(v=>({x:X(v.x),y:Y(v.low)}))],'#316E9115');segment=[];};for(const v of band){if(v.low===null){flush();continue;}segment.push(v);}flush();}
    series.forEach((s,index)=>{c.strokeStyle=s.color;c.lineWidth=1.8*scale;c.setLineDash((s.dash||[]).map(v=>v*scale));c.beginPath();let connected=false;for(const v of s.points){if(v.y===null||!Number.isFinite(v.y)){connected=false;continue;}if(connected)c.lineTo(X(v.x),Y(v.y));else c.moveTo(X(v.x),Y(v.y));connected=true;}c.stroke();c.setLineDash([]);if(s.dots)for(const v of s.points)if(Number.isFinite(v.y))dot(c,{x:X(v.x),y:Y(v.y)},1.7,s.color);if(legend){const xx=p.left+index*Math.min(135*scale,(w-p.left-p.right)/series.length);text(c,s.name,xx,p.top+5*scale,s.color,12);}});
    canvas.setAttribute('aria-label',unit+'，'+series.map(s=>s.name).join('、')+'，'+time(xMin)+'至'+time(xMax));
  }
  function drawPrediction(){
    const review=$('centerDemo').classList.contains('cd-slide');
    const t=sampleTime(),span=PU.active()?PU.input().horizon/60:C.horizon,p=M.prediction(rid(),t,span,S.scenario,C.quality,C.predMetric,lid()),series=[{name:'模拟历史',color:'#286452',points:p.history.filter(v=>v.t>=t-span).map(v=>({x:v.t,y:v.value}))}];
    if(PU.active()){
      if(C.forecast)series.push({name:'当前方案推演',color:'#7556A0',dash:[5,4],points:PU.frames().map(frame=>({x:frame.time,y:M.aggregate(frame.cells.filter(v=>v.regionId===rid()&&(lid()==='all'||v.layerId===lid())))[C.predMetric]}))});
      chart('cdForecastPlot',series,{unit:metrics[C.predMetric].unit,xMin:t-span,xMax:t+span,vertical:t,verticalLabel:review?'':'起报',legend:!review,centered:true,axes:true});return;
    }
    if(C.forecast)series.push({name:'模拟预测',color:'#316E91',dash:[5,4],points:p.future.map(v=>({x:v.t,y:v.value}))});
    chart('cdForecastPlot',series,{unit:metrics[C.predMetric].unit,xMin:t-span,xMax:t+span,vertical:t,verticalLabel:review?'':'起报',legend:!review,centered:true,axes:true,band:C.forecast?p.future.map(v=>({x:v.t,low:v.low,high:v.high})):null});
  }

  function drawPolicy(){PU.draw(window.CenterDemo.drawComparison);}
  function draw(){if(!initialized||!S||S.view!=='center')return;drawScene();drawPrediction();drawPolicy();HU.refresh();}
  function fitPresentation(){const scale=Math.min(innerWidth/1920,innerHeight/1080),el=$('centerDemo');el.style.setProperty('--cd-slide-scale',scale);el.style.setProperty('--cd-slide-left',(innerWidth-1920*scale)/2+'px');el.style.setProperty('--cd-slide-top',(innerHeight-1080*scale)/2+'px');}
  function presentation(enabled){const el=$('centerDemo');el.classList.toggle('cd-slide',enabled);document.body.classList.toggle('center-slide',enabled);$('cdViewToggle').textContent=enabled?'返回详情视图':'切换到16:9全览';$('cdViewToggle').setAttribute('aria-pressed',String(enabled));if(enabled)fitPresentation();requestAnimationFrame(draw);}
  function resize(){if(!initialized||S.view!=='center')return;if($('centerDemo').classList.contains('cd-slide'))fitPresentation();draw();}
  function render(state,goRegion){S=state;enterRegion=goRegion;PU.stop();if(performance.now()-lastFrame>200)lastTick=performance.now();if(!initialized)init();$('centerDemo').hidden=false;if($('centerDemo').classList.contains('cd-slide'))fitPresentation();update();}
  function frame(now){
    if(!S||S.view!=='center'){lastTick=now;return;}
    if(!lastTick)lastTick=now;
    if(document.hidden){lastTick=now;lastFrame=now;return;}
    if(PU.active()){PU.tick(now,C.rate);if(now-lastFrame>=33){const dt=lastFrame?(now-lastFrame)/1000:0;lastFrame=now;if(PU.playing()&&C.arrows&&!matchMedia('(prefers-reduced-motion: reduce)').matches){animation+=Math.min(.1,dt);drawScene();}}return;}
    if(S.playing&&now-lastTick>=60000/C.rate){const steps=Math.floor((now-lastTick)/(60000/C.rate));S.t=Math.min(60,S.t+steps);lastTick=now;if(S.t===60)S.playing=false;update();}
    if(now-lastFrame>=33){const dt=lastFrame?(now-lastFrame)/1000:0;lastFrame=now;if(S.playing&&C.arrows&&!matchMedia('(prefers-reduced-motion: reduce)').matches){animation+=Math.min(.1,dt);drawScene();}}
  }
  function inspect(){return {layout:'dashboard',quality:C.quality,regionId:rid(),layerId:lid(),time:S.t,sampleTime:sampleTime(),source:'MOCK-CENTER-01',summary:snapshot.summary,versions:M.topology(sampleTime(),C.quality).versions,horizon:C.horizon,metric:C.metric,policy:PU.inspect().policy,projection:{active:PU.active(),busy:PU.busy(),mode:PU.inspect().mode,elapsed:PU.inspect().elapsed,playing:PU.playing()},revision:C.revision,events:C.events,sceneHits:C.hits.length,sceneVehicles:C.movers.map(v=>({...v})),selectionStage:C.pickRegion===S.region?'layer':'region'};}
  window.CenterDemo={render,frame,resize,inspect,presentation,drawComparison:chart};
})();
