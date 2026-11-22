// Run with PLAYWRIGHT_MODULE pointing at an existing Playwright installation.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),http=require('node:http');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=path.resolve(__dirname,'..'),shots=process.env.FEEDBACK_SCREENSHOTS;
const server=http.createServer((req,res)=>{const name=decodeURIComponent(new URL(req.url,'http://localhost').pathname),file=path.resolve(root,'.'+name);if(file!==root&&!file.startsWith(root+path.sep))return res.writeHead(403).end();fs.readFile(name==='/'?path.join(root,'index.html'):file,(e,b)=>{if(e)return res.writeHead(404).end();res.setHeader('Content-Type',name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':'text/html');res.end(b);});});
(async()=>{
  await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
  try{
    browser=await chromium.launch({headless:true,executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE});
    const page=await browser.newPage({viewport:{width:1600,height:1100}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
    const settle=()=>page.waitForFunction(()=>RegionEvaluation.inspect().history?.ready);
    await page.goto('http://127.0.0.1:'+server.address().port);await page.evaluate(()=>{S.playing=false;S.t=20;render();});await page.locator('[data-view="region"]').click();
    await settle();
    const plots=()=>page.evaluate(()=>{const calls=[],original=RegionDemo.drawComparison;RegionDemo.drawComparison=(id,series,options)=>{if(/^(rdHistory|rdEffect)(Wait|Queue|HighLoad|Diverted|Count|Speed)$/.test(id))calls.push({id,series,options});return original(id,series,options);};try{RegionDemo.refreshProjection();return calls;}finally{RegionDemo.drawComparison=original;}});
    const initial=await plots();assert.equal(initial.length,12);assert.equal(await page.locator('#rdEffectCharts').isVisible(),true);assert.equal(await page.locator('#rdEffectEmpty').count(),0);assert(initial.slice(0,6).every(p=>p.series.length===1&&p.series.every(s=>s.data.length===121)&&p.options.from===600&&p.options.to===1200));assert(initial.slice(6).every(p=>p.series.length===0));
    assert.equal(await page.locator('#rdMaePlot,#rdRmsePlot,#rdMore,#rdStrategies,#rdPerf,#rdEvaluation,#rdObservedFile,#rdCompareRun').count(),0);
    assert.equal(await page.locator('.rd-comparison-charts canvas').count(),12);assert.equal(await page.locator('#rdEffectGroup').isDisabled(),true);
    assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().latest),null);assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.runs.length),8);assert(await page.evaluate(()=>RegionEvaluation.inspect().history.automatic));
    assert.equal(await page.locator('#rdHistoryContext').textContent(),'10:10:00–10:20:00 · RMSE');assert.equal(await page.locator('#rdHistoryMessage').getAttribute('data-state'),'ready');
    await page.locator('#rdViewToggle').click();assert.equal(await page.locator('#rdViewToggle').getAttribute('aria-pressed'),'true');assert.deepEqual(await plots(),initial);await page.keyboard.press('Escape');
    assert.equal(await page.locator('#rdViewToggle').getAttribute('aria-pressed'),'false');assert.equal(await page.evaluate(()=>S.t),20);assert.equal(await page.evaluate(()=>S.playing),false);
    const initialHistory=await page.evaluate(()=>RegionEvaluation.inspect().history);
    await page.evaluate(()=>{S.t=1203/60;render();});await settle();
    let advanced=await page.evaluate(()=>RegionEvaluation.inspect().history);assert.equal(advanced.to,1203);assert.equal(advanced.timeline.at(-1).t,1203);assert(advanced.runs.every(r=>r.timeline.at(-1).t===1203&&r.timeline.every(p=>p.t<=1203)));
    await page.evaluate(()=>{S.t=1206/60;render();});await settle();advanced=await page.evaluate(()=>RegionEvaluation.inspect().history);
    assert.equal(advanced.to,1206);assert(advanced.timeline.some(p=>p.t===1205));assert(!advanced.timeline.some(p=>p.t===1203),'temporary current-second endpoint is replaced by regular historical samples');
    await page.evaluate(()=>{S.t=20;render();});assert.deepEqual(await page.evaluate(()=>RegionEvaluation.inspect().history),initialHistory,'rewinding restores deterministic historical values without future data');
    await page.locator('#rdPlay').click();await page.waitForFunction(()=>RegionEvaluation.inspect().history.to>1200);await page.locator('#rdPlay').click();
    assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().records.length),0,'automatic playback never writes manual strategy records');
    await page.evaluate(()=>{S.t=20;S.playing=false;render();});
    await page.evaluate(()=>{S.t=3599.5/60;S.playing=true;render();});await page.waitForFunction(()=>!S.playing&&S.t===60&&RegionEvaluation.inspect().history.to===3600);await settle();
    assert((await page.locator('#rdTime').textContent()).includes('11:00:00'),'automatic playback flushes the final UI and history before becoming paused');
    await page.evaluate(()=>{S.t=20;S.playing=false;render();});
    if(shots){fs.mkdirSync(shots,{recursive:true});await page.locator('#rdEvaluations').screenshot({path:path.join(shots,'automatic-history-initial.png')});}
    await page.locator('#rdHistoryWindow').selectOption('300');await page.locator('#rdProjectionHorizon').selectOption('300');
    let firstHistory;
    for(const policy of ['baseline','layer','all']){
      await page.locator('#rdCompareStrategy').selectOption(policy);assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.policy.id),policy);
      assert.equal(await page.locator('#rdCompareLayer').isDisabled(),policy!=='layer');if(policy==='layer')await page.locator('#rdCompareLayer').selectOption('2');
      assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().records.length),0,'all historical strategies run without manual records');
      const h=await page.evaluate(()=>RegionEvaluation.inspect().history);assert.equal(h.from,900);assert.equal(h.to,1200);assert.equal(h.timeline.length,61);assert(h.timeline.every(p=>p.t<=1200&&p.t>=900));
      if(policy==='baseline')assert((await plots()).slice(0,6).every(p=>p.series[0].data.every(d=>d.y===0||d.y===null)),'identical baseline models have zero RMSE');
      if(firstHistory){assert.deepEqual(h.timeline,firstHistory.timeline,'history running values are independent of the viewed DT policy');assert.deepEqual(h.runs,firstHistory.runs,'all eight strategies are already running before selection');}else firstHistory=h;
    }
    for(const policy of ['baseline','layer','all']){await page.locator('#rdProjectionStrategy').selectOption(policy);await page.locator('#rdProjectionRun').click();}
    await page.locator('#rdProjectionTime').evaluate(el=>{el.value=3;el.dispatchEvent(new Event('input'));});
    assert.equal(await page.evaluate(()=>RegionDemo.inspect().t),1215);assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.to),1200,'future manual projection playback cannot extend the historical cutoff');
    await page.locator('#rdProjectionTime').evaluate(el=>{el.value=0;el.dispatchEvent(new Event('input'));});
    const actual=await plots(),historyPlots=actual.slice(0,6),effects=actual.slice(6),units=['s/架','架','管道·秒','架','架','m/s'];
    assert.deepEqual(historyPlots.map(p=>p.options.unit),units.map(u=>'RMSE · '+u));assert.deepEqual(effects.map(p=>p.options.unit),units);
    assert.deepEqual((await page.locator('.rd-history-panel figcaption').allTextContents()).map(s=>s.replace(' RMSE','')),await page.locator('.rd-effect-panel figcaption').allTextContents());
    assert((await page.locator('.rd-history-panel canvas').evaluateAll(els=>els.map(el=>el.getAttribute('aria-label')))).every(s=>s.includes('RMSE')));
    const expected=await page.evaluate(()=>{const {latest,history,records}=RegionEvaluation.inspect();return {dt:history.dt,history:history.timeline,records:records.map(r=>r.timeline),keys:RegionEvaluation.metrics.map(p=>p.key)};});
    for(let k=0;k<6;k++){
      const p=historyPlots[k];assert.equal(p.options.from,900);assert.equal(p.options.to,1200);assert.equal(p.options.forecastAt,undefined);assert.equal(p.series.length,1);assert.equal(p.options.xLabel,'时间');
      const pairedErrors=[];const rmse=expected.history.map(f=>{const value=f[expected.keys[k]],prediction=expected.dt.find(d=>d.t===f.t)?.[expected.keys[k]];if(!Number.isFinite(value)||!Number.isFinite(prediction))return {t:f.t,y:null};pairedErrors.push((prediction-value)**2);return {t:f.t,y:Math.sqrt(pairedErrors.reduce((a,b)=>a+b,0)/pairedErrors.length)};});
      assert.deepEqual(p.series[0].data,rmse,'each Y value is the paired cumulative RMSE from the selected window start');
      assert.equal(effects[k].series.length,3);assert.equal(effects[k].options.from,1200);assert.equal(effects[k].options.to,1500);assert.deepEqual(effects[k].series[0].dash,[5,3]);
      assert.deepEqual(effects[k].series.map(s=>s.data),expected.records.map(r=>r.map(f=>({t:f.t,y:f[expected.keys[k]]}))));
    }
    assert(historyPlots.some(p=>p.series[0].data.some(d=>d.y>0)),'a different policy produces nonzero RMSE');
    assert.equal(await page.locator('#rdEffectLegend [role=listitem]').count(),3);assert((await page.locator('#rdHistoryLegend').textContent()).includes('模拟替代'));assert((await page.locator('#rdHistoryLegend').textContent()).includes('RMSE'));
    const before=await plots();await page.locator('#rdRanks .rd-rank').nth(1).click();assert.deepEqual(await plots(),before,'comparison metrics are regional, independent of facility selection');
    if(shots)await page.locator('#rdEvaluations').screenshot({path:path.join(shots,'history-and-strategies.png')});
    await page.locator('#rdViewToggle').click();
    let sceneProjection;
    for(const size of [{width:1920,height:1080},{width:1600,height:900},{width:1280,height:720}]){
      await page.setViewportSize(size);await page.waitForTimeout(100);assert.equal(await page.locator('#rdTimeMeta').isVisible(),false);
      const layout=await page.evaluate(()=>{
        const rect=el=>{const r=el.getBoundingClientRect();return {x:r.x,y:r.y,w:r.width,h:r.height,bottom:r.bottom,right:r.right};};
        const stage=rect(document.getElementById('regionalDemo'));
        return {stage,overflow:document.documentElement.scrollWidth>innerWidth,clipped:[...document.querySelectorAll('#regionalDemo .rd-scene-panel,#regionalDemo .rd-evaluations>.panel,#regionalDemo .rd-lower .panel')].filter(el=>el.scrollHeight>el.clientHeight+2).map(el=>({id:el.id||el.className,height:el.clientHeight,content:el.scrollHeight})),panels:[...document.querySelectorAll('#regionalDemo .rd-comparison-charts canvas,#regionalDemo .rd-metric-note,#regionalDemo .rd-lower,#rdDetail,#rdProjectionTools,#rdViewToggle')].map(el=>({id:el.id||el.className,...rect(el)}))};
      });
      assert(Math.abs(layout.stage.w/layout.stage.h-16/9)<.001);assert(layout.stage.x>=-.1&&layout.stage.y>=-.1);assert(layout.stage.right<=size.width+.1&&layout.stage.bottom<=size.height+.1);
      assert.equal(layout.overflow,false);assert.deepEqual(layout.clipped,[],'all content fits without clipping');for(const r of layout.panels){assert(r.w>0&&r.h>0,r.id+' visible');assert(r.x>=layout.stage.x-.1&&r.right<=layout.stage.right+.1&&r.bottom<=layout.stage.bottom+.1,r.id+' fits 16:9 page '+JSON.stringify(r));}
      const projected=await page.evaluate(()=>{const r=document.getElementById('rdScene').getBoundingClientRect(),p=RegionDemo.project([100,100,135]);return {x:p.x/r.width,y:p.y/r.height};});
      if(sceneProjection){assert(Math.abs(projected.x-sceneProjection.x)<.005,JSON.stringify({projected,sceneProjection}));assert(Math.abs(projected.y-sceneProjection.y)<.005,'scene projection scales uniformly with the page '+JSON.stringify({projected,sceneProjection}));}else sceneProjection=projected;
      if(shots)await page.screenshot({path:path.join(shots,'overview-'+size.width+'x'+size.height+'.png')});
    }
    const scene=await page.locator('#rdScene').boundingBox(),target=await page.evaluate(()=>RegionDemo.project(RegionDemo.objects.get('TK-01').at(.5)));
    await page.mouse.click(scene.x+target.x,scene.y+target.y);assert.equal(await page.evaluate(()=>RegionDemo.inspect().selected),'TK-01','scaled scene retains correct pointer picking');
    assert.deepEqual(await plots(),before,'switching presentation layout does not alter simulation data');
    await page.locator('#rdSlideScenario').selectOption('reserve');await settle();assert.equal(await page.evaluate(()=>S.scenario),'reserve');assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().latest),null);
    assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.scenario),'reserve');assert((await plots()).slice(0,6).every(p=>p.series.every(s=>s.data.length===61)));await page.locator('#rdProjectionRun').click();assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().latest.input.scenario),'reserve');
    await page.locator('#rdSlideRegion').selectOption('0');await settle();assert.equal(await page.evaluate(()=>S.region),0);assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().latest),null);
    assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.region),'R01');await page.locator('#rdProjectionRun').click();assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().latest.input.region),0);
    await page.keyboard.press('Escape');assert(await page.locator('#rdTimeMeta').isVisible());await page.setViewportSize({width:1600,height:1100});await page.locator('#rdProjectionExit').click();
    await page.evaluate(()=>{S.t=0;S.playing=false;render();});await page.locator('#rdProjectionRun').click();
    assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.timeline.length),1);assert((await plots()).slice(0,6).every(p=>p.options.to===0&&p.series.every(s=>s.data.length===1&&s.data[0].t===0)));
    assert((await page.locator('#rdHistoryMessage').textContent()).includes('仅有初始样本'));
    for(let i=0;i<6;i++)await page.locator('#rdProjectionRun').click();assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().records.length),8);
    await page.locator('#rdViewToggle').click();await page.setViewportSize({width:1920,height:1080});
    if(shots)await page.screenshot({path:path.join(shots,'overview-retained-runs.png')});
    assert.deepEqual(await page.evaluate(()=>[...document.querySelectorAll('#regionalDemo .rd-evaluations>.panel,#regionalDemo .rd-scene-panel')].filter(el=>el.scrollHeight>el.clientHeight+2).map(el=>({id:el.className,height:el.clientHeight,content:el.scrollHeight}))),[],'retained run legends and changed scenario controls fit the overview');await page.keyboard.press('Escape');
    const groups=await page.evaluate(()=>[...new Set(RegionEvaluation.inspect().records.map(r=>r.group))]);assert(groups.length>=2);
    await page.locator('#rdEffectGroup').selectOption(String(groups[0]));assert((await plots()).slice(6).every(p=>p.series.length===1));
    await page.locator('#rdEffectClear').click();assert((await plots()).slice(6).every(p=>p.series.length===0));assert.equal(await page.locator('#rdEffectGroup').isDisabled(),true);
    await page.locator('#rdProjectionRun').click();assert((await plots()).slice(6).every(p=>p.series.length===1));
    await page.locator('[data-view="center"]').click();assert(await page.locator('#centerDemo').isVisible());await page.locator('[data-view="region"]').click();
    assert.deepEqual(errors,[]);console.log('PASS: automatic history on initial entry; all eight historical strategies without manual runs; historical simulated observations independent of selected strategy; six cumulative RMSE curves independently verified with zero baseline and nonzero policy errors; RMSE axes and accessible labels; history stops at the current runtime without future points; unchanged right-side original metrics; three selectable policies; isolated groups, eight-record retention and clear/re-run; no pre-start history; removed sample section; 16:9 full overview at 1920/1600/1280, context controls, Escape and data preservation; center navigation and zero page errors');
  }finally{await browser?.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
