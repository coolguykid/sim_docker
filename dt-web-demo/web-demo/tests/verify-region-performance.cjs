// Worker correctness, historical cache, interaction latency and graceful fallback.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),http=require('node:http');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=path.resolve(__dirname,'..'),out=path.resolve(process.env.REGION_PERF_OUTPUT||path.join(root,'../output/performance/region-optimized-20261031'));
const server=http.createServer((req,res)=>{const name=decodeURIComponent(new URL(req.url,'http://localhost').pathname),file=path.resolve(root,'.'+name);if(file!==root&&!file.startsWith(root+path.sep))return res.writeHead(403).end();fs.readFile(name==='/'?path.join(root,'index.html'):file,(e,b)=>{if(e)return res.writeHead(404).end();res.setHeader('Content-Type',name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':'text/html');res.end(b);});});
(async()=>{
  await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
  try{
    browser=await chromium.launch({headless:true,executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE});
    const page=await browser.newPage({viewport:{width:1920,height:1080}}),errors=[],report={browser:browser.version(),viewport:'1920x1080 DPR1',actions:[]};page.on('pageerror',e=>errors.push(e.message));
    const settle=()=>page.waitForFunction(()=>RegionEvaluation.inspect().history?.ready);
    const workStats=()=>page.evaluate(()=>{const {target,...stats}=RegionEvaluation.inspect().runtime;return stats;});
    const base='http://127.0.0.1:'+server.address().port;
    await page.goto(base);await page.evaluate(()=>{S.playing=false;S.t=30;window.__originalRegionalFrame=RegionDemo.frame;RegionDemo.frame=()=>{};CenterDemo.frame=()=>{};window.__mainRunners=0;const create=BranchSimulation.createRunner;BranchSimulation.createRunner=(...a)=>{__mainRunners++;return create(...a);};window.__heartbeats=[];let last=performance.now();window.__heartbeat=setInterval(()=>{const now=performance.now();__heartbeats.push(now-last);last=now;},10);});
    async function action(label,kind,value){
      const start=Date.now(),syncMs=await page.evaluate(({kind,value})=>{const start=performance.now();if(kind==='nav')document.querySelector('[data-view="'+value+'"]').click();else if(kind==='toggle')document.getElementById('rdViewToggle').click();else{const e=document.getElementById('rdTimeSlider');e.value=value;e.dispatchEvent(new Event('input',{bubbles:true}));}window.__actionEnd=performance.now();return __actionEnd-start;},{kind,value});
      const untilPaint=await page.evaluate(()=>new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(()=>r(performance.now()-__actionEnd)))));
      const result={label,syncMs:+syncMs.toFixed(2),toTwoRAFMs:+(syncMs+untilPaint).toFixed(2),wallMs:Date.now()-start};report.actions.push(result);return result;
    }
    const coldStart=Date.now(),cold=await action('cold-center-to-region-1800','nav','region');await settle();report.coldDataReadyMs=Date.now()-coldStart;
    assert(cold.syncMs<1000,'first entry must not synchronously replay history');
    const initial=await page.evaluate(()=>{clearInterval(__heartbeat);return {state:RegionEvaluation.inspect().runtime,mainRunners:__mainRunners,maxHeartbeatGap:Math.max(...__heartbeats)};});
    assert.equal(initial.state.mode,'worker');assert.equal(initial.state.computedTo,1800);assert.equal(initial.mainRunners,0,'history runners live only in the worker');assert(initial.maxHeartbeatGap<1000,'the main thread remains responsive while the worker calculates');report.initial=initial;
    await action('go-center','nav','center');await action('warm-center-to-region-1800','nav','region');assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().runtime.id),initial.state.id);
    await action('details-same-1800','slider',1800);await action('enable-16x9','toggle');await action('wide-same-1800','slider',1800);
    await action('wide-forward-1801','slider',1801);await settle();await action('wide-forward-1830','slider',1830);await settle();
    const cacheBefore=await workStats(),back=await action('wide-backward-1829','slider',1829);await settle();
    assert(back.toTwoRAFMs<1000,'cached rewind must not synchronously replay history');
    await action('wide-backward-1828','slider',1828);await settle();assert.deepEqual(await workStats(),cacheBefore,'rewind changes only the viewed time, with no new runners or computation');
    const forwardStart=Date.now();await action('wide-forward-2400','slider',2400);await settle();report.forwardDataReadyMs=Date.now()-forwardStart;
    const cached=await workStats();
    const combined=await page.evaluate(()=>{const e=document.getElementById('rdTimeSlider');let paints=0;const refresh=RegionEvaluation.refresh;RegionEvaluation.refresh=(...a)=>{paints++;return refresh(...a);};for(const value of [2200,2199,2100,1800,1200,1203]){e.value=value;e.dispatchEvent(new Event('input',{bubbles:true}));}return new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(()=>{RegionEvaluation.refresh=refresh;r({paints,t:RegionDemo.context().currentTime});})));});
    assert.equal(combined.t,1203);assert.equal(combined.paints,1,'six input events are coalesced into one full refresh');assert.deepEqual(await workStats(),cached);report.coalescing=combined;
    // Compare the actual, full regional demand with full-frame aggregation for all eight policies.
    const equivalent=await page.evaluate(()=>{const {history}=RegionEvaluation.inspect(),input=RegionDemo.captureRuntime();for(const run of history.runs){const branch=BranchSimulation.simulate({...input,horizon:history.to},run.policy.id!=='baseline',run.policy.layers),expected=RegionEvaluation.timeline(input,branch.frames).filter(p=>p.t>=history.from&&p.t<=history.to);if(JSON.stringify(expected)!==JSON.stringify(run.timeline))return {ok:false,policy:run.policy};}return {ok:true};});assert(equivalent.ok,JSON.stringify(equivalent));
    await page.screenshot({path:(fs.mkdirSync(out,{recursive:true}),path.join(out,'overview-optimized.png'))});
    await action('wide-forward-3600','slider',3600);await settle();assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().runtime.computedTo),3600);assert(await page.evaluate(()=>RegionEvaluation.inspect().history.runs.every(r=>r.timeline.at(-1).t===3600)));
    const track=await page.locator('#rdTimeSlider').boundingBox();await page.mouse.move(track.x+track.width-8,track.y+track.height/2);await page.mouse.down();await page.mouse.move(track.x+track.width*.35,track.y+track.height/2,{steps:8});await page.mouse.up();await settle();
    const dragged=await page.evaluate(()=>({selected:+document.getElementById('rdTimeSlider').value,current:RegionDemo.context().currentTime,history:RegionEvaluation.inspect().history.to}));assert(dragged.selected>1000&&dragged.selected<1500);assert.equal(dragged.current,dragged.selected);assert.equal(dragged.history,dragged.selected);report.nativeDrag=dragged;
    const pausedPaints=await page.evaluate(()=>new Promise(resolve=>{RegionDemo.frame=__originalRegionalFrame;let calls=0;const refresh=RegionEvaluation.refresh;RegionEvaluation.refresh=(...a)=>{calls++;return refresh(...a);};setTimeout(()=>{RegionEvaluation.refresh=refresh;resolve(calls);},550);}));assert.equal(pausedPaints,0,'paused pages do not redraw their comparisons every 250ms');
    // A change event flushes the final exact time, including without waiting for another animation frame.
    await page.evaluate(()=>{const e=document.getElementById('rdTimeSlider');e.value=1206;e.dispatchEvent(new Event('input'));e.dispatchEvent(new Event('change'));});assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.to),1206);
    await page.evaluate(()=>{S.t=60;render();S.region=0;render();S.region=1;render();S.t=1203/60;S.region=2;render();});await settle();
    const switched=await page.evaluate(()=>RegionEvaluation.inspect());assert.equal(switched.history.region,'R03');assert.equal(switched.history.to,1203);assert.equal(switched.runtime.cacheSize,2);assert(switched.history.runs.every(r=>r.timeline.every(p=>p.t<=1203)));
    report.rapidSwitch={region:switched.history.region,cacheSize:switched.runtime.cacheSize,ready:switched.history.ready};
    await page.evaluate(()=>{S.scenario='reserve';render();S.scenario='peak';render();});await settle();assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.scenario),'peak');
    await page.evaluate(()=>{S.t=60;S.region=3;render();S.t=0;S.region=4;render();});await settle();assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.to),0);assert.equal(await page.evaluate(()=>RegionEvaluation.inspect().history.timeline.length),1);
    // Both construction failure and asynchronous worker failure use the same cooperative engine.
    for(const failure of ['constructor','runtime']){
      const fallback=await browser.newPage({viewport:{width:1600,height:1100}});fallback.on('pageerror',e=>errors.push(e.message));
      if(failure==='constructor')await fallback.addInitScript(()=>{window.Worker=class{constructor(){throw Error('forced worker construction failure');}};});
      else await fallback.route('**/region-history-worker.js',route=>route.fulfill({contentType:'application/javascript',body:'throw Error("forced worker runtime failure");'}));
      await fallback.goto(base);await fallback.evaluate(()=>{S.t=1;S.playing=false;render();});await fallback.locator('[data-view="region"]').click();await fallback.waitForFunction(()=>RegionEvaluation.inspect().history?.ready);
      const state=await fallback.evaluate(()=>RegionEvaluation.inspect());assert.equal(state.runtime.mode,'cooperative');assert.equal(state.history.to,60);assert.equal(state.history.timeline.length,13);assert(state.runtime.error.includes('forced worker'));assert(state.history.runs.every(r=>r.timeline.at(-1).t===60));
      await fallback.close();
    }
    report.errors=errors;assert.deepEqual(errors,[]);report.result='PASS';fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));
    console.log('PASS: real worker, main-thread responsiveness, exact metric equivalence, cached rewind, coalesced input, final change flush, bounded caches, stale-result isolation and both worker failure fallbacks');
  }finally{await browser?.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
