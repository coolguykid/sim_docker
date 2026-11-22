/* Uses the same existing Playwright installation as verify-center-presentation.cjs. */
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),http=require('node:http');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright'),Model=require('../center-model.js'),Projection=require('../center-projection.js');
const root=path.resolve(__dirname,'..'),out=path.resolve(root,'../output/playwright/center-projection');
const server=http.createServer((req,res)=>{const url=decodeURIComponent(new URL(req.url,'http://localhost').pathname),file=path.resolve(root,'.'+(url==='/'?'/index.html':url));if(!file.startsWith(root+path.sep))return res.writeHead(403).end();fs.readFile(file,(err,data)=>{if(err)return res.writeHead(404).end();res.setHeader('Content-Type',url.endsWith('.js')?'text/javascript':url.endsWith('.css')?'text/css':'text/html');res.end(data);});});
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;const report={checks:[],errors:[],layouts:[],seekMs:[]};
  try{
    fs.mkdirSync(out,{recursive:true});browser=await chromium.launch({headless:true,executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE});
    const page=await browser.newPage({viewport:{width:1920,height:1080},reducedMotion:'reduce'}),url='http://127.0.0.1:'+server.address().port;let workers=0;
    page.on('pageerror',e=>report.errors.push(e.message));page.on('worker',worker=>{if(worker.url().endsWith('center-projection-worker.js'))workers++;});
    const record=async(name,fn)=>{await fn();report.checks.push(name);console.log('PASS '+name);};
    const settle=()=>page.evaluate(()=>new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve))));
    const run=async()=>{await page.locator('#cdProjectionRun').click();await page.waitForFunction(()=>CenterDemo.inspect().projection.active&&!CenterDemo.inspect().projection.busy);};
    const seek=async seconds=>page.locator('#cdTime').evaluate((el,seconds)=>{const started=performance.now();el.value=seconds;el.dispatchEvent(new Event('input',{bubbles:true}));return performance.now()-started;},seconds);
    await page.goto(url);await page.evaluate(()=>{S.t=30;S.region=14;S.layer='all';S.scenario='peak';S.playing=false;render();});
    const original=await page.evaluate(()=>CenterDemo.inspect().summary),source=Model.state(30,'peak'),expected=Projection.compare(Projection.capture(30,'peak',300),'balance');
    await record('no fabricated strategy curves or projection worker before manual execution',async()=>{assert.equal(workers,0);assert.equal(await page.locator('#cdPolicyPlot').getAttribute('aria-label'),'尚无推演记录');assert.equal(await page.evaluate(()=>CenterProjectionUI.inspect().records.length),0);assert(await page.locator('#cdProjectionBase').isDisabled());assert.equal(await page.locator('#cdTime').getAttribute('max'),'60');});
    await record('background calculation captures all 216 cells and starts both branches at the same snapshot',async()=>{
      await page.selectOption('#cdProjectionHorizon','300');await page.evaluate(()=>{window.projectionTicks=0;window.projectionTimer=setInterval(()=>window.projectionTicks++,10);});await run();
      const state=await page.evaluate(()=>{clearInterval(window.projectionTimer);const p=CenterProjectionUI.inspect();return {ticks:window.projectionTicks,time:S.t,playing:S.playing,elapsed:p.elapsed,scope:p.result.input.cells.length,firstA:p.result.baseline.frames[0].cells,firstB:p.result.strategy.frames[0].cells,recordCount:p.records.length};});
      assert.equal(workers,1);assert(state.ticks>0);assert.equal(state.time,30);assert.equal(state.playing,false);assert.equal(state.elapsed,0);assert.equal(state.scope,216);assert.deepEqual(state.firstA,source.cells);assert.deepEqual(state.firstB,source.cells);assert.deepEqual(await page.evaluate(()=>CenterDemo.inspect().summary),original);assert.equal(state.recordCount,1);assert.equal(await page.locator('#cdTime').getAttribute('max'),'300');assert.equal(await page.locator('#cdTime').getAttribute('step'),'5');
    });
    await record('shared replay updates KPI, heat cells, scene metadata and details using the calculated branch',async()=>{
      report.seekMs.push(await seek(300));const expectedFrame=expected.strategy.frames.at(-1),expectedSnapshot=Projection.snapshot(expected.input,expectedFrame);
      assert.deepEqual(await page.evaluate(()=>CenterDemo.inspect().summary),expectedSnapshot.summary);assert.equal(await page.evaluate(()=>S.t),30);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.elapsed),300);
      await page.selectOption('#cdMetric','n');const values=await page.locator('#cdHeat td button').allTextContents();assert.deepEqual(values,expectedFrame.cells.map(c=>c.n.toLocaleString('zh-CN')));assert((await page.locator('#cdSceneOverlay').textContent()).includes('宏观推演 +300秒'));assert((await page.locator('#cdForecastMeta').textContent()).includes('当前方案宏观推演 5分钟'));assert((await page.locator('#cdDetails').textContent()).includes(expectedSnapshot.regions[14].n.toLocaleString('zh-CN')));assert((await page.locator('#cdForecastPlot').getAttribute('aria-label')).includes('当前方案推演'));
    });
    await record('baseline/strategy toggling preserves the same replay endpoint and changes the actual scene data',async()=>{
      await page.locator('#cdProjectionBase').click();assert.deepEqual(await page.evaluate(()=>CenterDemo.inspect().summary),Projection.snapshot(expected.input,expected.baseline.frames.at(-1)).summary);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.elapsed),300);assert.equal(await page.locator('#cdProjectionBase').getAttribute('aria-pressed'),'true');
      await page.locator('#cdProjectionPolicy').click();assert.deepEqual(await page.evaluate(()=>CenterDemo.inspect().summary),Projection.snapshot(expected.input,expected.strategy.frames.at(-1)).summary);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.elapsed),300);
    });
    await record('six time-series metrics use their own units and only actual stored run values',async()=>{
      const units={meanWait:'s/架',queue:'架',highLoadSeconds:'宏观单元·秒',diverted:'架',n:'架',speed:'m/s'};
      for(const [key,unit]of Object.entries(units)){const label=await page.locator('#'+(key==='meanWait'?'cdPolicyPlot':'cdPolicyPlot'+key)).getAttribute('aria-label');assert(label.includes(unit));assert(label.includes('基线'));assert(label.includes('#1'));assert(label.includes('10:30至10:35'));}
      assert.deepEqual(await page.evaluate(()=>CenterProjectionUI.inspect().records[0].frames.map(f=>f.metrics)),expected.strategy.frames.map(f=>f.metrics));
    });
    await record('region/layer selection filters the computed scene without narrowing the full-domain effectiveness comparison',async()=>{
      const label=await page.locator('#cdPolicyPlot').getAttribute('aria-label');await page.locator('#cdHeat [data-region="20"][data-layer="2"]').click();assert.deepEqual(await page.evaluate(()=>CenterDemo.inspect().summary),Projection.snapshot(expected.input,expected.strategy.frames.at(-1),'L3').summary);assert.equal(await page.evaluate(()=>CenterDemo.inspect().regionId),'R21');assert.equal(await page.locator('#cdPolicyPlot').getAttribute('aria-label'),label);assert((await page.locator('#cdForecastMeta').textContent()).includes('R21 / L3'));await page.selectOption('#cdLayer','all');
    });
    await record('projection playback, pause, backtracking and end-of-run replay do not advance the live source clock',async()=>{
      await seek(0);await page.selectOption('#cdRate','60');await page.locator('#cdPlay').click();await page.waitForFunction(()=>CenterDemo.inspect().projection.elapsed>0);await page.locator('#cdPlay').click();const paused=await page.evaluate(()=>CenterDemo.inspect().projection.elapsed);await page.waitForTimeout(150);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.elapsed),paused);assert.equal(await page.evaluate(()=>S.t),30);assert.equal(await page.evaluate(()=>S.playing),false);await seek(300);await page.locator('#cdPlay').click();await page.waitForFunction(()=>CenterDemo.inspect().projection.elapsed<300);await page.locator('#cdPlay').click();await seek(125);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.elapsed),125);
    });
    await record('return to live restores the original clock, source data, timeline and controls',async()=>{
      await page.locator('#cdProjectionExit').click();assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.active),false);assert.deepEqual(await page.evaluate(()=>CenterDemo.inspect().summary),original);assert.equal(await page.evaluate(()=>S.t),30);assert.equal(await page.locator('#cdTime').getAttribute('max'),'60');assert.equal(await page.locator('#cdTime').getAttribute('step'),'1');assert(await page.locator('#cdHorizon').isEnabled());assert.equal(await page.evaluate(()=>CenterProjectionUI.inspect().records.length),1);
    });
    await record('same-input strategies share a group; changed horizon or source yields a separate comparison',async()=>{
      await page.selectOption('#cdPolicy','reserve');await run();assert.equal(await page.evaluate(()=>new Set(CenterProjectionUI.inspect().records.map(r=>r.group)).size),1);assert((await page.locator('#cdPolicyPlot').getAttribute('aria-label')).includes('#2'));
      await page.selectOption('#cdProjectionHorizon','600');assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.active),false);await run();assert.equal(await page.evaluate(()=>new Set(CenterProjectionUI.inspect().records.map(r=>r.group)).size),2);assert(!(await page.locator('#cdPolicyPlot').getAttribute('aria-label')).includes('#1'));assert((await page.locator('#cdPolicyPlot').getAttribute('aria-label')).includes('10:30至10:40'));
      await page.evaluate(()=>{S.t=31;render();});assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.active),false);await run();assert.equal(await page.evaluate(()=>new Set(CenterProjectionUI.inspect().records.map(r=>r.group)).size),3);
    });
    await record('only the newest eight thin metric records are retained and clearing keeps the active scene',async()=>{
      for(let i=0;i<5;i++){await page.selectOption('#cdPolicy',i%2?'balance':'reserve');await run();}assert.equal(await page.evaluate(()=>CenterProjectionUI.inspect().records.length),8);assert(await page.evaluate(()=>CenterProjectionUI.inspect().records.every(r=>r.frames.every(f=>!f.cells))));
      const snapshot=await page.evaluate(()=>CenterDemo.inspect().summary);await page.locator('#cdProjectionClear').click();assert.equal(await page.evaluate(()=>CenterProjectionUI.inspect().records.length),0);assert.deepEqual(await page.evaluate(()=>CenterDemo.inspect().summary),snapshot);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.active),true);assert.equal(await page.locator('#cdPolicyPlot').getAttribute('aria-label'),'尚无推演记录');
    });
    await record('changing scenario cancels an active or pending branch and cannot apply stale worker output',async()=>{
      await page.evaluate(()=>{S.scenario='normal';render();});assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.active),false);
      await page.selectOption('#cdProjectionHorizon','900');await page.evaluate(()=>{document.getElementById('cdProjectionRun').click();S.t=32;render();});await page.waitForTimeout(250);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.busy),false);assert.equal(await page.evaluate(()=>CenterProjectionUI.inspect().records.length),0);assert.equal(await page.evaluate(()=>S.t),32);
    });
    await record('0/60 minute source boundaries and the 15-minute horizon retain fixed source topology',async()=>{
      for(const t of [0,60]){await page.evaluate(t=>{S.t=t;S.scenario='peak';render();},t);await run();report.seekMs.push(await seek(900));assert.equal(await page.evaluate(()=>S.t),t);assert.equal(await page.evaluate(()=>CenterProjectionUI.current().time),t+15);assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.elapsed),900);assert.equal(await page.evaluate(()=>CenterProjectionUI.input().topology.versions.R15),t>=40?'C2':'C1');await page.locator('#cdProjectionExit').click();}
    });
    await record('all active controls and seven cards fit the 16:9 presentation at three sizes',async()=>{
      await page.evaluate(()=>{S.t=30;render();});await run();await seek(600);await page.locator('#cdViewToggle').click();
      for(const [width,height]of [[1920,1080],[1600,900],[1280,720]]){
        await page.setViewportSize({width,height});await settle();
        const layout=await page.evaluate(()=>({ratio:document.getElementById('centerDemo').getBoundingClientRect().width/document.getElementById('centerDemo').getBoundingClientRect().height,overflow:document.documentElement.scrollWidth>innerWidth+1,cards:[...document.querySelectorAll('.cd-dashboard .cd-card')].map(el=>({name:el.querySelector('h3').textContent,clipped:el.scrollHeight>el.clientHeight+1}))}));assert(Math.abs(layout.ratio-16/9)<.001);assert(!layout.overflow);assert.equal(layout.cards.length,7);assert(layout.cards.every(c=>!c.clipped),JSON.stringify(layout));
        for(const selector of ['#cdProjectionHorizon','#cdPolicy','#cdProjectionRun','#cdProjectionBase','#cdProjectionPolicy','#cdProjectionExit','#cdPlay','#cdTime','#cdProjectionGroup','#cdProjectionClear'])assert(await page.locator(selector).evaluate(el=>{const r=el.getBoundingClientRect(),top=document.elementFromPoint(r.x+r.width/2,r.y+r.height/2);return r.width>0&&r.height>0&&(top===el||el.contains(top));}),width+' clipped '+selector);
        report.layouts.push({width,height,...layout});await page.screenshot({path:path.join(out,'center-projection-'+width+'x'+height+'.png')});
      }
    });
    await record('navigation never forwards a projected future clock to the regional page; center return is paused',async()=>{
      const t=await page.evaluate(()=>S.t);await page.locator('#cdPlay').click();await page.locator('#cdSlideEnterRegion').click();assert.equal(await page.evaluate(()=>S.t),t);assert(await page.locator('#regionalDemo').isVisible());assert.equal(await page.locator('#rdViewToggle').getAttribute('aria-pressed'),'true');await page.locator('#rdSlideEnterCenter').click();assert.equal(await page.locator('#cdViewToggle').getAttribute('aria-pressed'),'true');assert.equal(await page.evaluate(()=>CenterDemo.inspect().projection.playing),false);assert.equal(await page.evaluate(()=>S.t),t);await page.keyboard.press('Escape');await page.screenshot({path:path.join(out,'center-projection-normal.png'),fullPage:true});
    });
    await record('worker constructor and load failures fall back to the same deterministic local result',async()=>{
      for(const failure of ['constructor','load']){
        const fallback=await browser.newPage({viewport:{width:1600,height:1000},reducedMotion:'reduce'});fallback.on('pageerror',e=>report.errors.push(e.message));
        if(failure==='constructor')await fallback.addInitScript(()=>{window.Worker=function(){throw Error('Test worker construction failure');};});else await fallback.route('**/center-projection-worker.js',route=>route.abort());
        await fallback.goto(url);await fallback.evaluate(()=>{S.t=30;S.playing=false;render();});await fallback.selectOption('#cdProjectionHorizon','300');await fallback.locator('#cdProjectionRun').click();await fallback.waitForFunction(()=>CenterDemo.inspect().projection.active);assert((await fallback.locator('#cdProjectionSource').textContent()).includes('已在本地完成计算'));assert.deepEqual(await fallback.evaluate(()=>CenterProjectionUI.inspect().result.strategy.frames.at(-1).metrics),expected.strategy.frames.at(-1).metrics);await fallback.close();
      }
    });
    assert.deepEqual(report.errors,[]);report.result='PASS';report.workerRuns=workers;report.browser=browser.version();console.log('PASS '+report.checks.length+' center projection UI checks; '+report.layouts.length+' active layouts; no page errors');
  }finally{fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));if(browser)await browser.close();await new Promise(resolve=>server.close(resolve));}
})().catch(error=>{console.error(error);process.exitCode=1;});
