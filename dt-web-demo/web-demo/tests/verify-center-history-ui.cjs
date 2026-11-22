const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),http=require('node:http');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=path.resolve(__dirname,'..'),out=path.resolve(root,'../output/center-rmse-20261031');
const server=http.createServer((req,res)=>{const name=decodeURIComponent(new URL(req.url,'http://localhost').pathname),file=path.resolve(root,'.'+(name==='/'?'/index.html':name));if(!file.startsWith(root+path.sep))return res.writeHead(403).end();fs.readFile(file,(e,b)=>{if(e)return res.writeHead(404).end();res.setHeader('Content-Type',name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':'text/html');res.end(b);});});
(async()=>{
  await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;const report={checks:[],errors:[]};
  try{
    fs.mkdirSync(out,{recursive:true});browser=await chromium.launch({headless:true,executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE});
    const page=await browser.newPage({viewport:{width:1920,height:1080},reducedMotion:'reduce'});page.on('pageerror',e=>report.errors.push(e.message));
    const base='http://127.0.0.1:'+server.address().port,settle=()=>page.waitForFunction(()=>CenterHistoryUI.inspect().history?.ready);
    const check=async(name,fn)=>{await fn();report.checks.push(name);console.log('PASS '+name);};
    const plots=()=>page.evaluate(()=>{const result=[],original=CenterDemo.drawComparison;CenterDemo.drawComparison=(id,series,options)=>{result.push({id,series,options});return original(id,series,options);};try{CenterDemo.resize();return result;}finally{CenterDemo.drawComparison=original;}});
    const assertRmse=async()=>{
      const captured=(await plots()).filter(p=>p.id.startsWith('cdHistoryPlot')),h=await page.evaluate(()=>CenterHistoryUI.inspect().history),entries=await page.evaluate(()=>Object.entries(CenterProjectionUI.metrics));
      assert.equal(captured.length,6);for(let k=0;k<6;k++){
        const [key,item]=entries[k],p=captured[k],errors=[];const expected=h.timeline.map(f=>{const a=f[key],b=h.dt.find(q=>q.t===f.t)?.[key];if(!Number.isFinite(a)||!Number.isFinite(b))return {x:f.t/60,y:null};errors.push((b-a)**2);return {x:f.t/60,y:Math.sqrt(errors.reduce((s,v)=>s+v,0)/errors.length)};});
        assert.equal(p.series.length,1);assert.deepEqual(p.series[0].points,expected);assert.equal(p.options.unit,'RMSE · '+item.unit);assert.equal(p.options.xLabel,'时间');assert(p.options.axes);assert.equal(p.options.xMin,h.from/60);assert.equal(p.options.xMax,h.to/60);assert(p.series[0].points.every(d=>d.x<=h.to/60));
      }return captured;
    };
    await page.goto(base);await page.evaluate(()=>{S.playing=false;S.t=20;render();});await settle();
    await check('automatic historical worker, six RMSE graphs and six initially empty effectiveness graphs',async()=>{
      assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().runtime.mode),'worker');assert.equal(await page.locator('.cd-history-card canvas').count(),6);assert.equal(await page.locator('.cd-policy-card canvas').count(),6);
      assert.equal(await page.locator('#cdMatrix,#cdLayerTrend,#cdLayerCurves,#cdPolicyMetric').count(),0);assert.equal(await page.evaluate(()=>CenterProjectionUI.inspect().records.length),0);
      assert((await page.locator('.cd-history-card').textContent()).includes('模拟替代'));assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().history.to),1200);await assertRmse();
    });
    await check('three policies and all windows recalculate paired RMSE without changing observed values',async()=>{
      const initial=await page.evaluate(()=>CenterHistoryUI.inspect().history.timeline);
      for(const policy of ['baseline','reserve','balance']){
        await page.selectOption('#cdHistoryPolicy',policy);const p=await assertRmse();assert.deepEqual(await page.evaluate(()=>CenterHistoryUI.inspect().history.timeline),initial);
        if(policy==='baseline')assert(p.every(v=>v.series[0].points.every(d=>d.y===0||d.y===null)));else assert(p.some(v=>v.series[0].points.some(d=>d.y>0)));
      }
      for(const seconds of [300,900,600]){await page.selectOption('#cdHistoryWindow',String(seconds));await assertRmse();assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().history.from),1200-seconds);}
      assert.equal(await page.evaluate(()=>CenterProjectionUI.inspect().records.length),0);
    });
    await check('cached rewind and forward stop exactly at the historical time and restore deterministic values',async()=>{
      const before=await page.evaluate(()=>CenterHistoryUI.inspect().history);
      await page.evaluate(()=>{S.t=30;render();});await settle();const computed=await page.evaluate(()=>CenterHistoryUI.inspect().runtime.to);assert.equal(computed,1800);
      await page.evaluate(()=>{S.t=20;render();});assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().runtime.to),1800);assert.deepEqual(await page.evaluate(()=>CenterHistoryUI.inspect().history),before);await assertRmse();
      await page.evaluate(()=>{S.t=1203/60;render();});assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().history.to),1200,'macro historical output uses complete five-second steps only');
      await page.evaluate(()=>{S.t=20;render();});
    });
    await check('full-domain history is independent of region/layer selection and manual future replay',async()=>{
      const before=await page.evaluate(()=>CenterHistoryUI.inspect().history);await page.locator('#cdHeat [data-region="20"][data-layer="2"]').click();assert.deepEqual(await page.evaluate(()=>CenterHistoryUI.inspect().history),before);
      await page.selectOption('#cdProjectionHorizon','300');await page.locator('#cdProjectionRun').click();await page.waitForFunction(()=>CenterProjectionUI.active());
      await page.locator('#cdTime').evaluate(el=>{el.value=300;el.dispatchEvent(new Event('input',{bubbles:true}));});assert.equal(await page.evaluate(()=>S.t),20);assert.deepEqual(await page.evaluate(()=>CenterHistoryUI.inspect().history),before);
      const p=(await plots()).filter(p=>p.id.startsWith('cdPolicyPlot'));assert.equal(p.length,6);assert(p.every(v=>v.series.length===2&&v.options.axes&&v.options.xLabel==='时间'));
      const result=await page.evaluate(()=>CenterProjectionUI.inspect().result),keys=await page.evaluate(()=>Object.keys(CenterProjectionUI.metrics));
      for(let k=0;k<6;k++){assert.deepEqual(p[k].series[0].points,result.baseline.frames.map(f=>({x:f.time,y:f.metrics[keys[k]]})));assert.deepEqual(p[k].series[1].points,result.strategy.frames.map(f=>({x:f.time,y:f.metrics[keys[k]]})));}
      await page.locator('#cdProjectionExit').click();
    });
    await check('forecast origin is geometrically centered for every horizon, replay and simulation start',async()=>{
      for(const t of [0,30])for(const horizon of ['5','10','15','20','30']){
        await page.evaluate(t=>{S.t=t;render();},t);await page.selectOption('#cdHorizon',horizon);
        const axis=await page.evaluate(()=>{const proto=CanvasRenderingContext2D.prototype,move=proto.moveTo,stroke=proto.stroke;let start,paths=[];proto.moveTo=function(x,y){if(this.canvas.id==='cdForecastPlot')start={x,y};return move.call(this,x,y);};proto.stroke=function(...args){if(this.canvas.id==='cdForecastPlot'&&this.getLineDash().join(',')==='3,3')paths.push({...start,width:this.canvas.getBoundingClientRect().width});return stroke.apply(this,args);};try{CenterDemo.resize();return paths;}finally{proto.moveTo=move;proto.stroke=stroke;}});
        assert(axis.length>0);assert(axis.every(v=>Math.abs(v.x-v.width/2)<.01));
      }
      await settle();const h=await page.evaluate(()=>CenterHistoryUI.inspect().history);assert.equal(h.to,1800);
      await page.evaluate(()=>{S.t=0;render();});await settle();const start=await assertRmse();assert(start.every(p=>p.series[0].points.length===1&&p.series[0].points[0].x===0));assert((await page.locator('#cdHistoryMessage').textContent()).includes('仅有初始样本'));
    });
    await check('rapid scenario changes isolate stale results and keep bounded historical caches',async()=>{
      await page.evaluate(()=>{S.t=1;S.scenario='reserve';render();S.scenario='normal';render();S.scenario='peak';render();});await settle();
      assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().history.scenario),'peak');assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().runtime.cacheSize),2);await assertRmse();
      await page.evaluate(()=>{S.t=60;render();});await settle();assert.equal(await page.evaluate(()=>CenterHistoryUI.inspect().history.timeline.at(-1).t),3600);await assertRmse();
    });
    await check('all twelve Cartesian graphs fit 16:9 at three sizes with readable captions and controls',async()=>{
      await page.evaluate(()=>{S.t=20;render();});await settle();await page.locator('#cdViewToggle').click();
      for(const [width,height]of [[1920,1080],[2560,1440],[3840,2160]]){
        await page.setViewportSize({width,height});await page.waitForTimeout(80);const layout=await page.evaluate(()=>{const root=document.getElementById('centerDemo'),box=root.getBoundingClientRect();return {ratio:box.width/box.height,clipped:[...document.querySelectorAll('.cd-dashboard .cd-card')].filter(el=>el.scrollHeight>el.clientHeight+1).map(el=>el.className),graphs:[...document.querySelectorAll('.cd-comparison-charts canvas')].map(el=>({width:el.getBoundingClientRect().width,height:el.getBoundingClientRect().height,label:el.getAttribute('aria-label')}))};});
        assert(Math.abs(layout.ratio-16/9)<.001);assert.deepEqual(layout.clipped,[]);assert.equal(layout.graphs.length,12);assert(layout.graphs.every(g=>g.width>100&&g.height>80));await page.screenshot({path:path.join(out,'center-rmse-'+width+'x'+height+'.png')});
      }await page.keyboard.press('Escape');
    });
    await check('worker constructor and runtime failures fall back to incremental local history',async()=>{
      for(const kind of ['constructor','runtime']){
        const fallback=await browser.newPage();fallback.on('pageerror',e=>report.errors.push(e.message));
        if(kind==='constructor')await fallback.addInitScript(()=>{window.Worker=function(){throw Error('forced history worker construction failure');};});else await fallback.route('**/center-history-worker.js',route=>route.fulfill({contentType:'application/javascript',body:'throw Error("forced history worker runtime failure");'}));
        await fallback.goto(base);await fallback.evaluate(()=>{S.t=1;S.playing=false;render();});await fallback.waitForFunction(()=>CenterHistoryUI.inspect().history?.ready);
        const state=await fallback.evaluate(()=>CenterHistoryUI.inspect());assert.equal(state.runtime.mode,'cooperative');assert.equal(state.history.to,60);assert.equal(state.history.timeline.length,13);assert(state.runtime.error.includes('forced'));await fallback.close();
      }
    });
    assert.deepEqual(report.errors,[]);report.result='PASS';console.log('PASS '+report.checks.length+' center historical RMSE and fixed-origin checks; zero page errors');
  }finally{if(browser)await browser.close();server.close();fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));}
})().catch(e=>{console.error(e);process.exitCode=1;});
