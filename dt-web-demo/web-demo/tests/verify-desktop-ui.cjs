/* Desktop rendering and interaction contract for both DT views. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const http = require('node:http');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.resolve(__dirname, '..');
const baseline = process.env.DT_CURRENT_ROOT || path.resolve(root, '../../低空专项/web-demo');
const out = process.env.DT_REVIEW_OUTPUT || path.resolve(root, '../output/playwright/impeccable');
const server = http.createServer((req, res) => {
  const url = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
  const old = url.startsWith('/baseline/');
  const base = old ? baseline : root;
  const relative = old ? url.slice('/baseline'.length) : url;
  const file = path.resolve(base, '.' + (relative === '/' ? '/index.html' : relative));
  if (!file.startsWith(base + path.sep)) return res.writeHead(403).end();
  fs.readFile(file, (error, data) => {
    if (error) return res.writeHead(404).end();
    res.setHeader('Content-Type', url.endsWith('.js') ? 'application/javascript' : url.endsWith('.css') ? 'text/css' : 'text/html');
    res.end(data);
  });
});
(async () => {
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  let browser;
  const report = {checks:[], layouts:[], emptyComparisons:[], spacing:[], recordedLayouts:[], errors:[], requests:[]};
  const check = (name, result) => { assert(result, name); report.checks.push(name); console.log('PASS ' + name); };
  try {
    fs.mkdirSync(out, {recursive:true});
    browser = await chromium.launch({headless:true, executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE});
    const page = await browser.newPage({viewport:{width:1920,height:1080}, reducedMotion:'reduce'});
    page.on('pageerror', error => report.errors.push(error.message));
    page.on('request', request => { if (!new URL(request.url()).hostname.match(/^(127\.0\.0\.1|localhost)$/) && !request.url().startsWith('data:')) report.requests.push(request.url()); });
    await page.goto('http://127.0.0.1:' + server.address().port);
    await page.evaluate(() => { S.playing=false; S.t=30; render(); });
    const settle = () => page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
    const state = () => page.evaluate(() => ({view:S.view,region:S.region,layer:S.layer,scenario:S.scenario,t:S.t,playing:S.playing}));
    for (const view of (process.env.DT_REVIEW_VIEW ? [process.env.DT_REVIEW_VIEW] : ['center','region'])) {
      await page.locator('nav [data-view="' + view + '"]').click();
      await settle();
      await page.waitForFunction(view=>view==='center'?CenterHistoryUI.inspect().history?.ready:RegionEvaluation.inspect().history?.ready,view);
      const prefix = view === 'center' ? 'cd' : 'rd';
      const scope = view === 'center' ? '#centerDemo' : '#regionalDemo';
      const clock = view === 'center' ? '.cd-timeline' : '.rd-time';
      const toggle = '#' + prefix + 'ViewToggle';
      for (const [width,height] of [[1920,1080],[2560,1440],[3840,2160]]) {
        await page.setViewportSize({width,height});
        for (const mode of ['desktop','presentation']) {
          if ((await page.locator(toggle).getAttribute('aria-pressed') === 'true') !== (mode === 'presentation')) await page.locator(toggle).click();
          await page.evaluate(() => window.scrollTo(0,0));
          await settle();
          const layout = await page.evaluate(({scope,clock,mode}) => {
            const root=document.querySelector(scope), rect=root.getBoundingClientRect(), timeline=root.querySelector(clock).getBoundingClientRect();
            const panels=[...root.querySelectorAll('.cd-dashboard .cd-card, .rd-layout .panel, .rd-evaluations > .panel, .rd-lower .panel')].filter(el=>el.getBoundingClientRect().height>0);
            return {width:innerWidth,height:innerHeight,documentWidth:document.documentElement.scrollWidth,root:{x:rect.x,y:rect.y,width:rect.width,height:rect.height},timeline:{y:timeline.y,bottom:timeline.bottom,height:timeline.height},scene:(()=>{const r=root.querySelector('canvas').getBoundingClientRect();return {y:r.y,height:r.height,bottom:r.bottom};})(),plots:[...root.querySelectorAll('.cd-mini-chart canvas, .cd-chart canvas, .rd-eval-plot canvas, .rd-history canvas, .rd-mini-chart canvas')].map(el=>{const r=el.getBoundingClientRect();return{id:el.id,height:r.height};}).filter(p=>p.height>0),panels:panels.map(el=>{const r=el.getBoundingClientRect();return{title:el.querySelector('h3,.panel-title')?.textContent,width:r.width,height:r.height,bottom:r.bottom};})};
          }, {scope,clock,mode});
          check(view+' '+mode+' '+width+' no horizontal overflow',layout.documentWidth<=width+1);
          if (mode==='presentation') {
            check(view+' '+width+' 16:9 canvas fits viewport',Math.abs(layout.root.width/layout.root.height-16/9)<0.01&&layout.root.x>=-1&&layout.root.y>=-1&&layout.root.x+layout.root.width<=width+1&&layout.root.y+layout.root.height<=height+1);
            check(view+' '+width+' timeline inside presentation',layout.timeline.y>=0&&layout.timeline.bottom<=height+1&&layout.timeline.height>20);
            check(view+' '+width+' charts have readable plotting height',layout.plots.every(p=>p.height>=95*(layout.root.width/1920)));
            check(view+' '+width+' panels stay inside canvas',layout.panels.every(p=>p.bottom<=height+2&&p.height>40));
            const comparison=await page.locator(scope).evaluate(root=>{
              const selectors=root.id==='centerDemo'?'.cd-history-card,.cd-policy-card':'.rd-history-panel,.rd-effect-panel';
              return {panels:[...root.querySelectorAll(selectors)].map(el=>{const r=el.getBoundingClientRect();return{x:r.x,y:r.y,width:r.width,height:r.height};}),count:[...root.querySelectorAll('.cd-comparison-charts canvas,.rd-comparison-charts canvas')].filter(c=>c.getBoundingClientRect().height>0).length};
            });
            check(view+' '+width+' empty strategy keeps twelve metric slots',comparison.count===12);
            const spacing=await page.locator(scope).evaluate(root=>{
              const panels=[...root.querySelectorAll('.cd-kpi,.cd-dashboard .cd-card,.rd-kpi,.rd-layout .panel,.rd-evaluations>.panel,.rd-lower .panel,.cd-timeline,.rd-time')].filter(el=>el.getBoundingClientRect().height>0);
              const collisions=[],close=[];
              for(let i=0;i<panels.length;i++)for(let j=i+1;j<panels.length;j++){
                const a=panels[i],b=panels[j];if(a.contains(b)||b.contains(a))continue;
                const x=a.getBoundingClientRect(),y=b.getBoundingClientRect(),dx=Math.min(x.right,y.right)-Math.max(x.left,y.left),dy=Math.min(x.bottom,y.bottom)-Math.max(x.top,y.top);
                const names=[a.id||a.className,b.id||b.className],minimum=7.5*root.getBoundingClientRect().width/1920;
                if(dx>1&&dy>1)collisions.push(names);
                else if((dx>1&&-dy<minimum)||(dy>1&&-dx<minimum))close.push(names);
              }
              const copy=root.innerText;
              return {collisions,close,extraCopy:/查看预测口径|查看计算范围与口径|从当前全域快照推演|MOCK-CENTER-01|全部216位置保留|连接定位|高度展开仅影响显示|SELECTED OBJECT|评审边界样例|自动补算完成|页面分批补算/.test(copy)};
            });
            if(spacing.collisions.length||spacing.close.length)console.log(JSON.stringify(spacing));
            check(view+' '+width+' no panel overlap',spacing.collisions.length===0);
            check(view+' '+width+' panels have at least 8 logical pixels separation',spacing.close.length===0);
            check(view+' '+width+' requested explanatory copy removed',!spacing.extraCopy);
            if(view==='region'){
              const labels=await page.evaluate(()=>RegionDemo.inspect().labels);
              check('region '+width+' scene labels do not overlap',labels.length>0&&labels.every((a,i)=>labels.slice(i+1).every(b=>a.x+a.width<=b.x||b.x+b.width<=a.x||a.y+a.height<=b.y||b.y+b.height<=a.y)));
              check('region '+width+' selected object label retained',labels.some(l=>l.text.includes('L1-P01')));
            }

            report.spacing.push({view,width,...spacing});

            report.emptyComparisons.push({view,width,panels:comparison.panels});

          } else {
            check(view+' '+width+' clock available without scrolling',layout.timeline.y>=0&&layout.timeline.bottom<=height+1&&layout.timeline.height<100);
            check(view+' '+width+' primary scene visible on arrival',Math.max(0,Math.min(height,layout.scene.bottom)-layout.scene.y)/layout.scene.height>=0.55);
          }
          report.layouts.push({view,mode,...layout});
          await page.screenshot({path:path.join(out,view+'-'+mode+'-'+width+'.png')});
        }
      }
      const before=await state();
      const zoom = page.locator(scope+' .dt-chart-zoom:visible').first();
      await zoom.click();
      check(view+' chart focus opens',await page.locator('#dtChartDialog').isVisible());
      const expanded = page.locator('#dtChartDialog canvas');
      check(view+' chart reuses live canvas',await expanded.count()===1&&(await expanded.boundingBox()).height>300);
      await page.keyboard.press('Escape');
      await settle();
      check(view+' Escape closes focus without leaving presentation',!(await page.locator('#dtChartDialog').isVisible())&&await page.locator(toggle).getAttribute('aria-pressed')==='true');
      check(view+' focus preserves selection and clock',JSON.stringify(await state())===JSON.stringify(before));
      await page.keyboard.press('Escape');
      check(view+' Escape returns to desktop',await page.locator(toggle).getAttribute('aria-pressed')==='false');
      check(view+' nav announces current page',await page.locator('nav [data-view="'+view+'"]').getAttribute('aria-current')==='page');
    }
    await page.setViewportSize({width:1920,height:1080});
    await page.locator('nav [data-view="center"]').click();
    await page.selectOption('#cdProjectionHorizon','300');
    await page.locator('#cdProjectionRun').click();
    await page.waitForFunction(()=>CenterDemo.inspect().projection.active&&!CenterDemo.inspect().projection.busy);
    check('center manual projection generates one real record',await page.evaluate(()=>CenterProjectionUI.inspect().records.length)===1);
    await page.locator('#cdProjectionExit').click();
    check('center returns to source clock',await page.evaluate(()=>S.t===30&&!CenterDemo.inspect().projection.active));
    await page.locator('nav [data-view="region"]').click();
    await page.selectOption('#rdProjectionHorizon','300');
    await page.locator('#rdProjectionRun').click();
    await page.waitForFunction(()=>RegionProjection.active());
    check('region manual projection uses real branch result',await page.evaluate(()=>RegionProjection.inspect().result.strategy.frames.length>1));
    await page.locator('#rdProjectionExit').click();
    check('region returns to source state',await page.evaluate(()=>!RegionProjection.active()&&S.t===30));
    for (const view of ['center','region']) {
      await page.locator('nav [data-view="'+view+'"]').click();
      const prefix=view==='center'?'cd':'rd',scope=view==='center'?'#centerDemo':'#regionalDemo';
      await page.waitForFunction(view=>view==='center'?CenterHistoryUI.inspect().history?.ready:RegionEvaluation.inspect().history?.ready,view);
      for(const policy of (view==='center'?['baseline','reserve']:['baseline','layer'])){
        await page.selectOption(view==='center'?'#cdPolicy':'#rdProjectionStrategy',policy);await page.locator('#'+prefix+'ProjectionRun').click();
        await page.waitForFunction(view=>view==='center'?CenterDemo.inspect().projection.active&&!CenterDemo.inspect().projection.busy:RegionProjection.active(),view);
        await page.locator('#'+prefix+'ProjectionExit').click();
      }
      check(view+' comparison retains three real strategy records',await page.evaluate(view=>view==='center'?CenterProjectionUI.inspect().records.length:RegionEvaluation.inspect().records.length,view)===3);
      await page.locator('#'+prefix+'ViewToggle').click();
      for(const [width,height] of [[1920,1080],[2560,1440],[3840,2160]]){
        await page.setViewportSize({width,height});await settle();
        const result=await page.locator(scope).evaluate(root=>{
          const selectors='.cd-dashboard .cd-card,.cd-comparison-charts,.rd-comparison-charts,.rd-evaluations>.panel,.rd-scene-panel';
          const clipped=[...root.querySelectorAll(selectors)].filter(el=>el.scrollHeight>el.clientHeight+2).map(el=>el.className);
          const canvases=[...root.querySelectorAll('.cd-comparison-charts canvas,.rd-comparison-charts canvas')].filter(c=>c.getBoundingClientRect().height>0);
          return {clipped,count:canvases.length,heights:canvases.map(c=>c.getBoundingClientRect().height)};
        });
        check(view+' '+width+' recorded strategy twelve graphs fit without clipping',result.clipped.length===0&&result.count===12&&result.heights.every(h=>h>=95*width/1920));
        const panels=await page.locator(scope).evaluate(root=>[...root.querySelectorAll(root.id==='centerDemo'?'.cd-history-card,.cd-policy-card':'.rd-history-panel,.rd-effect-panel')].map(el=>{const r=el.getBoundingClientRect();return{x:r.x,y:r.y,width:r.width,height:r.height};}));
        const empty=report.emptyComparisons.find(item=>item.view===view&&item.width===width)?.panels;
        check(view+' '+width+' comparison panels keep empty-state geometry',empty&&panels.every((r,i)=>Object.keys(r).every(k=>Math.abs(r[k]-empty[i][k])<1)));
        const gap=panels[1].x-panels[0].x-panels[0].width;
        check(view+' '+width+' comparison panels have at least 10 logical pixels gap',gap>=10*width/1920-1);

        report.recordedLayouts.push({view,width,...result,panels});
        if(view==='region')check('region '+width+' details remain scrollable',await page.locator('#rdDetail').evaluate(el=>getComputedStyle(el).overflowY==='auto'&&el.clientHeight>150));
        await page.screenshot({path:path.join(out,view+'-recorded-presentation-'+width+'.png')});
      }
      await page.keyboard.press('Escape');
    }
    const old = await browser.newPage({viewport:{width:1920,height:1080},reducedMotion:'reduce'});
    await old.goto('http://127.0.0.1:' + server.address().port + '/baseline/');
    for (const view of (process.env.DT_REVIEW_VIEW ? [process.env.DT_REVIEW_VIEW] : ['center','region'])) {
      const freeze = view => { S.view=view;S.region=14;S.layer='all';S.scenario='peak';S.t=30;S.playing=false;render(); };
      await page.evaluate(freeze,view); await old.evaluate(freeze,view);
      const data = () => S.view==='center' ? CenterDemo.inspect().summary : {scope:RegionDemo.inspect().scope,vehicles:RegionDemo.inspect().vehicles.map(v=>[v.id,v.objectId,v.speed])};
      check(view+' original/new data are identical',JSON.stringify(await page.evaluate(data))===JSON.stringify(await old.evaluate(data)));
    }
    await old.close();
    check('no page errors',report.errors.length===0);
    check('no external network requests',report.requests.length===0);
    report.browser=browser.version(); report.result='PASS';
    console.log('PASS '+report.checks.length+' checks, '+report.layouts.length+' base layouts, '+report.recordedLayouts.length+' recorded layouts');
  } finally {
    fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));
    await browser?.close();
    await new Promise(resolve=>server.close(resolve));
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
