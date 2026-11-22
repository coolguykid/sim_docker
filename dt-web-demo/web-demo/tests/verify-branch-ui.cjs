const fs=require('node:fs'),path=require('node:path'),http=require('node:http');
async function verify(page,base){
  const checks=[],errors=[],check=(name,ok)=>{checks.push({name,passed:!!ok});if(!ok)throw Error(name);};
  page.on('pageerror',e=>errors.push(String(e)));
  await page.setViewportSize({width:1600,height:1100});await page.goto(base);await page.locator('[data-view=region]').click();await page.locator('#scenario').selectOption('peak');
  await page.evaluate(()=>{S.playing=false;RegionDemo.render(S);});
  const seek=async t=>page.locator('#rdTimeSlider').evaluate((el,t)=>{el.value=t;el.dispatchEvent(new Event('input',{bubbles:true}));el.dispatchEvent(new Event('change'));},t);
  await seek(1200);
  check('独立策略面板及两条示意线画布已删除',await page.locator('#rdBranch,#rdBranchBase,#rdBranchPolicy').count()===0);
  check('推演入口位于六层总览内部',await page.evaluate(()=>document.querySelector('#rdProjectionRun').closest('.rd-scene-panel').contains(document.querySelector('#rdScene'))));
  const before=await page.evaluate(()=>{window.__originalScene=document.querySelector('#rdScene');return JSON.stringify(RegionDemo.inspect().vehicles);});
  await page.locator('#rdProjectionRun').click();
  const initial=await page.evaluate(()=>{const {result:r}=RegionProjection.inspect();return r&&{time:r.input.time,pipes:Object.values(r.input.facilities).filter(o=>o.kind==='pipe').length,guides:Object.values(r.input.facilities).filter(o=>['in','out','ramp'].includes(o.kind)).length,groups:Object.keys(r.input.groups).length,same:JSON.stringify(r.baseline.frames[0])===JSON.stringify(r.strategy.frames[0])};});
  check('捕获六层60管道44引路和12个同向组',initial?.time===1200&&initial.pipes===60&&initial.guides===44&&initial.groups===12);
  check('两方案从同一完整初始帧开始',initial.same);
  check('总览推演初始个体位置与源状态完全一致',before===await page.evaluate(()=>JSON.stringify(RegionDemo.inspect().vehicles)));
  check('原场景画布原位复用且已标记策略推演',await page.evaluate(()=>__originalScene===document.querySelector('#rdScene'))&&(await page.locator('#rdCameraTitle').innerText()).includes('策略推演'));
  await page.locator('#rdProjectionTime').evaluate(el=>{el.value=60;el.dispatchEvent(new Event('input',{bubbles:true}));});
  const audit=await page.evaluate(()=>{const {result:r}=RegionProjection.inspect();let errors=0,caps=0,speeds=0,unique=0;for(const b of [r.baseline,r.strategy])for(const f of b.frames){if(f.positions.length+f.completed!==f.arrived)errors++;if(new Set(f.positions.map(v=>v.id)).size!==f.positions.length)unique++;for(const [id,a] of Object.entries(f.states)){const vs=f.positions.filter(v=>v.objectId===id&&!v.waiting),cap=r.input.facilities[id].capacity;if(!a.missing&&a.n!==vs.length)errors++;if(cap&&vs.length>Math.max(cap,r.baseline.frames[0].states[id].visible))caps++;if(!a.missing&&vs.length&&Math.abs(a.speed-vs.reduce((n,v)=>n+v.speed,0)/vs.length)>1e-8)speeds++;}}return {errors,caps,speeds,unique,diverted:r.strategy.metrics.diverted,arrivals:r.baseline.metrics.arrived===r.strategy.metrics.arrived,snapshot:RegionDemo.inspect().t,waiting:r.strategy.frames[60].queue};});
  check('整个区域逐帧任务守恒且无重复个体',audit.errors===0&&audit.unique===0);
  check('所有设施容量准入及均速一致',audit.caps===0&&audit.speeds===0);
  check('全区域后续需求一致且真实执行分流',audit.arrivals&&audit.diverted>0);
  check('总览时间和无人机读取共同推演帧',audit.snapshot===1500&&(await page.locator('#rdProjectionClock').innerText()).includes('+300秒'));
  check('区域策略有效性展示六项指标',await page.locator('.rd-effect-panel .rd-error-chart').count()===6);
  check('入口等待个体参与区域绘制',audit.waiting>0&&await page.evaluate(()=>RegionDemo.inspect().vehicles.some(v=>v.waiting)));
  const policy=await page.evaluate(()=>JSON.stringify(RegionDemo.inspect().states));await page.locator('#rdProjectionBase').click();
  check('同一画布同一时刻切换基线产生不同设施状态',await page.evaluate(()=>RegionDemo.inspect().t)===1500&&policy!==await page.evaluate(()=>JSON.stringify(RegionDemo.inspect().states))&&(await page.locator('#rdCameraTitle').innerText()).includes('基线推演'));
  check('原始快照未被推演修改',before===await page.evaluate(()=>JSON.stringify(RegionDemo.inspect(1200).vehicles)));
  check('推演期间隐藏旧历史曲线，共用底部时间轴只显示推演控制',!(await page.locator('.rd-lower').isVisible())&&(await page.locator('.rd-time').isVisible())&&(await page.locator('.rd-time #rdProjectionPlayback').isVisible())&&!(await page.locator('.rd-time #rdTimeSlider').isVisible()));
  await page.locator('#rdProjectionPolicy').click();await page.locator('.rd-scene-panel').screenshot({path:'output/playwright/region-performance-20261031/manual-projection.png'});
  await page.locator('#rdProjectionPlay').click();await page.waitForTimeout(360);await page.locator('#rdProjectionPlay').click();check('六层推演可播放暂停',await page.locator('#rdProjectionPlay').innerText()==='播放推演');
  await page.locator('#rdProjectionExit').click();check('退出恢复原时刻及完整态势',before===await page.evaluate(()=>JSON.stringify(RegionDemo.inspect().vehicles))&&await page.locator('.rd-lower').isVisible());
  await page.locator('#scenario').selectOption('reserve');await seek(1200);await page.locator('#rdProjectionRun').click();
  check('全备用已开放时两方案结果相同',await page.evaluate(()=>{const {result:r}=RegionProjection.inspect();return JSON.stringify(r.baseline.frames)===JSON.stringify(r.strategy.frames)&&JSON.stringify(r.baseline.metrics)===JSON.stringify(r.strategy.metrics);}));
  await page.locator('#scenario').selectOption('peak');check('切换场景自动退出旧推演',await page.evaluate(()=>!RegionProjection.active())&&!(await page.locator('#rdProjectionCompare').isVisible()));
  await seek(3600);await page.locator('#rdProjectionRun').click();check('区域末帧之后只排空无新增任务',await page.evaluate(()=>RegionProjection.inspect().result.input.jobs.every(j=>j.initial)));
  await page.locator('#rdProjectionExit').click();await seek(0);
  for(const horizon of ['300','900']){await page.locator('#rdProjectionHorizon').selectOption(horizon);await page.locator('#rdProjectionRun').click();check('全区域起始时刻推演窗口 '+horizon,await page.evaluate(h=>{const {result:r}=RegionProjection.inspect();return r.input.horizon===+h&&r.strategy.frames.at(-1).elapsed===+h;},horizon));await page.locator('#rdProjectionExit').click();}
  check('区域推演无JavaScript异常',errors.length===0);
  await page.evaluate(r=>globalThis.__branchReport=r,{checks,errors});
  return {checks,errors};
}
if(require.main===module){
  const root=path.resolve(__dirname,'..'),{chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
  const server=http.createServer((req,res)=>{const name=decodeURIComponent(new URL(req.url,'http://localhost').pathname),file=path.resolve(root,'.'+name);if(file!==root&&!file.startsWith(root+path.sep))return res.writeHead(403).end();fs.readFile(name==='/'?path.join(root,'index.html'):file,(e,b)=>{if(e)return res.writeHead(404).end();res.setHeader('Content-Type',name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':'text/html');res.end(b);});});
  (async()=>{await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;try{fs.mkdirSync('output/playwright/region-performance-20261031',{recursive:true});browser=await chromium.launch({headless:true,executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE});const report=await verify(await browser.newPage(),'http://127.0.0.1:'+server.address().port);fs.writeFileSync('output/playwright/region-performance-20261031/manual-report.json',JSON.stringify(report,null,2));console.log('PASS: '+report.checks.length+' manual regional projection checks, full regional conservation/capacity/speed, playback, restore, scene change and 0/3600 boundaries');}finally{await browser?.close();server.close();}})().catch(e=>{console.error(e);process.exitCode=1;});
}
module.exports=verify;
