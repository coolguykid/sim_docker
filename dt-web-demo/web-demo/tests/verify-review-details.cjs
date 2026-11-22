/* Targeted 16:9 review details, including Canvas text and real pending history. */
'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),http=require('node:http');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=path.resolve(__dirname,'..'),out=path.resolve(root,'../output/playwright/review-details-20261104');
const server=http.createServer((req,res)=>{const url=decodeURIComponent(new URL(req.url,'http://localhost').pathname),file=path.resolve(root,'.'+(url==='/'?'/index.html':url));if(!file.startsWith(root+path.sep))return res.writeHead(403).end();fs.readFile(file,(error,data)=>{if(error)return res.writeHead(404).end();res.setHeader('Content-Type',url.endsWith('.js')?'application/javascript':url.endsWith('.css')?'text/css':'text/html');res.end(data);});});
(async()=>{
 await new Promise((resolve,reject)=>{server.once('error',reject);server.listen(Number(process.env.DT_REVIEW_PORT)||18773,'127.0.0.1',resolve);});let browser;
 const report={checks:[],errors:[],heat:null};const check=(name,value)=>{assert(value,name);report.checks.push(name);console.log('PASS '+name);};
 try{
  fs.mkdirSync(out,{recursive:true});browser=await chromium.launch({headless:true,executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE});
  const page=await browser.newPage({viewport:{width:2560,height:1440},reducedMotion:'reduce'});page.on('pageerror',error=>report.errors.push(error.message));
  await page.goto('http://127.0.0.1:'+server.address().port);await page.evaluate(()=>{S.playing=false;S.t=30;render();CenterDemo.presentation(true);});
  await page.waitForFunction(()=>CenterHistoryUI.inspect().history?.ready);
  report.heat=await page.locator('#cdHeat').evaluate(table=>{const cols=[...table.querySelectorAll('thead th')].map(th=>th.getBoundingClientRect()),row=table.querySelector('tbody tr'),cell=row.firstElementChild.getBoundingClientRect(),button=row.querySelector('button').getBoundingClientRect(),box=table.getBoundingClientRect(),parent=table.parentElement.getBoundingClientRect();return{regionWidth:cell.width,regionFill:button.width/cell.width,layers:cols.slice(1).map(r=>r.width),rightGap:parent.right-cols.at(-1).right,rows:table.querySelectorAll('tbody tr').length,cells:table.querySelectorAll('td button').length,tableWidth:box.width,parentWidth:parent.width};});
  check('region selector fills its heatmap column',report.heat.regionFill>.95);
  check('heatmap gives six layers equal width and fills right edge',Math.max(...report.heat.layers)-Math.min(...report.heat.layers)<1&&report.heat.rightGap<10&&Math.abs(report.heat.tableWidth-report.heat.parentWidth)<1);
  check('all 36 regions and 216 cells remain',report.heat.rows===36&&report.heat.cells===216);
  const forecastText=()=>page.evaluate(()=>{const texts=[],original=CanvasRenderingContext2D.prototype.fillText;CanvasRenderingContext2D.prototype.fillText=function(value,...args){if(this.canvas.id==='cdForecastPlot')texts.push(String(value));return original.call(this,value,...args);};try{CenterDemo.resize();}finally{CanvasRenderingContext2D.prototype.fillText=original;}return texts;});
  check('presentation forecast removes the three in-chart labels',!(await forecastText()).some(t=>['模拟历史','模拟预测','起报'].includes(t)));
  check('forecast accessible description keeps history and prediction semantics',(await page.locator('#cdForecastPlot').getAttribute('aria-label')).includes('模拟历史'));
  await page.screenshot({path:path.join(out,'center-2560.png')});
  await page.locator('#cdSlideEnterRegion').click();
  check('regional destination remains in 16:9',await page.locator('#rdViewToggle').getAttribute('aria-pressed')==='true');
  await page.evaluate(()=>{S.t=60;S.playing=false;render();});
  check('test reaches actual pending history state',await page.locator('#rdHistoryMessage').getAttribute('data-state')==='pending');
  check('pending computation text is hidden in regional presentation',!await page.locator('#rdHistoryMessage').isVisible());
  await page.screenshot({path:path.join(out,'region-pending-2560.png')});
  await page.waitForFunction(()=>RegionEvaluation.inspect().history?.ready);
  await page.evaluate(()=>{S.t=30;render();});await page.waitForFunction(()=>RegionEvaluation.inspect().history?.ready);
  const legend=await page.locator('.rd-limits-legend').evaluate(el=>[...el.children].map(item=>{const style=getComputedStyle(item.querySelector('i'));return{text:item.textContent.trim(),color:style.borderTopColor,line:style.borderTopStyle};}));
  report.legend=legend;
  check('constraint legend lists all four exact line meanings',JSON.stringify(legend.map(i=>i.text))===JSON.stringify(['安全距离下限 · m','运行最小纵向间距 · m','速度上限 · m/s','运行最高速度 · m/s']));
  check('constraint legend matches chart gold/blue and dashed/solid styles',JSON.stringify(legend.map(i=>[i.color,i.line]))===JSON.stringify([['rgb(172, 123, 44)','dashed'],['rgb(172, 123, 44)','solid'],['rgb(49, 110, 145)','dashed'],['rgb(49, 110, 145)','solid']]));
  for(const minute of [20,40]){
   await page.evaluate(minute=>{S.t=minute;render();},minute);
   const strokes=await page.evaluate(()=>{
    const proto=CanvasRenderingContext2D.prototype,original={beginPath:proto.beginPath,moveTo:proto.moveTo,lineTo:proto.lineTo,stroke:proto.stroke},lines=[];let points=[];
    proto.beginPath=function(...args){if(this.canvas.id==='rdLimits')points=[];return original.beginPath.apply(this,args);};
    for(const method of ['moveTo','lineTo'])proto[method]=function(x,y){if(this.canvas.id==='rdLimits')points.push({x,y});return original[method].call(this,x,y);};
    proto.stroke=function(...args){if(this.canvas.id==='rdLimits'&&this.getLineDash().length&&['#ac7b2c','#316e91'].includes(this.strokeStyle))lines.push({color:this.strokeStyle,points:points.slice()});return original.stroke.apply(this,args);};
    try{RegionDemo.resize();}finally{Object.assign(proto,original);}
    return lines;
   });
   const distance=strokes.find(s=>s.color==='#ac7b2c'),speed=strokes.find(s=>s.color==='#316e91');
   check('both constraint dashed lines stay separately visible at minute '+minute,distance?.points.length>1&&speed?.points.length>1&&Math.abs(distance.points.at(-1).y-speed.points.at(-1).y)>3);
   await page.screenshot({path:path.join(out,'region-constraints-'+minute+'-2560.png')});
  }
  await page.evaluate(()=>{S.t=30;render();});await page.waitForFunction(()=>RegionEvaluation.inspect().history?.ready);
  const fit=await page.locator('.rd-limits-legend').evaluate(el=>{const panel=el.closest('.panel').getBoundingClientRect(),timeline=document.querySelector('.rd-time').getBoundingClientRect(),children=[...el.querySelectorAll('span'),document.querySelector('#rdLimitsNote')].map(i=>{const r=i.getBoundingClientRect();return{x:r.x,y:r.y,right:r.right,bottom:r.bottom};});return{children,panel:{x:panel.x,y:panel.y,right:panel.right,bottom:panel.bottom},timelineTop:timeline.top};});
  check('legend and current values fit their panel above timeline',fit.children.every(r=>r.x>=fit.panel.x&&r.right<=fit.panel.right&&r.bottom<=fit.panel.bottom)&&fit.panel.bottom<fit.timelineTop);
  await page.screenshot({path:path.join(out,'region-2560.png')});
  await page.locator('#rdSlideEnterCenter').click();await page.keyboard.press('Escape');
  check('desktop forecast retains explanatory labels',(await forecastText()).includes('模拟历史'));
  check('no browser errors',report.errors.length===0);report.result='PASS';console.log('PASS '+report.checks.length+' review detail checks at 2560x1440');
 }finally{fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));await browser?.close();await new Promise(resolve=>server.close(resolve));}
})().catch(error=>{console.error(error);process.exitCode=1;});
