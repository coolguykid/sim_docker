/* Incremental center history: four independent runs, metrics only. */
importScripts('center-model.js','center-projection.js');
let engine,target=0,to=-5,timer=null;
function pump(){
  timer=null;const started=performance.now();let batch;
  do{
    const next=engine.advance(Math.min(target,to+5));to=next.to;
    if(!batch)batch=next;else{batch.to=next.to;next.runs.forEach((r,i)=>batch.runs[i].timeline.push(...r.timeline));}
  }while(to<target&&performance.now()-started<8);
  postMessage(batch);if(to<target)timer=setTimeout(pump,0);
}
onmessage=e=>{try{
  if(e.data.type==='init'){engine=CenterProjection.createHistory(e.data.scenario,e.data.quality);to=-5;}
  target=Math.floor(e.data.to/5)*5;if(timer===null&&to<target)timer=setTimeout(pump,0);
}catch(error){postMessage({error:error.message});}};
