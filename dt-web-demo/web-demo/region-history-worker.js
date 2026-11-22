/* One worker owns one immutable region/scenario input. Messages contain incremental metric batches. */
'use strict';
importScripts('branch-simulation.js','region-history.js');
let engine=null,target=0,current=0,timer=null;
function schedule(){if(timer===null)timer=setTimeout(pump,0);}
function pump(){timer=null;if(!engine||current>=target)return;const end=Math.min(target,current+30),start=performance.now(),batch=engine.advance(end);current=end;postMessage({...batch,computeMs:performance.now()-start});if(current<target)schedule();}
onmessage=({data})=>{try{if(data.type==='init'){engine=RegionHistory.create(data.input,data.policies);target=data.to;current=0;postMessage({...engine.advance(0),computeMs:0});schedule();}else if(data.type==='target'){target=data.to;schedule();}}catch(error){postMessage({error:error.message});}};
