const assert=require('node:assert/strict');
const sim=require('../branch-simulation.js'),evaluation=require('../region-evaluation.js');
const facilities={M:{kind:'pipe',l:0,capacity:1,open:true},R:{kind:'pipe',l:0,reserve:true,capacity:1,open:false},N:{kind:'pipe',l:1,capacity:1,open:true},S:{kind:'pipe',l:1,reserve:true,capacity:1,open:false}};
const paths=Object.fromEntries(Object.keys(facilities).map((id,i)=>['object:'+id,{samples:[[0,i*35,100],[200,i*35,100]],lengths:[0,200],length:200}]));
const input={version:'test-v1',region:0,scenario:'peak',time:0,horizon:300,facilities,paths,groups:{M:['M','R'],N:['N','S']},priorExits:[],speedLimit:18,initialCount:0,jobs:[]};
for(const id of ['M','N'])for(let k=0;k<60;k++)input.jobs.push({id:id+k,stream:'B-'+id,k,start:1+k*3,index:0,distance:0,initial:false,divertible:true,completionLayer:facilities[id].l,segments:[{id,path:'object:'+id,stage:'主干',length:200,speed:10}]});
const before=JSON.stringify(input);
const baseline=sim.compare(input,{id:'baseline',label:'维持现状',layers:null}),layer=sim.compare(input,{id:'layer',label:'开放L1备用',layers:[0]}),all=sim.compare(input);
assert.equal(JSON.stringify(input),before);
assert.deepEqual(baseline.strategy,baseline.baseline);
assert.deepEqual(all.baseline,layer.baseline);
assert.deepEqual(all.strategy.frames[0],layer.strategy.frames[0]);
assert(layer.strategy.frames.some(f=>f.states.R.n>0));
assert(layer.strategy.frames.every(f=>f.states.S.n===0));
assert(all.strategy.frames.some(f=>f.states.S.n>0));
assert(all.strategy.metrics.completed>layer.strategy.metrics.completed);
assert(layer.strategy.metrics.completed>baseline.strategy.metrics.completed);
assert.throws(()=>sim.compare(input,{id:'layer',layers:[6]}));
for(const run of [baseline,layer,all])for(const branch of [run.baseline,run.strategy]){
  const last=branch.frames.at(-1);
  for(const key of ['completed','unfinished','meanWait','highLoadSeconds','diverted'])assert.equal(last[key],branch.metrics[key],'final frame matches '+key);
  for(let i=0;i<branch.frames.length;i++){
    const frame=branch.frames[i];
    for(const key of ['completed','unfinished','queue','meanWait','highLoadSeconds','diverted'])assert(Number.isFinite(frame[key])&&frame[key]>=0);
    if(i)for(const key of ['completed','highLoadSeconds','diverted'])assert(frame[key]>=branch.frames[i-1][key],key+' is cumulative');
  }
}
const waitInput={...input,horizon:60,jobs:[0,1].map(k=>({id:'WAIT'+k,start:1,index:0,distance:0,initial:false,divertible:false,completionLayer:0,segments:[{id:'M',path:'object:M',stage:'主干',length:200,speed:10}]}))};
const waitFrame=sim.simulate(waitInput,false).frames.find(f=>f.elapsed===5);
assert.equal(waitFrame.arrived,2);assert.equal(waitFrame.unfinished,2);assert.equal(waitFrame.completed,0);
assert.equal(waitFrame.queue,1);assert.equal(waitFrame.meanWait,2);assert.equal(waitFrame.highLoadSeconds,3);assert.equal(waitFrame.diverted,0);
assert(sim.simulate({...input,jobs:[]},false).frames.every(f=>f.completed===0&&f.unfinished===0&&f.queue===0&&f.meanWait===0&&f.highLoadSeconds===0&&f.diverted===0));
console.log('PASS: six effectiveness metrics per frame, known 5-second queue/wait/high-load values, cumulative progress, final-summary consistency and zero-demand series');
const data={run_id:'TEST ONLY, not a real observation',region_id:'R01',topology_version:'test-v1',scenario:'peak',policy_id:'all',policy_layers:null,snapshot_time:0,horizon:300,samples:[{sim_time:5,object_id:'M',n:2,speed:8},{sim_time:10,object_id:'M',n:6,speed:12},{sim_time:11,object_id:'M',n:2,speed:8}]};
const fixture={input,policy:all.policy,strategy:{frames:[{elapsed:5,states:{M:{n:4,speed:10}}},{elapsed:10,states:{M:{n:4,speed:10}}}]}};
evaluation.validate(data,['M']);const result=evaluation.evaluate(data,fixture);
assert.equal(result.count,2);assert.equal(result.total,3);assert.equal(result.mae,2);assert.equal(result.rmse,2);assert.equal(result.speedMae,2);assert.equal(result.speedRmse,2);
for(const changes of [{region_id:'R02'},{topology_version:'other'},{scenario:'reserve'},{policy_id:'baseline'},{snapshot_time:1},{horizon:60},{policy_layers:[0]}])assert.throws(()=>evaluation.evaluate({...data,...changes},fixture));
for(const changes of [{sim_time:0},{sim_time:301},{object_id:'UNKNOWN'},{n:-1},{n:0.5},{speed:null},{speed:-1},{max_speed:1},{n:1,min_gap:10}])assert.throws(()=>evaluation.validate({...data,samples:[{...data.samples[0],...changes}]},['M']));
assert.throws(()=>evaluation.validate({...data,samples:[data.samples[0],data.samples[0]]},['M']));
evaluation.validate({...data,samples:[{sim_time:5,object_id:'M',n:0,speed:null}]},['M']);
const none=evaluation.evaluate({...data,samples:[data.samples[2]]},fixture);assert.equal(none.count,0);assert.equal(none.mae,null);assert.equal(none.rmse,null);
console.log('PASS: three strategies, identical immutable inputs, layer isolation, distinct congestion outcomes, observation validation, exact context/time matching, known MAE/RMSE and missing points');

const synthetic=evaluation.offlineObservations(all);
evaluation.validate(synthetic,Object.keys(input.facilities));
assert.equal(synthetic.source,'offline-synthetic');
assert.deepEqual(evaluation.offlineObservations(all),synthetic,'fixed offline generation is reproducible');
assert.equal(JSON.stringify(input),before,'offline generation does not mutate simulation input');
assert(synthetic.samples.every(p=>p.sim_time>input.time&&p.sim_time<=input.time+input.horizon));
assert(synthetic.samples.every(p=>p.n!==0||p.speed===null));
const syntheticErrors=evaluation.evaluate(synthetic,all);assert(syntheticErrors.count>0);assert(syntheticErrors.mae>0);
assert.equal(syntheticErrors.count,synthetic.samples.length);
console.log('PASS: deterministic offline observations, valid zero/null values, full future coverage, nonzero demonstration errors and unchanged inputs');

// Per-time errors aggregate facilities at that instant, rather than accumulating earlier errors.
const timeFixture={input,policy:all.policy,strategy:{frames:[
  {elapsed:20,states:{M:{n:6,speed:20}}},
  {elapsed:0,states:{M:{n:0,speed:null}}},
  {elapsed:10,states:{M:{n:0,speed:null}}},
  {elapsed:5,states:{M:{n:4,speed:10},N:{n:10,speed:12}}},
  {elapsed:15,states:{N:{n:null,speed:null,missing:true}}}
]}};
const timeData={...data,samples:[
  {sim_time:5,object_id:'N',n:6,speed:16},
  {sim_time:20,object_id:'M',n:7,speed:17},
  {sim_time:5,object_id:'M',n:2,speed:8},
  {sim_time:15,object_id:'N',n:1,speed:8},
  {sim_time:10,object_id:'M',n:0,speed:null},
  {sim_time:11,object_id:'M',n:2,speed:8}
]};
evaluation.validate(timeData,['M','N']);
const timeErrors=evaluation.evaluate(timeData,timeFixture);
assert.deepEqual(timeErrors.timeline,[
  {t:5,count:2,speedCount:2,mae:3,rmse:Math.sqrt(10),speedMae:3,speedRmse:Math.sqrt(10)},
  {t:10,count:1,speedCount:0,mae:0,rmse:0,speedMae:null,speedRmse:null},
  {t:15,count:0,speedCount:0,mae:null,rmse:null,speedMae:null,speedRmse:null},
  {t:20,count:1,speedCount:1,mae:1,rmse:1,speedMae:3,speedRmse:3}
]);
assert.equal(timeErrors.mae,1.75);assert.equal(timeErrors.rmse,Math.sqrt(21/4));
assert.equal(timeErrors.speedMae,3);assert.equal(timeErrors.speedRmse,Math.sqrt(29/3));
const emptyTimes=evaluation.evaluate({...timeData,samples:[timeData.samples.at(-1)]},timeFixture);
assert(emptyTimes.timeline.every(p=>p.count===0&&p.speedCount===0&&p.mae===null&&p.rmse===null&&p.speedMae===null&&p.speedRmse===null));
console.log('PASS: per-time multi-facility MAE/RMSE, time ordering, zero errors, missing-data gaps, speed nulls, exact frame matching and weighted whole-horizon summaries');

const metricFrames=[
  {elapsed:0,meanWait:2,queue:4,highLoadSeconds:7,diverted:3,states:{A:{n:2,speed:10},B:{n:6,speed:20},C:{n:100,speed:90,missing:true},D:{n:0,speed:null}}},
  {elapsed:5,meanWait:0,queue:0,highLoadSeconds:7,diverted:3,states:{A:{n:0,speed:null},B:{n:null,speed:null,missing:true}}}
];
const metricBefore=JSON.stringify(metricFrames),aligned=evaluation.timeline({time:100},metricFrames);
assert.deepEqual(evaluation.metrics.map(p=>p.key),['meanWait','queue','highLoadSeconds','diverted','n','speed']);
assert.deepEqual(aligned,[{t:100,meanWait:2,queue:4,highLoadSeconds:7,diverted:3,n:8,speed:17.5},{t:105,meanWait:0,queue:0,highLoadSeconds:7,diverted:3,n:0,speed:null}]);
assert.equal(JSON.stringify(metricFrames),metricBefore);
assert.deepEqual(evaluation.timeline({time:0},[]),[]);
assert.equal(evaluation.timeline({time:0},[{...metricFrames[0],states:{M:{n:null,speed:null,missing:true}}}])[0].n,null);
console.log('PASS: six shared metric definitions, exact timestamps, count excludes missing facilities and queue, aircraft-weighted speed, zero/null distinction and immutable input');

// History RMSE uses paired samples in chronological order, from the selected window start.
const observed=[{t:15,n:9},{t:0,n:1},{t:5,n:2},{t:10,n:4},{t:20,n:0},{t:25,n:null},{t:30,n:2}];
const predicted=[{t:0,n:1},{t:5,n:5},{t:10,n:8},{t:15,n:null},{t:20,n:0},{t:25,n:3},{t:29,n:2}];
const rmseBefore=JSON.stringify([observed,predicted]);
assert.deepEqual(evaluation.historyRmse(observed,predicted,'n'),[
  {t:0,y:0},{t:5,y:Math.sqrt(9/2)},{t:10,y:Math.sqrt(25/3)},
  {t:15,y:null},{t:20,y:2.5},{t:25,y:null},{t:30,y:null}
]);
assert.deepEqual(evaluation.historyRmse(observed.filter(p=>p.t>=5&&p.t<=10),predicted,'n'),[{t:5,y:3},{t:10,y:Math.sqrt(25/2)}]);
assert.deepEqual(evaluation.historyRmse([],predicted,'n'),[]);
assert.deepEqual(evaluation.historyRmse([{t:0,n:NaN},{t:5,n:1},{t:10,n:Infinity}], [{t:0,n:1},{t:5,n:Infinity},{t:10,n:0}],'n'),[{t:0,y:null},{t:5,y:null},{t:10,y:null}]);
assert.equal(JSON.stringify([observed,predicted]),rmseBefore);
console.log('PASS: known cumulative history RMSE, exact time pairing, chronological ordering, window reset, missing/null/nonfinite gaps, valid zeros and immutable inputs');
