const assert=require('node:assert/strict'),sim=require('../branch-simulation.js');
const facilities={M:{kind:'pipe',l:0,capacity:2,open:true},R:{kind:'pipe',l:0,reserve:true,capacity:2,open:false},G:{kind:'in',l:0,capacity:1,open:true},F:{kind:'free',l:null,capacity:null,open:true}};
const paths={'object:M':{samples:[[0,0,100],[200,0,100]],lengths:[0,200],length:200},'object:R':{samples:[[0,35,100],[200,35,100]],lengths:[0,200],length:200},guide:{samples:[[0,0,0],[0,0,100]],lengths:[0,100],length:100},free:{samples:[[-100,0,0],[0,0,0]],lengths:[0,100],length:100}};
const input={time:0,horizon:300,facilities,paths,groups:{M:['M','R']},priorExits:[],speedLimit:18,initialCount:1,jobs:[]};
for(let k=0;k<80;k++)input.jobs.push({id:'V'+k,stream:'B-M',k,start:k*3,index:0,distance:k?0:50,initial:k===0,divertible:true,completionLayer:0,segments:[{id:'M',path:'object:M',stage:'主干',length:200,speed:10}]});
const before=JSON.stringify(input),a=sim.compare(input);
assert.equal(JSON.stringify(input),before);assert.deepEqual(a.baseline.frames[0],a.strategy.frames[0]);
for(const b of [a.baseline,a.strategy]){assert.equal(b.metrics.completed+b.metrics.unfinished,b.metrics.arrived);assert.equal(new Set(b.finished.map(j=>j.id)).size,b.finished.length);for(const f of b.frames){assert(f.states.M.n<=2&&f.states.R.n<=2);assert.equal(f.positions.length+f.completed,f.arrived);assert.equal(new Set(f.positions.map(p=>p.id)).size,f.positions.length);}}
assert(a.strategy.metrics.diverted>0);assert(a.strategy.metrics.completed>a.baseline.metrics.completed);assert(a.strategy.metrics.meanWait<a.baseline.metrics.meanWait);
assert.deepEqual(sim.compare(input),a);
const open=sim.compare({...input,facilities:{...facilities,R:{...facilities.R,open:true}}});assert.deepEqual(open.baseline.frames,open.strategy.frames);assert.deepEqual(open.baseline.metrics,open.strategy.metrics);
const empty=sim.compare({...input,jobs:[]});assert.equal(empty.strategy.metrics.completed,0);assert.equal(empty.strategy.metrics.meanIntervalTime,null);
const chained={...input,horizon:60,jobs:Array.from({length:10},(_,k)=>({id:'C'+k,stream:'C',k,start:k*2,index:0,distance:0,initial:k===0,divertible:false,completionLayer:0,segments:[{id:'F',path:'free',stage:'自由飞',length:100,speed:10},{id:'G',path:'guide',stage:'汇入',length:100,speed:10},{id:'M',path:'object:M',stage:'主干',length:200,speed:10}]}))};
const c=sim.compare(chained);assert(c.baseline.metrics.totalWait>0,'Guide participates in capacity/queue model');assert(c.baseline.frames.every(f=>f.states.G.n<=1));assert.deepEqual(c.baseline.frames.map(f=>f.positions),c.strategy.frames.map(f=>f.positions),'Non-boundary chained tasks keep original routes');
const snap=sim.snapshot(input,a.strategy,10);assert(snap.vehicles.every(v=>v.position.every(Number.isFinite)&&v.ahead.every(Number.isFinite)));assert(snap.vehicles.every(v=>v.objectId!=='R'||Math.abs(v.position[1]-35)<1e-8));
console.log('PASS: whole-region immutable snapshot, identical initial frames, task conservation, pipe/guide capacity and queues, deterministic diversion, open-reserve equality, empty demand, route continuity and world positions');
for(const openReserve of [false,true]){
  const runner=sim.createRunner(input,openReserve);
  for(const end of [0,1,4,5,17,60,299,300])assert.deepEqual(runner.advance(end),sim.simulate({...input,horizon:end},openReserve),'incremental advancement equals a fresh run through '+end);
  assert.throws(()=>runner.advance(299));assert.throws(()=>runner.advance(301));assert.throws(()=>runner.advance(300.5));
  const observed=[],compact=sim.createRunner(input,openReserve,null,{retainFrames:false,onFrame:f=>observed.push({elapsed:f.elapsed,queue:f.queue})});
  compact.advance(17);const result=compact.advance(300);
  assert.equal(result.frames.length,1);assert.deepEqual(result.frames[0],sim.simulate(input,openReserve).frames.at(-1));
  assert.deepEqual(result.metrics,sim.simulate(input,openReserve).metrics);assert(observed.every(f=>f.elapsed<=300));assert.equal(observed.at(-1).elapsed,300);
}
assert.equal(JSON.stringify(input),before);
console.log('PASS: incremental replay equals complete simulation at exact cutoffs, non-5-second endpoints, monotonic bounds, compact frame retention, callbacks and immutable demand');
const crypto=require('node:crypto'),golden=['cfcc0b96b15ca663a0619e0821dfb889fb95424c75d9578b42ce7b3c7abcdbc6','ba40f94922ccb31b7a148b85e9fdc055c29e7b87577a6f03b6f2ab1480bace81','df89dc969a53701006c9e19f18802b7ec32dc4ac75ba5fae9c96e41d684aa799','61529bf5b449a0f0354953e331176538d7c7c716b58c2b47b1ee37a8644254f1'];
assert.deepEqual([a.baseline,a.strategy,c.baseline,c.strategy].map(v=>crypto.createHash('sha256').update(JSON.stringify(v)).digest('hex')),golden,'complete results match the pre-optimization simulator');
const history=require('../region-history.js'),evaluation=require('../region-evaluation.js');
const policies=[{id:'baseline',layers:null},{id:'all',layers:null},...Array.from({length:6},(_,l)=>({id:'layer',layers:[l]}))];
for(const source of [input,chained,{...input,jobs:[]},{...input,facilities:{...facilities,M:{...facilities.M,state:{missing:true}}}},{...input,jobs:[...input.jobs].reverse()}]){
  for(const policy of policies){const points=[],runner=sim.createRunner(source,policy.id!=='baseline',policy.layers,{summaryOnly:true,sampleStep:1,onFrame:({elapsed,...p})=>points.push(p)});
    runner.advance(source.horizon);const complete=sim.createRunner(source,policy.id!=='baseline',policy.layers,{sampleStep:1}).advance(source.horizon);
    assert.deepEqual(points,evaluation.timeline(source,complete.frames),'lightweight samples equal full-frame aggregation at every second');}
}
const engine=history.create(input,policies),batches=[engine.advance(0),engine.advance(17),engine.advance(60),engine.advance(300)];
for(let k=0;k<policies.length;k++){const samples=batches.flatMap(b=>b.runs[k].timeline);assert.equal(samples.length,301);assert(samples.every((p,i)=>p.t===i));assert(samples.every(p=>!('states'in p)&&!('positions'in p)));}
assert.throws(()=>engine.advance(299));assert.throws(()=>engine.advance(301));
assert.throws(()=>sim.createRunner(input,false,null,{sampleStep:0}));assert.throws(()=>sim.createRunner(input,false,null,{sampleStep:1.5}));
console.log('PASS: pre-optimization output hashes, every-second lightweight equivalence across eight policies, zero/missing/unsorted inputs and incremental history batches');
