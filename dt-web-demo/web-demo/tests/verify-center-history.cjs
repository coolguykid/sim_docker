const assert=require('node:assert/strict'),P=require('../center-projection.js'),E=require('../region-evaluation.js');
const engine=P.createHistory('peak'),zero=engine.advance(0),first=engine.advance(125),next=engine.advance(300);
assert.equal(zero.to,0);assert.equal(zero.runs.length,4);assert(zero.runs.every(r=>r.timeline.length===1&&r.timeline[0].t===0));
assert.equal(first.to,125);assert(first.runs.every(r=>r.timeline.length===25));assert.equal(next.to,300);
const runs=zero.runs.map((r,i)=>({...r,timeline:[...r.timeline,...first.runs[i].timeline,...next.runs[i].timeline]}));
const input=P.capture(0,'peak',300);
for(const [i,id]of ['baseline','baseline','balance','reserve'].entries()){
  const expected=P.compare(input,id).strategy.frames.map(f=>({t:f.elapsed,...f.metrics}));assert.deepEqual(runs[i].timeline,expected);
  assert(runs[i].timeline.every(p=>!('cells'in p)&&!('transfers'in p)));
}
assert.deepEqual(runs[0].timeline,runs[1].timeline);
for(const key of E.metrics.map(p=>p.key)){
  assert(E.historyRmse(runs[0].timeline,runs[1].timeline,key).every(p=>p.y===0||p.y===null));
  const errors=[];const expected=runs[0].timeline.map((p,i)=>{const q=runs[2].timeline[i];if(!Number.isFinite(p[key])||!Number.isFinite(q[key]))return {t:p.t,y:null};errors.push((q[key]-p[key])**2);return {t:p.t,y:Math.sqrt(errors.reduce((s,x)=>s+x,0)/errors.length)};});
  assert.deepEqual(E.historyRmse(runs[0].timeline,runs[2].timeline,key),expected);
}
assert(engine.advance(125).runs.every(r=>r.timeline.length===0));
assert.equal(engine.advance(302).to,300);const end=engine.advance(3600);assert.equal(end.to,3600);assert(end.runs.every(r=>r.timeline.at(-1).t===3600));
for(const t of [-1,3601,NaN,Infinity])assert.throws(()=>engine.advance(t));
const missing=P.createHistory('peak','missing').advance(300);assert(missing.runs.every(r=>r.timeline.every(p=>p.n!==null&&Number.isFinite(p.speed))));
assert.throws(()=>P.createHistory('unknown'));
console.log('PASS: four independent historical runs, exact full-model equivalence, deterministic incremental batches, six RMSE metrics, zero baseline, thin results, rewind, five-second cutoff, hour endpoint and missing-sample aggregation');
