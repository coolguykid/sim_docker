// Run from the project root: node web-demo/tests/verify-direct-guides.cjs.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const context = {window:{}};
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname,'../regional.js'),'utf8'),context);
const {objects} = context.window.RegionDemo;
const distance = (a,b) => Math.hypot(...a.map((v,i)=>v-b[i]));
assert(!objects.has('IN-STEM')&&!objects.has('OUT-STEM'),'Remove the shared highest-layer guides');
assert.equal(objects.size,107,'Direct guides and twenty adjacent-layer ramps');
let checked = 0;
for(const o of objects.values()){
  if(!['in','out'].includes(o.kind))continue;
  const incoming = o.kind==='in',pipe=objects.get(o.pipe);
  const funnel=objects.get(incoming?'TK-01':'LD-01');
  assert(pipe,`${o.id}: target pipe exists`);
  assert(distance(o.at(incoming?0:1),funnel.at(incoming?1:0))<1e-8,`${o.id}: connects directly to funnel`);
  assert(distance(o.at(incoming?1:0),pipe.at(incoming?.2:.8))<1e-8,`${o.id}: connects to target pipe`);
  for(let i=1;i<o.parts.length;i++){
    assert(distance(o.parts[i-1].at(1),o.parts[i].at(0))<1e-8,`${o.id}: continuous curve pieces`);
  }
  for(const [i,p] of o.samples.entries()){
    assert(p[2]>=110-1e-8&&p[2]<=pipe.height+1e-8,`${o.id}: stays between funnel and target height`);
    if(i)assert((p[2]-o.samples[i-1][2])*(incoming?1:-1)>=-1e-8,`${o.id}: no vertical detour`);
  }
  checked++;
}
assert.equal(checked,24,'Six layers, two directions, incoming and outgoing');
console.log(`PASS: ${checked} direct guides connect funnels to target layers without highest-layer detours`);
