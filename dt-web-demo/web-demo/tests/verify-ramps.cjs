const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const context={window:{}};vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(__dirname,'../regional.js'),'utf8'),context);
const objects=context.window.RegionDemo.objects,ramps=[...objects.values()].filter(o=>o.kind==='ramp');
const distance=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));
assert.equal(ramps.length,20,'Ten upward and ten independent downward ramps');
let down=0;
for(const r of ramps){
  const descending=r.targetLayer<r.l,source=objects.get(`L${r.l+1}-P${r.p?'06':'01'}`),target=objects.get(`L${r.targetLayer+1}-P${r.p?'06':'01'}`);
  assert.equal(Math.abs(r.targetLayer-r.l),1);
  assert(distance(r.at(0),source.at(descending?.3:.6))<1e-8,`${r.id}: source exit`);
  assert(distance(r.at(1),target.at(descending?.42:.72))<1e-8,`${r.id}: target entry`);
  const tangent=(a,b)=>b.map((v,i)=>v-a[i]),dot=(a,b)=>a.reduce((n,v,i)=>n+v*b[i],0);
  assert(dot(tangent(r.points[0],r.points[1]),tangent(source.at(0),source.at(1)))>0,`${r.id}: exit follows source traffic`);
  assert(dot(tangent(r.points[2],r.points[3]),tangent(target.at(0),target.at(1)))>0,`${r.id}: entry follows target traffic`);
  for(let i=1;i<r.samples.length;i++)assert((r.samples[i][2]-r.samples[i-1][2])*(descending?-1:1)>=-1e-8,`${r.id}: monotonic altitude`);
  assert(r.length>distance(r.at(0),r.at(1)),`${r.id}: computed curve length`);
  if(descending){down++;const reverse=objects.get(`X-L${r.targetLayer+1}-L${r.l+1}${r.p?'-R':''}`);assert(reverse);assert(distance(r.at(0),reverse.at(1))>1,`${r.id}: independent interfaces`);}
}
assert.equal(down,10);
console.log('PASS: 20 ramps, 10 up / 10 down; independent interfaces, traffic tangents and monotonic heights');
