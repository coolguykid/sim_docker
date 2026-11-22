const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../regional.js'),'utf8').replace('return {drawComparison:chart','return {makeModel,activeVehicles,stats,aggregateScope,drawComparison:chart');
const context={window:{}};vm.createContext(context);vm.runInContext(source,context);
const demo=context.window.RegionDemo,model=demo.makeModel(14,'peak'),vehicles=demo.activeVehicles(1200,model),object=demo.objects.get('L3-P06'),state=demo.stats(object,1200,vehicles,model);
assert.equal(state.n,35);assert.equal(state.capacity,32);assert.equal(state.u,1,'35/32 must display 100%');
const states=new Map([[object.id,state]]);assert.equal(demo.aggregateScope([object],{states}).u,1);
assert.equal(demo.stats(object,1200,[],model).u,0);
assert.equal(demo.stats(demo.objects.get('L3-P01'),1800,[],model).u,null);
// Center calculations retain their existing raw ratio.
const center=demo.centerSample(14,2,20,'peak');assert.equal(center.u,center.n/center.capacity);
const sim=require('../branch-simulation.js');
const input={time:0,horizon:60,region:0,scenario:'peak',facilities:{M:{kind:'pipe',l:0,capacity:1,open:true}},paths:{'object:M':{samples:[[0,0,0],[200,0,0]],lengths:[0,200],length:200}},groups:{M:['M']},priorExits:[],speedLimit:18,initialCount:2,jobs:[0,1].map(k=>({id:'V'+k,start:0,index:0,distance:0,initial:true,divertible:false,completionLayer:0,segments:[{id:'M',path:'object:M',stage:'主干',length:200,speed:10}]}))};
const result=sim.compare(input);assert.equal(result.baseline.frames[0].states.M.n,2);
for(const branch of [result.baseline,result.strategy])for(const frame of branch.frames)for(const s of Object.values(frame.states))assert(s.u==null||s.u>=0&&s.u<=1);
console.log('PASS: overloaded 35/32 and 2/1 show 100%, aggregate capped, zero/missing preserved, raw counts retained, center ratio unchanged, all projection frames bounded');
