/* Shared by the history worker and the cooperative fallback. Only metric samples leave the simulator. */
(function(root){
  'use strict';
  function create(input,policies){
    const simulation=root.BranchSimulation||(typeof require==='function'?require('./branch-simulation.js'):null);
    const runs=policies.map(policy=>{const samples=[];const runner=simulation.createRunner(input,policy.id!=='baseline',policy.layers,{summaryOnly:true,retainFrames:false,sampleStep:1,onFrame:({elapsed,...point})=>samples.push(point)});return {policy,samples,runner};});
    let current=0;
    function advance(to){for(const run of runs)run.runner.advance(to);const from=current;current=to;return {from,to,runs:runs.map(run=>({policy:run.policy,timeline:run.samples.splice(0)}))};}
    return {advance};
  }
  if(typeof module!=='undefined'&&module.exports)module.exports={create};else root.RegionHistory={create};
})(typeof window==='undefined'?globalThis:window);
