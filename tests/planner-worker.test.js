'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');

test('planner worker reports ambiguous reverse starts instead of choosing one arbitrarily',()=>{
 const messages=[];
 let onMessage=null;
 const sandbox={};
 sandbox.self=sandbox;
 sandbox.postMessage=message=>messages.push(message);
 sandbox.addEventListener=(type,handler)=>{if(type==='message')onMessage=handler;};
 const context=vm.createContext(sandbox);
 sandbox.importScripts=(...files)=>{
  for(const file of files){
   const cleanFile=file.split('?')[0];
   const source=fs.readFileSync(path.join(__dirname,'..',cleanFile),'utf8');
   vm.runInContext(source,context,{filename:cleanFile});
  }
 };
 const workerSource=fs.readFileSync(path.join(__dirname,'..','planner-worker.js'),'utf8');
 vm.runInContext(workerSource,context,{filename:'planner-worker.js'});
 assert.equal(typeof onMessage,'function','worker should register a message handler');
 onMessage({data:{
  id:17,
  options:{
   start:{x:0,y:0,psi:0,psiT:0},
   goal:{x:-3,y:0},
   params:{wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55},
   obstacles:[],
   maxExpansions:5000,
   minClearance:0.3,
   measurementUncertainty:0.1
  }
 }});
 assert.ok(messages.some(m=>m.id===17&&m.type==='progress'),'worker should send progress');
 assert.ok(messages.some(m=>m.id===17&&m.type==='progress'&&m.progress.scenario==='reserve'),'worker should report the uncertainty-reserve scenario');
 const resultMessage=messages.find(m=>m.id===17&&m.type==='result');
 assert.ok(resultMessage,'worker should return a result');
 assert.equal(resultMessage.result.ok,false,JSON.stringify(resultMessage.result));
 assert.equal(resultMessage.result.reason,'ambiguous-start');
 assert.equal(resultMessage.result.status,'ambiguous');
 assert.equal(resultMessage.result.candidatesAttempted,5);
 assert.ok(resultMessage.result.candidatesFound>=2);
 assert.equal(resultMessage.result.comparison.recommendation,null,'ambiguous candidates must not produce a single start recommendation');
 assert.equal(resultMessage.result.parameterSensitivity.status,'not-varied','zero physical dimension tolerances must be reported as not assessed');
 assert.ok(resultMessage.result.candidateSummaries.some(candidate=>candidate.initialGear===-1&&candidate.firstTurn==='left'));
 assert.ok(resultMessage.result.candidateSummaries.some(candidate=>candidate.initialGear===-1&&candidate.firstTurn==='right'));
});
