'use strict';
importScripts('./vehicle-physics.js?v=20261128-4','./articulated-planner.js?v=20261128-3');

self.addEventListener('message',event=>{
 const data=event.data||{};
 const id=data.id;
 try{
  if(!self.TruckVisionPhysics||!self.TruckVisionPlanner)throw new Error('Модули физики или планировщика не загрузились.');
  const options=data.options||{};
  const integrate=(q,v,delta,dt,params)=>self.TruckVisionPhysics.rk4(q,v,delta,dt,params);
  const result=self.TruckVisionPlanner.planAlternativesWithVehicleParameterBounds({
   ...options,
   integrate,
   onProgress:progress=>self.postMessage({id,type:'progress',progress})
  });
  const poseOptions={
   goal:options.goal,
   minClearance:options.minClearance,
   positionTolerance:options.positionTolerance,
   headingTolerance:options.headingTolerance,
   maxTerminalArticulation:options.maxTerminalArticulation
  };
  const assessPose=states=>self.TruckVisionPlanner.analyzeInitialPoseUncertainty(
   states,options.obstacles,options.params,options.initialPoseUncertainty,
   integrate,poseOptions
  );
  if(result.ok&&Array.isArray(result.states)){
   result.initialPoseSensitivity=assessPose(result.states);
  }
  for(const key of ['candidateRoutes','conservativeCandidateRoutes']){
   if(Array.isArray(result[key])){
    result[key]=result[key].map(candidate=>candidate&&Array.isArray(candidate.states)?
     {...candidate,initialPoseSensitivity:assessPose(candidate.states)}:candidate);
   }
  }
  self.postMessage({id,type:'result',result});
 }catch(error){
  self.postMessage({id,type:'error',message:error&&error.message?error.message:String(error)});
 }
});
