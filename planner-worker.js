'use strict';
importScripts('./vehicle-physics.js?v=20261128-4','./articulated-planner.js?v=20261128-5');

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
  let poseObstacles=Array.isArray(options.obstacles)?options.obstacles.slice():[];
  if(options.gateGeometry!==undefined&&options.gateGeometry!==null){
   const boundedGatePosts=self.TruckVisionPlanner.createGateObstacles(options.gateGeometry);
   poseObstacles=[
    ...poseObstacles.filter(obstacle=>obstacle?.kind!=='gate-jamb'),
    ...boundedGatePosts
   ];
  }
  const obstaclePositionUncertainty=options.obstaclePositionUncertainty??0;
  const obstacleSizeUncertainty=options.obstacleSizeUncertainty??0;
  const obstacleExpansionPerSide=obstaclePositionUncertainty+obstacleSizeUncertainty/2;
  poseObstacles=poseObstacles.map(obstacle=>{
   if(Array.isArray(obstacle?.polygon)){
    return {...obstacle,polygon:self.TruckVisionPlanner.inflateConvexPolygon(
     obstacle.polygon,Math.SQRT2*obstacleExpansionPerSide
    )};
   }
   return {...obstacle,w:obstacle.w+2*obstacleExpansionPerSide,
    h:obstacle.h+2*obstacleExpansionPerSide};
  });
  const baseFootprint=options.params.footprintUncertainty??0;
  const vehicleFootprintUncertainty=options.vehicleFootprintUncertainty??0;
  const poseParams={
   ...options.params,
   footprintUncertainty:baseFootprint+vehicleFootprintUncertainty
  };
  const measurementReserve=2*(options.measurementUncertainty??0);
  const requiredClearance=(options.minClearance??0.3)+measurementReserve;
  const poseOptions={
   goal:options.goal,
   minClearance:requiredClearance,
   positionTolerance:options.positionTolerance,
   headingTolerance:options.headingTolerance,
   maxTerminalArticulation:options.maxTerminalArticulation
  };
  const assessPose=states=>{
   const analysis=self.TruckVisionPlanner.analyzeInitialPoseUncertainty(
    states,poseObstacles,poseParams,options.initialPoseUncertainty,
    integrate,poseOptions
   );
   return {
    ...analysis,
    geometryEnvelopeApplied:true,
    expandedObstacleCount:poseObstacles.length,
    obstacleExpansionPerSide,
    expandedFootprintUncertainty:poseParams.footprintUncertainty,
    measurementReserve,
    gateGeometryIncluded:options.gateGeometry!==undefined&&options.gateGeometry!==null
   };
  };
  if(result.ok&&Array.isArray(result.states)){
   result.initialPoseSensitivity=assessPose(result.states);
  }
  for(const key of ['candidateRoutes','conservativeCandidateRoutes']){
   if(Array.isArray(result[key])){
    result[key]=result[key].map(candidate=>candidate&&Array.isArray(candidate.states)?
     {...candidate,initialPoseSensitivity:assessPose(candidate.states)}:candidate);
   }
  }
  if(Array.isArray(result.candidateRoutes)&&result.candidateRoutes.length){
   result.recommendationPoseSensitivity=self.TruckVisionPlanner.compareCandidateRecommendationAcrossPoseScenarios(
    result.candidateRoutes,{costTolerance:options.costTolerance??0.25}
   );
   if(result.recommendationPoseSensitivity.ok&&
      ['stable','single-candidate-unverified'].includes(result.recommendationPoseSensitivity.status)&&
      result.reason!=='ambiguous-start'){
    // Keep the worker message compact when the comparison is resolved.
    delete result.candidateRoutes;
   }else{
    result.candidateRoutes=result.candidateRoutes.map(candidate=>({
     ...candidate,
     recommendationPoseSensitivity:result.recommendationPoseSensitivity
    }));
   }
  }else{
   result.recommendationPoseSensitivity={
    ok:false,status:'not-assessed',stable:false,recommendation:null,
    reason:'candidate-routes-unavailable',
    note:'Сравнение первых действий по альтернативным траекториям не выполнено.'
   };
  }
  self.postMessage({id,type:'result',result});
 }catch(error){
  self.postMessage({id,type:'error',message:error&&error.message?error.message:String(error)});
 }
});
