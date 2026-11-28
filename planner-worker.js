'use strict';
importScripts('./vehicle-physics.js?v=20261128-7','./articulated-planner.js?v=20261128-5');

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
  const poseRouteKeys=['candidateRoutes','conservativeCandidateRoutes'];
  const candidatePoseRoutes=poseRouteKeys.flatMap(key=>Array.isArray(result[key])?result[key]:[]);
  const selectedRouteCovered=candidatePoseRoutes.some(candidate=>
   candidate&&(candidate.index===result.selectedCandidate||candidate.states===result.states)
  );
  const poseRoutes=selectedRouteCovered||!result.ok||!Array.isArray(result.states)?
   candidatePoseRoutes:[...candidatePoseRoutes,{states:result.states}];
  const estimatedReplayStates=poseRoutes.reduce((sum,candidate)=>
   sum+(Array.isArray(candidate?.states)?candidate.states.length*17:0),0
  );
  const maxCandidatePoseReplayStates=options.maxCandidatePoseReplayStates??150000;
  const validReplayBudget=Number.isInteger(maxCandidatePoseReplayStates)&&
   maxCandidatePoseReplayStates>=1&&maxCandidatePoseReplayStates<=1000000;
  if(poseRoutes.length&&(!validReplayBudget||estimatedReplayStates>maxCandidatePoseReplayStates)){
   const note='Сравнение первых действий не выполнено: сценарная проверка всех найденных кандидатов потребовала бы '+
    estimatedReplayStates+' повторных состояний при лимите '+maxCandidatePoseReplayStates+
    '. Уменьшите сложность сцены или используйте более короткий маршрут; резерв погрешности не уменьшайте.';
   result.initialPoseSensitivity=result.ok?{
    ok:false,status:'not-assessed',reason:'candidate-pose-screen-budget-exceeded',
    estimatedReplayStates,maxCandidatePoseReplayStates,note
   }:result.initialPoseSensitivity;
   result.recommendationPoseSensitivity={
    ok:false,status:'candidate-pose-screen-budget-exceeded',stable:false,
    recommendation:null,reason:'candidate-pose-screen-budget-exceeded',
    estimatedReplayStates,maxCandidatePoseReplayStates,note
   };
   result.rejectedCandidateCount=candidatePoseRoutes.length;
   result.candidateRoutes=[];
   result.conservativeCandidateRoutes=[];
  }else{
   for(const key of poseRouteKeys){
    if(Array.isArray(result[key])){
     result[key]=result[key].map(candidate=>candidate&&Array.isArray(candidate.states)?
      {...candidate,initialPoseSensitivity:assessPose(candidate.states)}:candidate);
    }
   }
   if(result.ok&&Array.isArray(result.candidateRoutes)){
    const selectedCandidate=result.candidateRoutes.find(candidate=>
     candidate&&(candidate.index===result.selectedCandidate||candidate.states===result.states)
    );
    result.initialPoseSensitivity=selectedCandidate?.initialPoseSensitivity||
     (Array.isArray(result.states)?assessPose(result.states):{
      ok:false,status:'not-assessed',reason:'selected-candidate-pose-evidence-missing',
      note:'Не найдена сценарная проверка выбранной траектории.'
     });
   }else if(result.ok&&Array.isArray(result.states)){
    result.initialPoseSensitivity=assessPose(result.states);
   }
   if(Array.isArray(result.candidateRoutes)&&result.candidateRoutes.length){
    result.recommendationPoseSensitivity=self.TruckVisionPlanner.compareCandidateRecommendationAcrossPoseScenarios(
     result.candidateRoutes,{
      costTolerance:options.costTolerance??0.25,
      firstTurnDistanceTolerance:options.firstTurnDistanceTolerance??0.25
     }
    );
    if(result.recommendationPoseSensitivity.ok&&
       ['stable','single-candidate-unverified'].includes(result.recommendationPoseSensitivity.status)&&
       result.reason!=='ambiguous-start'&&result.initialPoseSensitivity?.ok===true){
     // Keep the worker message compact when the comparison is resolved.
     delete result.candidateRoutes;
    }else{
     const allCandidates=result.candidateRoutes;
     result.candidateRoutes=allCandidates.filter(candidate=>
      candidate&&candidate.initialPoseSensitivity?.ok===true
     ).map(candidate=>({
      ...candidate,
      recommendationPoseSensitivity:result.recommendationPoseSensitivity
     }));
     result.rejectedCandidateCount=(result.rejectedCandidateCount||0)+
      allCandidates.length-result.candidateRoutes.length;
    }
   }else{
    result.recommendationPoseSensitivity={
     ok:false,status:'not-assessed',stable:false,recommendation:null,
     reason:'candidate-routes-unavailable',
     note:'Сравнение первых действий по альтернативным траекториям не выполнено.'
    };
   }
  }
  self.postMessage({id,type:'result',result});
 }catch(error){
  self.postMessage({id,type:'error',message:error&&error.message?error.message:String(error)});
 }
});
