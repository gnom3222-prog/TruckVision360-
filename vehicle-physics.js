(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionPhysics=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';

 const norm=angle=>Math.atan2(Math.sin(angle),Math.cos(angle));

 const VEHICLE_PARAMETER_LIMITS=Object.freeze({
  wheelbase:{min:1,max:15},
  hitchOffset:{min:0,max:5},
  kingpinAxles:{min:0.5,max:30},
  trailerLength:{min:4,max:30},
  trailerWidth:{min:1.5,max:5},
  trailerAxleSpacing:{min:0.3,max:5,optional:true}
 });

 function validateVehicleParams(params){
  const errors=[];
  if(!params||typeof params!=='object'){
   return ['Параметры автопоезда отсутствуют.'];
  }
  for(const [key,limits] of Object.entries(VEHICLE_PARAMETER_LIMITS)){
   const value=params[key];
   if(limits.optional&&(value===undefined||value===null))continue;
   if(!Number.isFinite(value)){
    errors.push(key+': значение должно быть конечным числом.');
    continue;
   }
   if(value<limits.min||value>limits.max){
    errors.push(key+': значение вне допустимого диапазона '+limits.min+'–'+limits.max+'.');
   }
  }
  // The three axle centers must remain inside the trailer body's longitudinal envelope.
  // The body extends 1.5 m ahead of the kingpin and trailerLength - 1.5 m behind it.
  const spacing=params.trailerAxleSpacing===undefined||params.trailerAxleSpacing===null?
   1.3:params.trailerAxleSpacing;
  const validAxleEnvelope=Number.isFinite(params.kingpinAxles)&&
   params.kingpinAxles>=VEHICLE_PARAMETER_LIMITS.kingpinAxles.min&&
   params.kingpinAxles<=VEHICLE_PARAMETER_LIMITS.kingpinAxles.max&&
   Number.isFinite(params.trailerLength)&&
   params.trailerLength>=VEHICLE_PARAMETER_LIMITS.trailerLength.min&&
   params.trailerLength<=VEHICLE_PARAMETER_LIMITS.trailerLength.max&&
   Number.isFinite(spacing)&&spacing>=0.3&&spacing<=5;
  if(validAxleEnvelope){
   const bodyFront=1.5;
   const bodyRear=-(params.trailerLength-1.5);
   [spacing,0,-spacing].forEach((offset,index)=>{
    const position=-params.kingpinAxles+offset;
    if(position>bodyFront+1e-9||position<bodyRear-1e-9){
     errors.push('trailerAxleSpacing: продольное положение оси '+(index+1)+' выходит за границы кузова полуприцепа.');
    }
   });
  }
  return errors;
 }


 function derivative(q,v,delta,params){
  const yawRate=v/params.wheelbase*Math.tan(delta);
  const beta=norm(q.psi-q.psiT);
  // No-slip constraint at the trailer axle: lateral axle velocity is zero.
  const trailerYawRate=(
   v*Math.sin(beta)
   -params.hitchOffset*yawRate*Math.cos(beta)
  )/params.kingpinAxles;

  return{
   x:v*Math.cos(q.psi),
   y:v*Math.sin(q.psi),
   psi:yawRate,
   psiT:trailerYawRate
  };
 }

 function addState(q,k,f){
  return{
   x:q.x+k.x*f,
   y:q.y+k.y*f,
   psi:q.psi+k.psi*f,
   psiT:q.psiT+k.psiT*f
  };
 }

 function rk4(q,v,delta,dt,params){
  const k1=derivative(q,v,delta,params);
  const k2=derivative(addState(q,k1,dt/2),v,delta,params);
  const k3=derivative(addState(q,k2,dt/2),v,delta,params);
  const k4=derivative(addState(q,k3,dt),v,delta,params);

  return{
   x:q.x+dt*(k1.x+2*k2.x+2*k3.x+k4.x)/6,
   y:q.y+dt*(k1.y+2*k2.y+2*k3.y+k4.y)/6,
   psi:norm(q.psi+dt*(k1.psi+2*k2.psi+2*k3.psi+k4.psi)/6),
   psiT:norm(q.psiT+dt*(k1.psiT+2*k2.psiT+2*k3.psiT+k4.psiT)/6)
  };
 }

 
 function localPoint(origin,angle,longitudinal,lateral=0){
  return{
   x:origin.x+longitudinal*Math.cos(angle)-lateral*Math.sin(angle),
   y:origin.y+longitudinal*Math.sin(angle)+lateral*Math.cos(angle)
  };
 }

 function hitchPoint(q,params){
  return localPoint(q,q.psi,-params.hitchOffset);
 }

 function axlePoint(q,params){
  const h=hitchPoint(q,params);
  return localPoint(h,q.psiT,-params.kingpinAxles);
 }

 function bodyCorners(origin,angle,front,rear,width){
  return[
   localPoint(origin,angle,front,width/2),
   localPoint(origin,angle,rear,width/2),
   localPoint(origin,angle,rear,-width/2),
   localPoint(origin,angle,front,-width/2)
  ];
 }

 function vehiclePolygons(q,params){
  const h=hitchPoint(q,params);
  const cab=localPoint(q,q.psi,2.1);
  return[
   bodyCorners(q,q.psi,5,-1.2,2.55),
   bodyCorners(cab,q.psi,1.35,-0.9,2.25),
   bodyCorners(h,q.psiT,1.5,-(params.trailerLength-1.5),params.trailerWidth)
  ];
 }

 function trailerAxleOffsets(params){
  const spacing=Number.isFinite(params.trailerAxleSpacing)&&params.trailerAxleSpacing>0?
   params.trailerAxleSpacing:1.3;
  return [spacing,0,-spacing];
 }

 function trailerAxlePoints(q,params){
  const center=axlePoint(q,params);
  return trailerAxleOffsets(params).map(offset=>localPoint(center,q.psiT,offset));
 }

 // Rigid-body lateral velocity relative to the equivalent axle-group center.
 // This is a scrub indicator, not a tire-force or multi-body dynamics model.
 function trailerAxleDiagnostics(q,v,delta,params){
  const motion=derivative(q,v,delta,params);
  const offsets=trailerAxleOffsets(params);
  const axles=offsets.map((offset,index)=>({
   index:index+1,
   offsetFromGroupCenter:offset,
   lateralVelocity:motion.psiT*offset
  }));
  return {
   trailerYawRate:motion.psiT,
   axles,
   maxAbsLateralVelocity:Math.max(...axles.map(axle=>Math.abs(axle.lateralVelocity)))
  };
 }

 // Integrates a rigid-axle-group geometric incompatibility proxy along a sampled path.
 // It is not a tire-slip, tire-force, wear, or thermal model.
 function trailerAxleScrubExposure(path,params,nominalSpeed=0.7){
  const errors=validateVehicleParams(params);
  if(errors.length)throw new Error('Некорректная геометрия осевой группы: '+errors.join(' '));
  if(!Array.isArray(path))throw new Error('Траектория должна быть массивом состояний.');
  if(!Number.isFinite(nominalSpeed)||nominalSpeed<=0||nominalSpeed>20){
   throw new Error('Расчётная скорость должна быть больше нуля и не превышать 20 м/с.');
  }
  const readState=(q,index)=>{
   if(!q||![q.x,q.y,q.psi,q.psiT].every(Number.isFinite)){
    throw new Error('Состояние траектории '+(index+1)+' содержит некорректные координаты или углы.');
   }
   const gear=q.gear===undefined?1:q.gear;
   const steer=q.steer===undefined?0:q.steer;
   if(gear!==1&&gear!==-1)throw new Error('Передача в состоянии '+(index+1)+' должна быть +1 или -1.');
   if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
    throw new Error('Угол руля в состоянии '+(index+1)+' некорректен.');
   }
   return {q,gear,steer};
  };
  const offsets=trailerAxleOffsets(params);
  const axleTravel=offsets.map(()=>0);
  const maxAbsLateralVelocityByAxle=offsets.map(()=>0);
  const states=path.map(readState);
  const evaluate=(q,steer)=>{
   // Absolute lateral speed is identical in forward and reverse for the same pose and steering angle.
   const diagnostic=trailerAxleDiagnostics(q,nominalSpeed,steer,params);
   return diagnostic.axles.map((axle,index)=>{
    const speed=Math.abs(axle.lateralVelocity);
    maxAbsLateralVelocityByAxle[index]=Math.max(maxAbsLateralVelocityByAxle[index],speed);
    return speed;
   });
  };
  states.forEach(state=>evaluate(state.q,state.steer));
  const interpolateState=(a,b,t)=>({
   x:a.q.x+(b.q.x-a.q.x)*t,
   y:a.q.y+(b.q.y-a.q.y)*t,
   psi:norm(a.q.psi+norm(b.q.psi-a.q.psi)*t),
   psiT:norm(a.q.psiT+norm(b.q.psiT-a.q.psiT)*t)
  });
  // Composite Simpson quadrature (4 equal subintervals) reduces midpoint bias.
  // It remains a sampled-path diagnostic, not a certified integration-error bound.
  const sampleTimes=[0,0.25,0.5,0.75,1];
  for(let i=1;i<states.length;i++){
   const previous=states[i-1],current=states[i];
   const distance=Math.hypot(current.q.x-previous.q.x,current.q.y-previous.q.y);
   if(distance<=0)continue;
   const samples=sampleTimes.map(t=>{
    const q=interpolateState(previous,current,t);
    const steer=previous.steer+(current.steer-previous.steer)*t;
    return evaluate(q,steer);
   });
   const elapsed=distance/nominalSpeed;
   for(let axle=0;axle<offsets.length;axle++){
    const f0=samples[0][axle],f1=samples[1][axle],f2=samples[2][axle];
    const f3=samples[3][axle],f4=samples[4][axle];
    axleTravel[axle]+=elapsed*(f0+4*f1+2*f2+4*f3+f4)/12;
   }
  }
  const totalAbsoluteLateralTravel=axleTravel.reduce((sum,value)=>sum+value,0);
  const maxAbsLateralVelocity=Math.max(0,...maxAbsLateralVelocityByAxle);
  return {
   nominalSpeed,
   maxAbsLateralVelocity,
   maxAbsLateralVelocityByAxle,
   cumulativeAbsoluteLateralTravelByAxle:axleTravel,
   maxAbsoluteLateralTravelByAxle:axleTravel.length?Math.max(...axleTravel):0,
   totalAbsoluteLateralTravel,
   integration:'composite-simpson-4-subintervals',
   model:'rigid-axle-group-kinematic-proxy'
  };
 }

 return Object.freeze({norm,validateVehicleParams,derivative,rk4,hitchPoint,axlePoint,trailerAxlePoints,trailerAxleDiagnostics,trailerAxleScrubExposure,vehiclePolygons});
});
