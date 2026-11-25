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
  trailerAxleSpacing:{min:0.3,max:5,optional:true},
  footprintUncertainty:{min:0,max:1,optional:true},
  cabOffset:{min:0,max:6,optional:true},
  cabFront:{min:0.5,max:3.5,optional:true},
  cabRear:{min:0.3,max:2.5,optional:true},
  cabWidth:{min:1.8,max:2.55,optional:true},
  mirrorOutboard:{min:0,max:1,optional:true},
  mirrorLongitudinal:{min:-2,max:8,optional:true},
  mirrorLength:{min:0.2,max:2,optional:true},
  tractorFrontTrack:{min:1,max:3.5,optional:true},
  tractorRearTrack:{min:1,max:3.5,optional:true},
  trailerTrack:{min:1,max:4.5,optional:true},
  tireLength:{min:0.2,max:1.5,optional:true},
  tireWidth:{min:0.1,max:0.8,optional:true}
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

  // Cab dimensions are configurable calibration inputs, not factory defaults.
  const cabOffset=params.cabOffset??2.1;
  const cabFront=params.cabFront??1.35;
  const cabRear=params.cabRear??0.9;
  const cabWidth=params.cabWidth??2.25;
  const validCabEnvelope=[cabOffset,cabFront,cabRear,cabWidth].every(Number.isFinite)&&
   cabOffset>=0&&cabOffset<=6&&cabFront>=0.5&&cabFront<=3.5&&
   cabRear>=0.3&&cabRear<=2.5&&cabWidth>=1.8&&cabWidth<=2.55;
  if(validCabEnvelope&&(
   cabOffset+cabFront>5+1e-9||
   cabOffset-cabRear< -1.2-1e-9||
   cabWidth>2.55+1e-9
  )){
   errors.push('Геометрия кабины выходит за пределы общего контура тягача; проверьте положение, длину и ширину кабины.');
  }

  const mirrorOutboard=params.mirrorOutboard??0;
  const mirrorLongitudinal=params.mirrorLongitudinal??3.0;
  const mirrorLength=params.mirrorLength??0.8;
  if(validCabEnvelope&&Number.isFinite(mirrorOutboard)&&mirrorOutboard>0&&
     Number.isFinite(mirrorLongitudinal)&&Number.isFinite(mirrorLength)&&
     mirrorOutboard<=1&&mirrorLongitudinal>=-2&&mirrorLongitudinal<=8&&
     mirrorLength>=0.2&&mirrorLength<=2){
   const cabRearEdge=cabOffset-cabRear;
   const cabFrontEdge=cabOffset+cabFront;
   if(mirrorLongitudinal-mirrorLength/2<cabRearEdge-1e-9||
      mirrorLongitudinal+mirrorLength/2>cabFrontEdge+1e-9){
    errors.push('mirrorLongitudinal/mirrorLength: оболочка зеркал должна располагаться вдоль боковой поверхности кабины.');
   }
  }

  // Wheel/track dimensions are calibration inputs, not Volvo factory specifications.
  const frontTrack=params.tractorFrontTrack??2.05;
  const rearTrack=params.tractorRearTrack??2.05;
  const trailerTrack=params.trailerTrack??(Number.isFinite(params.trailerWidth)?params.trailerWidth+0.24:NaN);
  const tireWidth=params.tireWidth??0.44;
  if([frontTrack,rearTrack,trailerTrack,tireWidth].every(Number.isFinite)){
   if(frontTrack+tireWidth>3.2+1e-9||rearTrack+tireWidth>3.2+1e-9){
    errors.push('tractorTrack/tireWidth: наружная ширина колёс тягача выходит за допустимую калибровочную оболочку 3,20 м.');
   }
   if(Number.isFinite(params.trailerWidth)&&trailerTrack+tireWidth>params.trailerWidth+1.0+1e-9){
    errors.push('trailerTrack/tireWidth: колёса выходят за допустимую калибровочную оболочку относительно ширины полуприцепа.');
   }
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

 function tractorAxlePoints(q,params){
  if(!q||![q.x,q.y,q.psi].every(Number.isFinite)||!params||
   !Number.isFinite(params.wheelbase)||params.wheelbase<=0){
   throw new Error('Параметры осей тягача некорректны.');
  }
  // q is the rear-axle reference: the steer axle is one wheelbase forward.
  return [
   localPoint(q,q.psi,params.wheelbase),
   {x:q.x,y:q.y}
  ];
 }

 function bodyCorners(origin,angle,front,rear,width){
  return[
   localPoint(origin,angle,front,width/2),
   localPoint(origin,angle,rear,width/2),
   localPoint(origin,angle,rear,-width/2),
   localPoint(origin,angle,front,-width/2)
  ];
 }

 function polygonOutsideEnvelope(poly,origin,angle,front,rear,width){
  const c=Math.cos(angle),s=Math.sin(angle),halfWidth=width/2;
  return poly.some(point=>{
   const dx=point.x-origin.x,dy=point.y-origin.y;
   const longitudinal=dx*c+dy*s;
   const lateral=-dx*s+dy*c;
   return longitudinal>front+1e-9||longitudinal<rear-1e-9||Math.abs(lateral)>halfWidth+1e-9;
  });
 }

 function vehiclePolygons(q,params){
  const h=hitchPoint(q,params);
  const cabOffset=params.cabOffset??2.1;
  const cabFront=params.cabFront??1.35;
  const cabRear=params.cabRear??0.9;
  const cabWidth=params.cabWidth??2.25;
  const cab=localPoint(q,q.psi,cabOffset);
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  // The tractor rectangle remains a broad outer envelope. The cab is a
  // nested diagnostic component whose dimensions can be calibrated separately.
  return[
   bodyCorners(q,q.psi,5+uncertainty,-1.2-uncertainty,2.55+2*uncertainty),
   bodyCorners(cab,q.psi,cabFront+uncertainty,-cabRear-uncertainty,cabWidth+2*uncertainty),
   bodyCorners(h,q.psiT,1.5+uncertainty,-(params.trailerLength-1.5+uncertainty),params.trailerWidth+2*uncertainty)
  ];
 }

 // Optional side-mirror envelopes. mirrorOutboard is the measured projection
 // beyond each cab side; zero means the mirrors are not included in collision geometry.
 function mirrorPolygons(q,params){
  const outboard=params.mirrorOutboard??0;
  if(!Number.isFinite(outboard)||outboard<=0)return [];
  const cabWidth=params.cabWidth??2.25;
  const longitudinal=params.mirrorLongitudinal??3.0;
  const length=params.mirrorLength??0.8;
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  return [1,-1].map((side,index)=>{
   const center=localPoint(q,q.psi,longitudinal,side*(cabWidth/2+outboard/2));
   return {
    name:index===0?'левое зеркало':'правое зеркало',
    polygon:bodyCorners(center,q.psi,length/2+uncertainty,-length/2-uncertainty,outboard+2*uncertainty)
   };
  });
 }

 // Collision components are grouped into the same three driver-facing parts.
 // Mirrors are separate cab components, so their nearest obstacle can be named.
 function vehicleCollisionParts(q,params){
  const bodies=vehiclePolygons(q,params);
  const mirrors=mirrorPolygons(q,params);
  const wheels=buildWheelFootprints(q,params,0);
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  const tractorWheelComponents=wheels.filter(wheel=>
   wheel.vehiclePart==='тягач'&&polygonOutsideEnvelope(
    wheel.polygon,q,q.psi,5+uncertainty,-1.2-uncertainty,2.55+2*uncertainty
   )
  );
  const hitch=hitchPoint(q,params);
  const trailerWheelComponents=wheels.filter(wheel=>
   wheel.vehiclePart==='полуприцеп'&&polygonOutsideEnvelope(
    wheel.polygon,hitch,q.psiT,1.5+uncertainty,
    -(params.trailerLength-1.5+uncertainty),params.trailerWidth+2*uncertainty
   )
  );
  return [
   {vehiclePart:'тягач',bodyIndex:0,components:[
    {name:'основной контур тягача',polygon:bodies[0]},
    ...tractorWheelComponents.map(wheel=>({name:wheel.name,polygon:wheel.polygon}))
   ]},
   {vehiclePart:'кабина',bodyIndex:1,components:[
    {name:'кабина',polygon:bodies[1]},
    ...mirrors
   ]},
   {vehiclePart:'полуприцеп',bodyIndex:2,components:[
    {name:'кузов полуприцепа',polygon:bodies[2]},
    ...trailerWheelComponents.map(wheel=>({name:wheel.name,polygon:wheel.polygon}))
   ]}
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

 // Build the ten calibrated rectangular tire footprints without re-validating the
 // complete vehicle parameter set on every planner node.
 function buildWheelFootprints(q,params,steer=0){
  const frontTrack=params.tractorFrontTrack??2.05;
  const rearTrack=params.tractorRearTrack??2.05;
  const trailerTrack=params.trailerTrack??(params.trailerWidth+0.24);
  const tireLength=params.tireLength??0.72;
  const tireWidth=params.tireWidth??0.44;
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  const wheels=[];
  const add=(vehiclePart,axleIndex,side,center,heading,name)=>{
   wheels.push({
    vehiclePart,axleIndex,side,center,heading,name,
    polygon:bodyCorners(center,heading,
     tireLength/2+uncertainty,-tireLength/2-uncertainty,tireWidth+2*uncertainty)
   });
  };
  const [frontAxle,rearAxle]=tractorAxlePoints(q,params);
  for(const side of [1,-1]){
   const sideName=side>0?'левое':'правое';
   add('тягач',1,side>0?'left':'right',
    localPoint(frontAxle,q.psi,0,side*frontTrack/2),q.psi+steer,
    sideName+' переднее колесо тягача');
   add('тягач',2,side>0?'left':'right',
    localPoint(rearAxle,q.psi,0,side*rearTrack/2),q.psi,
    sideName+' заднее колесо тягача');
  }
  trailerAxlePoints(q,params).forEach((axle,index)=>{
   for(const side of [1,-1]){
    const sideName=side>0?'левое':'правое';
    add('полуприцеп',index+1,side>0?'left':'right',
     localPoint(axle,q.psiT,0,side*trailerTrack/2),q.psiT,
     sideName+' колесо '+(index+1)+'-й оси полуприцепа');
   }
  });
  return wheels;
 }

 function wheelFootprints(q,params,steer=0){
  const errors=validateVehicleParams(params);
  if(errors.length)throw new Error('Некорректные параметры колёс: '+errors.join(' '));
  if(!q||![q.x,q.y,q.psi,q.psiT].every(Number.isFinite)){
   throw new Error('Положение автопоезда для геометрии колёс некорректно.');
  }
  if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
   throw new Error('Угол поворота передних колёс некорректен.');
  }
  return buildWheelFootprints(q,params,steer);
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
  if(path.length===0)throw new Error('Траектория должна содержать хотя бы одно состояние.');
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
  // Array.from visits sparse slots so a hole cannot masquerade as an empty, zero-exposure route.
  const states=Array.from(path,readState);
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


 // Returns sampled center tracks for all six trailer tires and all three axle centers.
 // Left/right inner-side classification is relative to the direction of travel:
 // reverse gear swaps the vehicle-side label. Distances are chords between route samples.
 function trailerWheelTrajectoryDiagnostics(path,params){
  const errors=validateVehicleParams(params);
  if(errors.length)throw new Error('Некорректная геометрия траектории колёс: '+errors.join(' '));
  if(!Array.isArray(path))throw new Error('Траектория колёс должна быть массивом состояний.');
  if(path.length===0)throw new Error('Траектория колёс должна содержать хотя бы одно состояние.');
  const readState=(state,index)=>{
   if(!state||![state.x,state.y,state.psi,state.psiT].every(Number.isFinite)){
    throw new Error('Состояние траектории колёс '+(index+1)+' содержит некорректные координаты или углы.');
   }
   const gear=state.gear===undefined?1:state.gear;
   const steer=state.steer===undefined?0:state.steer;
   if(gear!==1&&gear!==-1)throw new Error('Передача в состоянии траектории колёс '+(index+1)+' должна быть +1 или -1.');
   if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
    throw new Error('Угол руля в состоянии траектории колёс '+(index+1)+' некорректен.');
   }
   return {q:state,gear,steer};
  };
  const states=Array.from(path,readState);
  const track=params.trailerTrack??(params.trailerWidth+0.24);
  if(!Number.isFinite(track)||track<=0)throw new Error('Колея полуприцепа для траектории колёс некорректна.');
  const offsets=trailerAxleOffsets(params);
  const axles=offsets.map((offset,index)=>({
   axleIndex:index+1,offsetFromGroupCenter:offset,
   center:{points:[],sampledTravel:0},
   left:{side:'left',name:'левое колесо '+(index+1)+'-й оси полуприцепа',points:[],sampledTravel:0},
   right:{side:'right',name:'правое колесо '+(index+1)+'-й оси полуприцепа',points:[],sampledTravel:0}
  }));
  const segments=[];
  let routeDistance=0;
  let previousTracks=null;
  let previousState=null;
  for(let index=0;index<states.length;index++){
   const state=states[index];
   if(index===0&&state.q.stepDistance!==undefined&&
      (!Number.isFinite(state.q.stepDistance)||state.q.stepDistance!==0)){
    throw new Error('Начальное состояние траектории колёс должно иметь stepDistance равный нулю.');
   }
   if(index>0){
    const chord=Math.hypot(state.q.x-previousState.q.x,state.q.y-previousState.q.y);
    let segmentDistance=chord;
    if(state.q.stepDistance!==undefined){
     if(!Number.isFinite(state.q.stepDistance)||state.q.stepDistance<=0){
      throw new Error('Состояние траектории колёс '+(index+1)+' содержит некорректный stepDistance.');
     }
     if(chord>state.q.stepDistance+1e-6){
      throw new Error('Хорда траектории колёс превышает stepDistance на сегменте '+index+'.');
     }
     segmentDistance=state.q.stepDistance;
    }
    routeDistance+=segmentDistance;
   }
   const axleCenters=trailerAxlePoints(state.q,params);
   const currentTracks=axleCenters.map((center,axleIndex)=>({
    center,
    left:localPoint(center,state.q.psiT,0,track/2),
    right:localPoint(center,state.q.psiT,0,-track/2)
   }));
   currentTracks.forEach((sample,axleIndex)=>{
    const axle=axles[axleIndex];
    const append=(trackData,point)=>{
     if(trackData.points.length){
      const previous=trackData.points[trackData.points.length-1];
      trackData.sampledTravel+=Math.hypot(point.x-previous.x,point.y-previous.y);
     }
     trackData.points.push({
      x:point.x,y:point.y,routeDistance,heading:state.q.psiT,gear:state.gear,sampleIndex:index
     });
    };
    append(axle.center,sample.center);
    append(axle.left,sample.left);
    append(axle.right,sample.right);
   });
   if(previousTracks){
    const deltaHeading=norm(state.q.psiT-previousState.q.psiT);
    const axleSegments=currentTracks.map((sample,axleIndex)=>{
     const previous=previousTracks[axleIndex];
     const leftWheelDistance=Math.hypot(sample.left.x-previous.left.x,sample.left.y-previous.left.y);
     const rightWheelDistance=Math.hypot(sample.right.x-previous.right.x,sample.right.y-previous.right.y);
     const centerDistance=Math.hypot(sample.center.x-previous.center.x,sample.center.y-previous.center.y);
     const turnDirection=centerDistance<1e-9||Math.abs(deltaHeading)<1e-8?
      'straight':(deltaHeading>0?'left':'right');
     const innerSide=turnDirection==='straight'?null:
      (turnDirection==='left'?
       (state.gear===1?'left':'right'):
       (state.gear===1?'right':'left'));
     const outerSide=innerSide===null?null:(innerSide==='left'?'right':'left');
     const innerWheelDistance=innerSide==='left'?leftWheelDistance:
      (innerSide==='right'?rightWheelDistance:null);
     const outerWheelDistance=outerSide==='left'?leftWheelDistance:
      (outerSide==='right'?rightWheelDistance:null);
     return {
      axleIndex:axleIndex+1,centerDistance,leftWheelDistance,rightWheelDistance,
      turnDirection,innerSide,outerSide,innerWheelDistance,outerWheelDistance,
      outerMinusInner:innerWheelDistance===null?null:outerWheelDistance-innerWheelDistance
     };
    });
    segments.push({
     fromSample:index-1,toSample:index,gear:state.gear,gearChanged:state.gear!==previousState.gear,
     tractorReferenceDistance:routeDistance,
     trailerHeadingChange:deltaHeading,
     axles:axleSegments
    });
   }
   previousTracks=currentTracks;
   previousState=state;
  }
  return {
   samples:states.length,segments:segments.length,totalRouteDistance:routeDistance,
   axles,
   segments,
   distanceMethod:'sampled-centerline-chords-between-route-states',
   model:'single-equivalent-trailer-axle-group',
   limitations:[
    'Траектории колёс — выборки в узлах переданного маршрута; длина между ними измеряется хордами и не является точной непрерывной длиной.',
    'Все три оси жёстко привязаны к одному эквивалентному курсу полуприцепа; независимое руление, подвеска и деформация шин не моделируются.',
    'Внутренняя и внешняя стороны классифицируются по знаку изменения курса между соседними узлами и передаче; это геометрический индикатор среза, не измерение бокового скольжения.'
   ]
  };
 }

 return Object.freeze({norm,validateVehicleParams,derivative,rk4,hitchPoint,axlePoint,tractorAxlePoints,trailerAxlePoints,wheelFootprints,trailerAxleDiagnostics,trailerAxleScrubExposure,trailerWheelTrajectoryDiagnostics,vehiclePolygons,mirrorPolygons,vehicleCollisionParts});
});
