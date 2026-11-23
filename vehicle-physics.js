(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionPhysics=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';

 const norm=angle=>Math.atan2(Math.sin(angle),Math.cos(angle));

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

 return Object.freeze({norm,derivative,rk4,hitchPoint,axlePoint,trailerAxlePoints,trailerAxleDiagnostics,vehiclePolygons});
});
