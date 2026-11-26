'use strict';
importScripts('./vehicle-physics.js?v=20261126-14','./articulated-planner.js?v=20261126-8');

self.addEventListener('message',event=>{
 const data=event.data||{};
 const id=data.id;
 try{
  if(!self.TruckVisionPhysics||!self.TruckVisionPlanner)throw new Error('Модули физики или планировщика не загрузились.');
  const options=data.options||{};
  const result=self.TruckVisionPlanner.planAlternativesWithVehicleParameterBounds({
   ...options,
   integrate:(q,v,delta,dt,params)=>self.TruckVisionPhysics.rk4(q,v,delta,dt,params),
   onProgress:progress=>self.postMessage({id,type:'progress',progress})
  });
  self.postMessage({id,type:'result',result});
 }catch(error){
  self.postMessage({id,type:'error',message:error&&error.message?error.message:String(error)});
 }
});
