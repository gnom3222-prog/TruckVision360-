(function(root,factory){
 'use strict';
 const physics=root&&root.TruckVisionPhysics?root.TruckVisionPhysics:
  (typeof require==='function'?require('./vehicle-physics.js'):null);
 const api=factory(physics);
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionPlanner=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(rootPhysics){
 'use strict';
 const norm=a=>Math.atan2(Math.sin(a),Math.cos(a));
 const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
 function goalErrors(q,goal){
  return {
   position:Math.hypot(goal.x-q.x,goal.y-q.y),
   headingError:Number.isFinite(goal.heading)?Math.abs(norm(q.psi-goal.heading)):null,
   articulation:Math.abs(norm(q.psi-q.psiT))
  };
 }
 function goalReached(q,goal,options={}){
  const errors=goalErrors(q,goal);
  const positionTolerance=options.positionTolerance??0.9;
  const headingTolerance=options.headingTolerance??12*Math.PI/180;
  const maxTerminalArticulation=options.maxTerminalArticulation??12*Math.PI/180;
  return errors.position<=positionTolerance&&
   (errors.headingError===null||errors.headingError<=headingTolerance)&&
   errors.articulation<=maxTerminalArticulation;
 }
 function polygons(q,p){
  if(!rootPhysics||typeof rootPhysics.vehiclePolygons!=='function')throw new Error('Shared vehicle geometry is unavailable.');
  return rootPhysics.vehiclePolygons(q,p);
 }
 function obstaclePoly(o){return[
  {x:o.x-o.w/2,y:o.y-o.h/2},{x:o.x+o.w/2,y:o.y-o.h/2},
  {x:o.x+o.w/2,y:o.y+o.h/2},{x:o.x-o.w/2,y:o.y+o.h/2}
 ];}
 function intersects(a,b){
  for(const poly of [a,b])for(let i=0;i<poly.length;i++){
   const n=poly[(i+1)%poly.length],c=poly[i],dx=n.x-c.x,dy=n.y-c.y,l=Math.hypot(dx,dy)||1;
   const ax=-dy/l,ay=dx/l,pa=a.map(v=>v.x*ax+v.y*ay),pb=b.map(v=>v.x*ax+v.y*ay);
   if(Math.max(...pa)<Math.min(...pb)||Math.max(...pb)<Math.min(...pa))return false;
  }
  return true;
 }
 function segDist(p,a,b){
  const dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy,t=l?clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l,0,1):0;
  return Math.hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
 }
 function polyDist(a,b){
  if(intersects(a,b))return 0;
  let d=Infinity;
  for(const p of a)for(let i=0;i<b.length;i++)d=Math.min(d,segDist(p,b[i],b[(i+1)%b.length]));
  for(const p of b)for(let i=0;i<a.length;i++)d=Math.min(d,segDist(p,a[i],a[(i+1)%a.length]));
  return d;
 }
 function polygonBounds(poly){
  let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
  for(const p of poly){minX=Math.min(minX,p.x);minY=Math.min(minY,p.y);maxX=Math.max(maxX,p.x);maxY=Math.max(maxY,p.y);}
  return {minX,minY,maxX,maxY};
 }
 function aabbDistance(a,b){
  const dx=Math.max(0,a.minX-b.maxX,b.minX-a.maxX);
  const dy=Math.max(0,a.minY-b.maxY,b.minY-a.maxY);
  return Math.hypot(dx,dy);
 }
 function createClearanceChecker(obstacles,p){
  const prepared=obstacles.map(o=>{
   const poly=obstaclePoly(o);
   return {poly,bounds:polygonBounds(poly),name:o.name||'препятствие'};
  });
  return function(q){
   const bodies=polygons(q,p),bodyBounds=bodies.map(polygonBounds);
   let min=Infinity,hit=null;
   for(const obstacle of prepared){
    for(let i=0;i<bodies.length;i++){
     if(aabbDistance(bodyBounds[i],obstacle.bounds)>=min)continue;
     const d=polyDist(bodies[i],obstacle.poly);
     if(d<min){min=d;hit=obstacle.name;}
    }
   }
   return {distance:min,hit};
  };
 }
 function sweptDisplacementPerMeter(steer,p){
  if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2)throw new Error('Steering angle is outside the valid range.');
  const wheelbase=Number(p.wheelbase),hitchOffset=Number(p.hitchOffset);
  const kingpinAxles=Number(p.kingpinAxles),trailerLength=Number(p.trailerLength),trailerWidth=Number(p.trailerWidth);
  if(![wheelbase,hitchOffset,kingpinAxles,trailerLength,trailerWidth].every(Number.isFinite)||
     wheelbase<=0||hitchOffset<0||kingpinAxles<=0||trailerLength<=0||trailerWidth<=0){
   throw new Error('Vehicle geometry is incomplete or invalid for swept-clearance checking.');
  }
  const tractorYawPerMeter=Math.abs(Math.tan(steer)/wheelbase);
  const tractorRadius=Math.hypot(5,2.55/2);
  const cabRadiusFromRearAxle=2.1+Math.hypot(1.35,2.25/2);
  const trailerRadius=Math.hypot(Math.max(1.5,Math.abs(trailerLength-1.5)),trailerWidth/2);
  const trailerYawPerMeter=(1+hitchOffset*tractorYawPerMeter)/kingpinAxles;
  return Math.max(
   1+tractorRadius*tractorYawPerMeter,
   1+cabRadiusFromRearAxle*tractorYawPerMeter,
   1+hitchOffset*tractorYawPerMeter+trailerRadius*trailerYawPerMeter
  );
 }
 function clearance(q,obstacles,p){return createClearanceChecker(obstacles,p)(q);}
 function push(heap,n){
  let i=heap.length;heap.push(n);
  while(i){const p=(i-1)>>1;if(heap[p].f<=n.f)break;heap[i]=heap[p];i=p;heap[i]=n;}
 }
 function pop(heap){
  const top=heap[0],last=heap.pop();
  if(heap.length){let i=0;heap[0]=last;for(;;){let l=i*2+1,r=l+1,m=i;if(l<heap.length&&heap[l].f<heap[m].f)m=l;if(r<heap.length&&heap[r].f<heap[m].f)m=r;if(m===i)break;[heap[i],heap[m]]=[heap[m],heap[i]];i=m;}}
  return top;
 }
 function plan(options){
  const {start,goal,params,obstacles}=options;
  const maxExpansions=options.maxExpansions||24000;
  const bounds=options.bounds||{minX:-20,maxX:42,minY:-20,maxY:20};
  const step=0.20,angleStep=Math.PI/60,primitive=0.6,ds=0.05;
  const maxSteer=options.maxSteer||Math.PI/7;
  const minClearance=Number.isFinite(options.minClearance)?Math.max(0,options.minClearance):0.30;
  const positionTolerance=options.positionTolerance??0.9;
  const headingTolerance=options.headingTolerance??12*Math.PI/180;
  const maxTerminalArticulation=options.maxTerminalArticulation??12*Math.PI/180;
  const goalOptions={positionTolerance,headingTolerance,maxTerminalArticulation};
  const checkClearance=createClearanceChecker(obstacles,params);
  const key=q=>[
   Math.round(q.x/step),Math.round(q.y/step),
   Math.round(norm(q.psi)/angleStep),Math.round(norm(q.psiT)/angleStep)
  ].join(':');
  const h=q=>{
   const e=goalErrors(q,goal);
   return e.position+0.45*(e.headingError??0)+0.35*e.articulation;
  };
  const startClearance=checkClearance(start).distance;
  if(startClearance===0)return {ok:false,reason:'start-collision',expanded:0};
  if(Number.isFinite(startClearance)&&startClearance<minClearance-1e-9){
   return {ok:false,reason:'start-clearance-insufficient',expanded:0,startClearance,requiredClearance:minClearance};
  }
  const requiredClearance=minClearance;
  const initial={q:{...start},g:0,f:h(start),parent:null,gear:1,steer:0,samples:[],clearance:startClearance,sweptLowerBound:startClearance};
  const open=[];push(open,initial);
  const best=new Map([[key(start),0]]);
  let expanded=0,finish=null,closest=initial;
  const quality=q=>{
   const e=goalErrors(q,goal);
   return e.position+0.5*(e.headingError??0)+0.5*Math.max(0,e.articulation-maxTerminalArticulation);
  };
  let closestQuality=quality(start);
  const steering=[-maxSteer,-0.24,-0.12,0,0.12,0.24,maxSteer];
  const controls=[];
  for(const gear of [-1,1])for(const steer of steering)controls.push([gear,steer]);
  while(open.length&&expanded<maxExpansions){
   const cur=pop(open),ck=key(cur.q);
   if(cur.g>(best.get(ck)??Infinity)+1e-8)continue;
   expanded++;
   const errors=goalErrors(cur.q,goal);
   const dist=errors.position;
   const currentQuality=quality(cur.q);
   if(currentQuality<closestQuality){closest=cur;closestQuality=currentQuality;}
   if(options.onProgress&&(expanded===1||expanded%250===0)){
    try{options.onProgress({expanded,maxExpansions,goalDistance:dist,headingError:errors.headingError,articulationAngle:errors.articulation,closest:{x:closest.q.x,y:closest.q.y}});}catch(_){}
   }
   if(goalReached(cur.q,goal,goalOptions)){finish=cur;break;}
   for(const [gear,steer] of controls){
    let q={...cur.q},valid=true,minD=Infinity,samples=[];
    let currentClearance=cur.clearance,sweptLowerBound=cur.sweptLowerBound,travelled=0;
    const displacementRate=sweptDisplacementPerMeter(steer,params);
    while(travelled<primitive-1e-9){
     const remaining=primitive-travelled;
     let travel=Math.min(ds,remaining);
     if(Number.isFinite(currentClearance)){
      const margin=currentClearance-requiredClearance;
      if(margin<=1e-9){valid=false;break;}
      travel=Math.min(travel,0.8*margin/displacementRate);
     }
     if(travel<1e-4){valid=false;break;}
     const next=options.integrate(q,gear*0.7,steer,travel/0.7,params);
     if(next.x<bounds.minX||next.x>bounds.maxX||next.y<bounds.minY||next.y>bounds.maxY||
        Math.abs(norm(next.psi-next.psiT))>Math.PI*0.92){valid=false;break;}
     const c=checkClearance(next);
     if(c.distance<requiredClearance-1e-9){valid=false;break;}
     const segmentLowerBound=Number.isFinite(currentClearance)?currentClearance-displacementRate*travel:Infinity;
     sweptLowerBound=Math.min(sweptLowerBound,segmentLowerBound);
     minD=Math.min(minD,c.distance);
     q=next;
     currentClearance=c.distance;
     travelled+=travel;
     samples.push({x:q.x,y:q.y,psi:q.psi,psiT:q.psiT,gear,steer});
    }
    if(!valid)continue;
    const nk=key(q);
    const gearPenalty=gear<0?0.55:0;
    const steerPenalty=Math.abs(steer)*0.08+(cur.steer!==steer?0.10:0);
    const ng=cur.g+primitive+gearPenalty+steerPenalty;
    if(ng>=(best.get(nk)??Infinity))continue;
    best.set(nk,ng);
    push(open,{q,g:ng,f:ng+h(q),parent:cur,gear,steer,minClearance:minD,samples,clearance:currentClearance,sweptLowerBound});
   }
  }
  if(!finish){
   const errors=goalErrors(closest.q,goal);
   return {ok:false,reason:'no-path',expanded,
    closest:{x:closest.q.x,y:closest.q.y,psi:closest.q.psi,psiT:closest.q.psiT},
    goalDistance:errors.position,headingError:errors.headingError,articulationAngle:errors.articulation};
  }
  const chain=[];for(let n=finish;n&&n.parent;n=n.parent)chain.push(n);
  chain.reverse();
  const states=[{x:start.x,y:start.y,psi:start.psi,psiT:start.psiT,gear:1,steer:0}];
  for(const n of chain)states.push(...n.samples);
  const pathMinimum=Math.min(startClearance,...chain.map(n=>n.minClearance));
  return {ok:true,states,expanded,length:finish.g,minimumClearance:pathMinimum,minimumClearanceLowerBound:finish.sweptLowerBound,requiredClearance,terminalErrors:goalErrors(finish.q,goal)};
 }
 function recommendStart(states,options={}){
  if(!Array.isArray(states)||states.length<2)return {ok:false,reason:'route-too-short'};
  const steeringThreshold=Number.isFinite(options.steeringThreshold)&&options.steeringThreshold>0?options.steeringThreshold:0.08;
  let distance=0;
  const first=states[1];
  const initialGear=(first.gear??1)<0?-1:1;
  let firstTurn=null;
  let firstTurnDistance=null;
  let firstTurnGear=null;
  for(let i=1;i<states.length;i++){
   const previous=states[i-1],current=states[i];
   distance+=Math.hypot(current.x-previous.x,current.y-previous.y);
   const steer=Number(current.steer)||0;
   if(Math.abs(steer)>=steeringThreshold){
    firstTurn=steer>0?'left':'right';
    firstTurnDistance=distance;
    firstTurnGear=(current.gear??initialGear)<0?-1:1;
    break;
   }
  }
  return {
   ok:true,
   initialGear,
   initialSteer:Number(first.steer)||0,
   firstTurn,
   firstTurnDistance,
   firstTurnGear,
   firstTurnAfterGearChange:firstTurnGear!==null&&firstTurnGear!==initialGear
  };
 }

 return Object.freeze({plan,clearance,createClearanceChecker,sweptDisplacementPerMeter,polygons,goalErrors,goalReached,recommendStart});
});
