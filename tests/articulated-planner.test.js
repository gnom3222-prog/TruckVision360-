'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');
const params={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55};
const integrate=(q,v,steer,dt,p)=>physics.rk4(q,v,steer,dt,p);
test('articulated planner reaches a clear goal in open space',()=>{
 const result=planner.plan({start:{x:0,y:0,psi:0,psiT:0},goal:{x:7,y:0},params,obstacles:[],integrate,maxExpansions:5000});
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.states.length>10);
 assert.ok(Math.hypot(result.states.at(-1).x-7,result.states.at(-1).y)<0.9);
});
test('articulated planner satisfies the requested terminal tractor heading and trailer alignment',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:2,y:0,heading:0},
  params,obstacles:[],integrate,
  maxExpansions:1000,
  positionTolerance:0.9,
  headingTolerance:5*Math.PI/180,
  maxTerminalArticulation:5*Math.PI/180
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.terminalErrors.position<=0.9);
 assert.ok(result.terminalErrors.headingError<=5*Math.PI/180);
 assert.ok(result.terminalErrors.articulation<=5*Math.PI/180);
});

test('articulated planner does not accept a position-only match with the wrong final heading',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:0.5,y:0,heading:Math.PI},
  params,obstacles:[],integrate,
  maxExpansions:1,
  headingTolerance:5*Math.PI/180,
  maxTerminalArticulation:5*Math.PI/180
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'no-path');
 assert.ok(result.headingError>Math.PI-1e-9);
});

test('articulated planner routes the complete trailer around an obstacle',()=>{
 const obstacles=[{x:20,y:0,w:1.2,h:1.2,name:'test pallet'}];
 const result=planner.plan({start:{x:0,y:0,psi:0,psiT:0},goal:{x:40,y:0},params,obstacles,integrate,maxExpansions:24000,bounds:{minX:-18,maxX:50,minY:-16,maxY:16}});
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.minimumClearanceLowerBound>=0.30-1e-6,'swept lower bound must preserve the requested clearance');
 assert.ok(result.minimumClearance>=result.minimumClearanceLowerBound-1e-6,'sampled clearance must not be below its conservative lower bound');
 for(const q of result.states.slice(1)){
  assert.ok(planner.clearance(q,obstacles,params).distance>=0.30-1e-6,'vehicle footprint must remain clear');
 }
});

test('articulated planner can select reverse gear and reports progress',()=>{
 const progress=[];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:-3,y:0},
  params,
  obstacles:[],
  integrate,
  maxExpansions:5000,
  onProgress:p=>progress.push(p.expanded)
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.states.some((q,i)=>i>0&&q.gear===-1),'reverse motion should be represented in the route');
 assert.ok(progress.length>0,'planner should report progress');
 assert.ok(progress.every((n,i)=>i===0||n>progress[i-1]),'progress counters should increase');
});
 
test('planner and simulator use exactly the same articulated collision geometry',()=>{
 const q={x:3.2,y:-1.7,psi:0.28,psiT:-0.19};
 assert.deepEqual(planner.polygons(q,params),physics.vehiclePolygons(q,params));
 const hitch=physics.hitchPoint(q,params);
 const axle=physics.axlePoint(q,params);
 assert.ok(Math.hypot(hitch.x-(q.x-params.hitchOffset*Math.cos(q.psi)),hitch.y-(q.y-params.hitchOffset*Math.sin(q.psi)))<1e-10);
 assert.ok(Math.hypot(axle.x-(hitch.x-params.kingpinAxles*Math.cos(q.psiT)),axle.y-(hitch.y-params.kingpinAxles*Math.sin(q.psiT)))<1e-10);
});

test('prepared clearance checker matches direct collision-distance queries',()=>{
 const q={x:2.3,y:1.1,psi:-0.17,psiT:0.09};
 const obstacles=[
  {x:7,y:3,w:1.2,h:1.2,name:'A'},
  {x:-4,y:-2,w:0.8,h:2.0,name:'B'}
 ];
 const check=planner.createClearanceChecker(obstacles,params);
 assert.deepEqual(check(q),planner.clearance(q,obstacles,params));
});

test('clearance checker identifies whether the tractor or trailer is nearest to an obstacle',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const rear=planner.clearance(q,[{x:-13.5,y:0,w:1,h:1,name:'rear pallet'}],params);
 assert.equal(rear.vehiclePart,'полуприцеп');
 assert.equal(rear.obstacle,'rear pallet');
 assert.equal(rear.bodyIndex,2);
 const front=planner.clearance(q,[{x:6.5,y:0,w:1,h:1,name:'front pallet'}],params);
 assert.equal(front.vehiclePart,'тягач');
 assert.equal(front.obstacle,'front pallet');
 assert.equal(front.bodyIndex,0);
});

test('articulated planner passes through a two-pallet clearance gate',()=>{
 const obstacles=[
  {x:12,y:3,w:1.2,h:2,name:'upper pallet'},
  {x:12,y:-3,w:1.2,h:2,name:'lower pallet'}
 ];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:25,y:0},
  params,obstacles,integrate,
  maxExpansions:5000,
  minClearance:0.30,
  bounds:{minX:-18,maxX:32,minY:-12,maxY:12}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.minimumClearanceLowerBound>=0.30-1e-6,'continuous-step bound must preserve the gate clearance');
 assert.ok(result.minimumClearanceDetail,'route should expose the nearest sampled clearance detail');
 assert.ok(result.minimumClearanceDetail.vehiclePart);
 assert.ok(result.minimumClearanceDetail.obstacle);
 assert.ok(result.minimumClearanceDetail.state);
 assert.ok(result.sweptClearanceDetail,'route should identify the segment responsible for its lower bound');
 assert.ok(Number.isFinite(result.sweptClearanceDetail.lowerBound));
 for(const q of result.states.slice(1)){
  assert.ok(planner.clearance(q,obstacles,params).distance>=0.30-1e-6,'entire articulated footprint must clear both pallets');
 }
});

test('articulated planner rejects an initial tractor or trailer collision',()=>{
 const obstacles=[{x:0,y:0,w:1,h:1,name:'blocking pallet'}];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:8,y:0},
  params,obstacles,integrate,
  maxExpansions:1000
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'start-collision');
});


test('swept displacement bound covers every tractor, cab and trailer vertex in forward and reverse steps',()=>{
 const cases=[
  {q:{x:0,y:0,psi:0.2,psiT:-0.4},steer:Math.PI/7,gear:1},
  {q:{x:1,y:-2,psi:-0.3,psiT:0.55},steer:-Math.PI/7,gear:-1},
  {q:{x:-3,y:1,psi:1.1,psiT:-0.1},steer:0.24,gear:1}
 ];
 for(const item of cases){
  const globalRate=planner.sweptDisplacementPerMeter(item.steer,params);
  const rate=planner.sweptDisplacementPerMeter(item.steer,params,item.q,0.05);
  assert.ok(rate<=globalRate+1e-9,'local articulation bound should not exceed the all-angle bound');
  const initial=planner.polygons(item.q,params);
  for(let sample=1;sample<=8;sample++){
   const distance=0.05*sample/8;
   const next=physics.rk4(item.q,item.gear*0.7,item.steer,distance/0.7,params);
   const moved=planner.polygons(next,params);
   for(let body=0;body<initial.length;body++)for(let vertex=0;vertex<initial[body].length;vertex++){
    const a=initial[body][vertex],b=moved[body][vertex];
    const displacement=Math.hypot(b.x-a.x,b.y-a.y);
    assert.ok(displacement<=rate*distance+1e-8,
     'swept bound must cover every body vertex between sampled poses');
   }
  }
 }
});

test('swept displacement bound rejects missing or incomplete vehicle geometry',()=>{
 assert.throws(()=>planner.sweptDisplacementPerMeter(0,null),/missing/i);
 assert.throws(()=>planner.sweptDisplacementPerMeter(0,{...params,kingpinAxles:0}),/invalid/i);
 assert.throws(()=>planner.sweptDisplacementPerMeter(0,params,null,0),/positive/i);
});

test('planner rejects a start position already inside the requested clearance reserve',()=>{
 const obstacles=[{x:6,y:0,w:1,h:1,name:'nearby pallet'}];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:8,y:0},
  params,obstacles,integrate,
  minClearance:0.65,
  maxExpansions:1000
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'start-clearance-insufficient');
 assert.ok(result.startClearance<result.requiredClearance);
});

test('planner adaptively shortens reverse steps near the requested clearance reserve',()=>{
 const obstacles=[{x:5.82,y:0,w:1,h:1,name:'front pallet'}];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:-2,y:0},
  params,obstacles,integrate,
  minClearance:0.30,
  maxExpansions:5000,
  bounds:{minX:-18,maxX:18,minY:-12,maxY:12}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(Math.hypot(result.states[1].x-result.states[0].x,result.states[1].y-result.states[0].y)<0.05,
  'the first step should be shorter than the normal 0.05 m step near the clearance reserve');
 assert.ok(result.minimumClearanceLowerBound>=0.30-1e-6);
 assert.ok(result.states.some((q,i)=>i>0&&q.gear===-1),'the route should use reverse to move away from the obstacle');
});

test('start recommendation identifies the first steering side after a straight lead-in',()=>{
 const states=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.2,y:0,gear:1,steer:0},
  {x:0.4,y:0,gear:1,steer:0.12},
  {x:0.6,y:0.02,gear:1,steer:0.12}
 ];
 const result=planner.recommendStart(states);
 assert.equal(result.ok,true);
 assert.equal(result.initialGear,1);
 assert.equal(result.firstTurn,'left');
 assert.equal(result.firstTurnGear,1);
 assert.equal(result.tractorYawDirection,'left');
 assert.equal(result.firstTurnAfterGearChange,false);
 assert.ok(Math.abs(result.firstTurnDistance-0.4)<1e-9);
});

test('start recommendation keeps steering-wheel direction explicit during reverse',()=>{
 const result=planner.recommendStart([
  {x:0,y:0,gear:1,steer:0},
  {x:-0.1,y:0,gear:-1,steer:-0.16},
  {x:-0.2,y:-0.01,gear:-1,steer:-0.16}
 ]);
 assert.equal(result.initialGear,-1);
 assert.equal(result.firstTurn,'right');
 assert.equal(result.firstTurnGear,-1);
 assert.equal(result.tractorYawDirection,'left');
 assert.equal(result.firstTurnAfterGearChange,false);
});

test('start recommendation reports a turn that begins after a gear change',()=>{
 const result=planner.recommendStart([
  {x:0,y:0,gear:1,steer:0},
  {x:0.3,y:0,gear:1,steer:0},
  {x:0.3,y:0,gear:-1,steer:0},
  {x:0.2,y:0,gear:-1,steer:0.12}
 ]);
 assert.equal(result.initialGear,1);
 assert.equal(result.firstTurn,'left');
 assert.equal(result.firstTurnGear,-1);
 assert.equal(result.tractorYawDirection,'right');
 assert.equal(result.firstTurnAfterGearChange,true);
 assert.ok(Math.abs(result.firstTurnDistance-0.4)<1e-9);
});

test('start recommendation reports straight travel when no steering input is planned',()=>{
 const result=planner.recommendStart([
  {x:0,y:0,gear:1,steer:0},
  {x:0.5,y:0,gear:1,steer:0},
  {x:1,y:0,gear:1,steer:0}
 ]);
 assert.equal(result.initialGear,1);
 assert.equal(result.firstTurn,null);
 assert.equal(result.firstTurnDistance,null);
});

test('start recommendation rejects a route without movement samples',()=>{
 assert.equal(planner.recommendStart([{x:0,y:0}]).reason,'route-too-short');
});


test('start recommendation ignores zero and invalid steering thresholds',()=>{
 const states=[{x:0,y:0,gear:1,steer:0},{x:0.5,y:0,gear:1,steer:0}];
 assert.equal(planner.recommendStart(states,{steeringThreshold:0}).firstTurn,null);
 assert.equal(planner.recommendStart(states,{steeringThreshold:-1}).firstTurn,null);
});

test('planner rejects a three-axle group that extends beyond the trailer body before search',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:5,y:0},
  params:{...params,trailerAxleSpacing:5},obstacles:[],integrate,
  maxExpansions:100
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'invalid-input');
 assert.equal(result.expanded,0);
 assert.ok(result.errors.some(message=>message.includes('trailerAxleSpacing')));
});

test('planner rejects invalid vehicle geometry before collision checking or search',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:5,y:0},
  params:{...params,trailerWidth:NaN},obstacles:[],integrate,
  maxExpansions:1000
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'invalid-input');
 assert.equal(result.expanded,0);
 assert.ok(result.errors.some(message=>message.includes('trailerWidth')));
});

test('planner rejects malformed start states and zero-size obstacles',()=>{
 const badStart=planner.plan({
  start:{x:0,y:0,psi:Infinity,psiT:0},goal:{x:5,y:0},
  params,obstacles:[],integrate,maxExpansions:1000
 });
 assert.equal(badStart.reason,'invalid-input');
 assert.ok(badStart.errors.some(message=>message.includes('Начальное состояние')));
 const badObstacle=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:5,y:0},
  params,obstacles:[{x:2,y:0,w:0,h:1}],integrate,maxExpansions:1000
 });
 assert.equal(badObstacle.reason,'invalid-input');
 assert.ok(badObstacle.errors.some(message=>message.includes('положительные ширину/высоту')));
});

test('direct clearance checks fail closed for malformed obstacles',()=>{
 assert.throws(()=>planner.createClearanceChecker([{x:1,y:2,w:1,h:0}],params),/Некорректные препятствия/);
});


test('swept displacement bound remains conservative across articulation extrema, angle wrap and both gears',()=>{
 const betaValues=[
  -2.8,
  -Math.PI/2-0.02,
  -Math.PI/2,
  -Math.PI/2+0.02,
  -0.02,
  0.02,
  Math.PI/2-0.02,
  Math.PI/2,
  Math.PI/2+0.02,
  2.8
 ];
 const steeringValues=[-Math.PI/7,-0.24,0,0.24,Math.PI/7];
 const maxDistance=0.05;
 for(const beta of betaValues)for(const steer of steeringValues)for(const gear of [-1,1]){
  const q={x:0.7,y:-1.2,psi:3.05,psiT:3.05-beta};
  const rate=planner.sweptDisplacementPerMeter(steer,params,q,maxDistance);
  const initial=planner.polygons(q,params);
  for(let sample=1;sample<=40;sample++){
   const distance=maxDistance*sample/40;
   const next=physics.rk4(q,gear*0.7,steer,distance/0.7,params);
   const moved=planner.polygons(next,params);
   for(let body=0;body<initial.length;body++)for(let vertex=0;vertex<initial[body].length;vertex++){
    const a=initial[body][vertex],b=moved[body][vertex];
    const displacement=Math.hypot(b.x-a.x,b.y-a.y);
    assert.ok(displacement<=rate*distance+1e-8,
     `swept bound underestimates body ${body+1} vertex ${vertex+1} at beta=${beta}, steer=${steer}, gear=${gear}, distance=${distance}`);
   }
  }
 }
});

test('swept bound uses the exact axle-centered trailer corner radius across long kingpin offsets',()=>{
 const configurations=[
  params,
  {...params,kingpinAxles:25,trailerLength:30,trailerWidth:2.55},
  {...params,kingpinAxles:27,trailerLength:30,trailerWidth:5}
 ];
 const betaValues=[-2.9,-Math.PI/2,-0.3,0.2,Math.PI/2,2.9];
 const steeringValues=[-Math.PI/7,0,Math.PI/7];
 for(const p of configurations){
  for(const steer of steeringValues){
   const yaw=Math.abs(Math.tan(steer)/p.wheelbase);
   const a=p.hitchOffset*yaw;
   const front=p.kingpinAxles+1.5;
   const rear=p.kingpinAxles-(p.trailerLength-1.5);
   let trailerRate=0;
   for(const coupling of [-a,a]){
    for(const longitudinal of [front,rear]){
     for(const lateral of [-p.trailerWidth/2,p.trailerWidth/2]){
      const m11=1+coupling*lateral/p.kingpinAxles;
      const m12=coupling-lateral/p.kingpinAxles;
      const m21=-coupling*longitudinal/p.kingpinAxles;
      const m22=longitudinal/p.kingpinAxles;
      const c1=m11*m11+m21*m21;
      const c2=m12*m12+m22*m22;
      const cross=m11*m12+m21*m22;
      const lambda=0.5*(c1+c2+Math.hypot(c1-c2,2*cross));
      trailerRate=Math.max(trailerRate,Math.sqrt(Math.max(0,lambda)));
     }
    }
   }
   const expected=Math.max(
    1+Math.hypot(5,2.55/2)*yaw,
    1+(2.1+Math.hypot(1.35,2.25/2))*yaw,
    trailerRate
   );
   for(const beta of betaValues){
    for(const gear of [-1,1]){
     const q={x:0.7,y:-1.2,psi:3.05,psiT:3.05-beta};
     const rate=planner.sweptDisplacementPerMeter(steer,p,q,0.05);
     assert.ok(Math.abs(rate-expected)<1e-10,
      'swept rate should equal the maximum spectral bound for all accepted body corners');
     const initial=planner.polygons(q,p);
     for(let sample=1;sample<=20;sample++){
      const distance=0.05*sample/20;
      const next=physics.rk4(q,gear*0.7,steer,distance/0.7,p);
      const moved=planner.polygons(next,p);
      for(let body=0;body<initial.length;body++)for(let vertex=0;vertex<initial[body].length;vertex++){
       const from=initial[body][vertex],to=moved[body][vertex];
       const displacement=Math.hypot(to.x-from.x,to.y-from.y);
       assert.ok(displacement<=rate*distance+1e-7,
        `swept bound underestimates body ${body+1} vertex ${vertex+1} for kingpinAxles=${p.kingpinAxles}, beta=${beta}, steer=${steer}, gear=${gear}`);
      }
     }
    }
   }
  }
 }
});


test('articulation exposure locates the peak angle and cumulative route distance',()=>{
 const result=planner.articulationExposure([
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0},
  {x:1,y:0,psi:0.2,psiT:0.1,gear:1,steer:0.1},
  {x:3,y:0,psi:Math.PI/3,psiT:0,gear:-1,steer:-0.12},
  {x:4,y:0,psi:0.3,psiT:0.2,gear:-1,steer:0}
 ]);
 assert.equal(result.stateIndex,2);
 assert.equal(result.segmentIndex,1);
 assert.equal(result.gear,-1);
 assert.ok(Math.abs(result.maxAngleDegrees-60)<1e-9);
 assert.ok(Math.abs(result.distanceAlongRoute-3)<1e-9);
 assert.deepEqual(result.state,{x:3,y:0,psi:Math.PI/3,psiT:0,gear:-1,steer:-0.12});
});

test('articulation exposure uses wrapped angular difference near plus and minus pi',()=>{
 const result=planner.articulationExposure([
  {x:0,y:0,psi:Math.PI-0.01,psiT:-Math.PI+0.02,gear:1}
 ]);
 assert.ok(Math.abs(result.maxAngleRadians-0.03)<1e-9);
 assert.ok(Math.abs(result.maxAngleDegrees-0.03*180/Math.PI)<1e-9);
});

test('articulation exposure rejects malformed route states',()=>{
 assert.throws(()=>planner.articulationExposure([]),/Route states/);
 assert.throws(()=>planner.articulationExposure([{x:0,y:0,psi:NaN,psiT:0}]),/invalid coordinates/);
});


test('start recommendation mirrors steering side and tractor yaw on a mirrored route',()=>{
 const route=[
  {x:0,y:0,psi:0,psiT:0,gear:-1,steer:0},
  {x:-0.4,y:0.02,psi:-0.04,psiT:-0.01,gear:-1,steer:0.12},
  {x:-0.8,y:0.10,psi:-0.08,psiT:-0.02,gear:-1,steer:0.12}
 ];
 const mirrored=route.map(state=>({
  ...state,y:-state.y,psi:-state.psi,psiT:-state.psiT,steer:-state.steer
 }));
 const originalAdvice=planner.recommendStart(route);
 const mirroredAdvice=planner.recommendStart(mirrored);
 assert.equal(originalAdvice.firstTurn,'left');
 assert.equal(mirroredAdvice.firstTurn,'right');
 assert.equal(originalAdvice.tractorYawDirection,'right');
 assert.equal(mirroredAdvice.tractorYawDirection,'left');
 assert.equal(originalAdvice.initialGear,mirroredAdvice.initialGear);
 assert.ok(Math.abs(originalAdvice.firstTurnDistance-mirroredAdvice.firstTurnDistance)<1e-12);
});

test('near-equal candidate routes with opposite first turns produce no preferred side',()=>{
 const left=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.5,y:0.01,gear:1,steer:0.12}
 ];
 const right=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.5,y:-0.01,gear:1,steer:-0.12}
 ];
 const result=planner.compareStartCandidates([
  {states:left,cost:10,label:'влево'},
  {states:right,cost:10.1,label:'вправо'}
 ],{costTolerance:0.25});
 assert.equal(result.ok,true);
 assert.equal(result.status,'ambiguous');
 assert.equal(result.recommendation,null);
 assert.ok(result.ambiguousFields.includes('firstTurn'));
 assert.equal(result.candidatesCompared,2);
});

test('candidate comparison selects a side only when the alternative is outside the cost tolerance',()=>{
 const left=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.5,y:0.01,gear:1,steer:0.12}
 ];
 const right=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.5,y:-0.01,gear:1,steer:-0.12}
 ];
 const result=planner.compareStartCandidates([
  {states:left,cost:10,label:'влево'},
  {states:right,cost:10.5,label:'вправо'}
 ],{costTolerance:0.25});
 assert.equal(result.ok,true);
 assert.equal(result.status,'preferred');
 assert.equal(result.recommendation.firstTurn,'left');
 assert.deepEqual(result.ambiguousFields,[]);
});

test('candidate comparison labels a single route as unverified against alternatives',()=>{
 const states=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.5,y:0,gear:1,steer:0.12}
 ];
 const result=planner.compareStartCandidates([{states,cost:5}]);
 assert.equal(result.ok,true);
 assert.equal(result.status,'single-candidate');
 assert.equal(result.recommendation.firstTurn,'left');
 assert.equal(result.candidatesCompared,1);
});

test('candidate comparison rejects invalid candidate costs and routes',()=>{
 const states=[{x:0,y:0,gear:1,steer:0},{x:0.5,y:0,gear:1,steer:0.12}];
 assert.equal(planner.compareStartCandidates([{states,cost:NaN}]).reason,'invalid-candidate-cost');
 assert.equal(planner.compareStartCandidates([{states:[{x:0,y:0}],cost:1}]).reason,'invalid-candidate-route');
});


test('planner enforces a constrained initial gear and steering side',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.12,0.05/0.7,params);
 const result=planner.plan({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,maxExpansions:300,
  positionTolerance:0.001,headingTolerance:0.001,
  maxTerminalArticulation:0.5,
  firstActionConstraint:{initialGear:1,firstTurn:'left'}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 const advice=planner.recommendStart(result.states);
 assert.equal(advice.initialGear,1);
 assert.equal(advice.firstTurn,'left');
});

test('planner rejects malformed first-action constraints',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:3,y:0},
  params,obstacles:[],integrate,maxExpansions:100,
  firstActionConstraint:{initialGear:0,firstTurn:'straight'}
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'invalid-input');
 assert.ok(result.errors.some(message=>message.includes('Ограничение первого действия')));
});

test('multi-start planner searches a free route and four constrained first actions',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.12,0.05/0.7,params);
 const result=planner.planAlternatives({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  maxExpansions:1200,alternativeMaxExpansions:1200,
  positionTolerance:0.02,headingTolerance:0.01,
  maxTerminalArticulation:0.5
 });
 assert.equal(result.candidatesAttempted,5);
 assert.ok(result.candidatesFound>=1,JSON.stringify(result));
 assert.ok(result.candidateSummaries.some(candidate=>candidate.label==='вперёд, первый поворот влево'),
  'the matching constrained forward-left route should be found');
 assert.ok(result.candidateSummaries.every(candidate=>Number.isFinite(candidate.cost)));
 if(result.ok){
  assert.ok(['single-candidate','preferred'].includes(result.status));
  assert.ok(result.states.length>1);
  assert.ok(result.recommendation);
 }else{
  assert.equal(result.reason,'ambiguous-start');
  assert.equal(result.comparison.recommendation,null);
 }
});

test('multi-start planner rejects an invalid per-candidate expansion budget',()=>{
 const result=planner.planAlternatives({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:2,y:0},
  params,obstacles:[],integrate,alternativeMaxExpansions:0
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'invalid-input');
});


test('recommendation sensitivity detects a changed initial action across clearance scenarios',()=>{
 const nominal={
  ok:true,recommendation:{initialGear:1,firstTurn:'left',firstTurnGear:1,tractorYawDirection:'left'}
 };
 const reserve={
  ok:true,recommendation:{initialGear:-1,firstTurn:'right',firstTurnGear:-1,tractorYawDirection:'left'}
 };
 const result=planner.compareRecommendationSensitivity(nominal,reserve);
 assert.equal(result.ok,true);
 assert.equal(result.status,'changed');
 assert.equal(result.stable,false);
 assert.ok(result.changedFields.includes('initialGear'));
 assert.ok(result.changedFields.includes('firstTurn'));
});
 
test('recommendation sensitivity reports a stable action without claiming global optimality',()=>{
 const action={initialGear:-1,firstTurn:'right',firstTurnGear:-1,tractorYawDirection:'left'};
 const result=planner.compareRecommendationSensitivity(
  {ok:true,recommendation:action,status:'single-candidate'},
  {ok:true,recommendation:{...action},status:'preferred'}
 );
 assert.equal(result.ok,true);
 assert.equal(result.status,'stable');
 assert.equal(result.stable,true);
 assert.deepEqual(result.changedFields,[]);
});
 
test('uncertainty-aware alternatives add twice the per-side reserve to required clearance',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.12,0.05/0.7,params);
 const result=planner.planAlternativesWithReserve({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.02,headingTolerance:0.01,
  maxTerminalArticulation:0.5,
  minClearance:0.30,measurementUncertainty:0.10
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.requiredClearance,0.50);
 assert.equal(result.sensitivity.status,'stable-start-action');
 assert.equal(result.sensitivity.nominalRequiredClearance,0.30);
 assert.equal(result.sensitivity.reserveRequiredClearance,0.50);
 assert.equal(result.sensitivity.comparison.stable,true);
});
 
test('uncertainty-aware alternatives reject a route that only meets the nominal clearance',()=>{
 const obstacle={x:2,y:2,w:1,h:0.5,name:'near-side wall'};
 const start={x:0,y:0,psi:0,psiT:0};
 const initialClearance=planner.clearance(start,[obstacle],params).distance;
 assert.ok(initialClearance>0.30&&initialClearance<0.50,
  'test obstacle must leave a nominal but not uncertainty-adjusted starting gap');
 const result=planner.planAlternativesWithReserve({
  start,goal:{x:2,y:0},params,obstacles:[obstacle],integrate,
  alternativeMaxExpansions:1200,maxExpansions:1200,
  minClearance:0.30,measurementUncertainty:0.10,
  bounds:{minX:-18,maxX:18,minY:-12,maxY:12}
 });
 assert.equal(result.ok,false,JSON.stringify(result));
 assert.equal(result.reason,'uncertainty-no-route');
 assert.equal(result.sensitivity.status,'no-route-under-reserve');
 assert.equal(result.requiredClearance,0.50);
});
 
test('uncertainty-aware alternatives reject negative or excessive uncertainty',()=>{
 const base={
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:2,y:0},
  params,obstacles:[],integrate,alternativeMaxExpansions:100
 };
 assert.equal(planner.planAlternativesWithReserve({...base,measurementUncertainty:-0.01}).reason,'invalid-input');
 assert.equal(planner.planAlternativesWithReserve({...base,measurementUncertainty:5.1}).reason,'invalid-input');
});
