'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');
const params={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,tractorSteeringTrack:2.05,tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.11,tireLength:0.72,tireWidth:0.44};
const integrate=(q,v,steer,dt,p)=>physics.rk4(q,v,steer,dt,p);
test('articulated planner reaches a clear goal in open space',()=>{
 const result=planner.plan({start:{x:0,y:0,psi:0,psiT:0},goal:{x:7,y:0},params,obstacles:[],integrate,maxExpansions:5000});
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.states.length>10);
 assert.ok(Math.hypot(result.states.at(-1).x-7,result.states.at(-1).y)<0.9);
 assert.equal(result.trajectoryClearance.parts.length,3);
 assert.equal(result.states[0].stepDistance,0);
 assert.ok(result.states.slice(1).every(state=>Number.isFinite(state.stepDistance)&&state.stepDistance>0));
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

test('articulated planner reports an expansion limit instead of claiming no route before search exhaustion',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:0.5,y:0,heading:Math.PI},
  params,obstacles:[],integrate,
  maxExpansions:1,
  headingTolerance:5*Math.PI/180,
  maxTerminalArticulation:5*Math.PI/180
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'search-limit');
 assert.equal(result.searchLimitReached,true);
 assert.equal(result.searchExhausted,false);
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

test('clearance checker reports independent minima for tractor, cab and trailer',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const obstacles=[
  {x:6.3,y:0,w:1,h:1,name:'front tractor pallet'},
  {x:-13.65,y:0,w:1,h:1,name:'rear trailer pallet'}
 ];
 const result=planner.createClearanceChecker(obstacles,params)(q);
 assert.equal(result.perPart.length,3);
 assert.equal(result.perPart[0].vehiclePart,'тягач');
 assert.equal(result.perPart[0].obstacle,'front tractor pallet');
 assert.equal(result.perPart[2].vehiclePart,'полуприцеп');
 assert.equal(result.perPart[2].obstacle,'rear trailer pallet');
 assert.ok(result.perPart[2].distance<result.perPart[0].distance);
});

test('trajectory clearance attributes sampled and swept lower bounds to each vehicle part and route segment',()=>{
 const states=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0},
  {x:0.05,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0.05},
  {x:0.10,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0.05}
 ];
 const obstacles=[
  {x:6.3,y:0,w:1,h:1,name:'front tractor pallet'},
  {x:-13.65,y:0,w:1,h:1,name:'rear trailer pallet'}
 ];
 const result=planner.analyzeTrajectoryClearance(states,obstacles,params);
 assert.equal(result.statesAnalyzed,3);
 assert.equal(result.segmentsAnalyzed,2);
 assert.ok(Math.abs(result.distanceAlongRoute-0.1)<1e-10);
 const tractor=result.parts[0],cab=result.parts[1],trailer=result.parts[2];
 assert.equal(tractor.minimumSampledDetail.obstacle,'front tractor pallet');
 assert.equal(trailer.minimumSampledDetail.obstacle,'rear trailer pallet');
 assert.ok(tractor.minimumClearanceLowerBound<=tractor.minimumSampledClearance);
 assert.ok(trailer.minimumClearanceLowerBound<=trailer.minimumSampledClearance);
 assert.equal(trailer.limitingDetail.kind,'segment-lower-bound');
 assert.equal(trailer.limitingDetail.gear,1);
 assert.equal(trailer.limitingDetail.vehiclePart,'полуприцеп');
 assert.equal(result.limitingPart,'полуприцеп');
 assert.ok(result.minimumClearanceLowerBound<=trailer.minimumClearanceLowerBound+1e-9);
 assert.ok(cab.minimumClearanceLowerBound>=tractor.minimumClearanceLowerBound);
});

test('per-part trajectory clearance rejects invalid gear and steering controls',()=>{
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const invalidGear={x:0.05,y:0,psi:0,psiT:0,gear:0,steer:0,stepDistance:0.05};
 const invalidSteer={x:0.05,y:0,psi:0,psiT:0,gear:1,steer:Math.PI/2,stepDistance:0.05};
 assert.throws(()=>planner.analyzeTrajectoryClearance([start,invalidGear],[],params),/invalid gear/);
 assert.throws(()=>planner.analyzeTrajectoryClearance([start,invalidSteer],[],params),/steering angle/);
});

test('per-part trajectory clearance rejects a chord longer than the declared integration distance',()=>{
 const states=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0},
  {x:0.2,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0.05}
 ];
 assert.throws(
  ()=>planner.analyzeTrajectoryClearance(states,[{x:6.3,y:0,w:1,h:1}],params),
  /moves farther than its integration stepDistance/
 );
});

test('per-part trajectory clearance fails closed when an integration step distance is missing',()=>{
 const states=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0},
  {x:0.05,y:0,psi:0,psiT:0,gear:1,steer:0}
 ];
 assert.throws(
  ()=>planner.analyzeTrajectoryClearance(states,[{x:6.3,y:0,w:1,h:1}],params),
  /stepDistance/
 );
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

test('gate helper creates two oriented convex jamb polygons with the requested clear opening',()=>{
 const gate=planner.createGateObstacles({
  centerX:10,centerY:-2,clearWidth:4,postWidth:0.2,depth:0.6,heading:Math.PI/2
 });
 assert.equal(gate.length,2);
 assert.deepEqual(gate.map(item=>item.name),['Левый косяк ворот','Правый косяк ворот']);
 assert.equal(planner.validateObstacles(gate).length,0);
 const u={x:Math.cos(Math.PI/2),y:Math.sin(Math.PI/2)};
 const projections=gate.map(item=>item.polygon.map(point=>point.x*u.x+point.y*u.y));
 const leftInside=Math.max(...projections[0]);
 const rightInside=Math.min(...projections[1]);
 assert.ok(Math.abs((rightInside-leftInside)-4)<1e-9,'inside faces must preserve the measured clear width');
});

test('gate width uncertainty narrows the opening and map pose uncertainty expands both jambs',()=>{
 const nominal=planner.createGateObstacles({
  centerX:10,centerY:-2,clearWidth:4,postWidth:0.2,depth:0.6,heading:Math.PI/2
 });
 const widthBound=planner.createGateObstacles({
  centerX:10,centerY:-2,clearWidth:4,postWidth:0.2,depth:0.6,heading:Math.PI/2,
  widthUncertainty:0.2
 });
 const axis={x:Math.cos(Math.PI/2),y:Math.sin(Math.PI/2)};
 const opening=items=>{
  const projections=items.map(item=>item.polygon.map(point=>point.x*axis.x+point.y*axis.y));
  return Math.min(...projections[1])-Math.max(...projections[0]);
 };
 assert.ok(Math.abs(opening(widthBound)-3.8)<1e-9);
 assert.equal(widthBound[0].gateUncertainty.conservativeClearWidth,3.8);
 const poseBound=planner.createGateObstacles({
  centerX:10,centerY:-2,clearWidth:4,postWidth:0.2,depth:0.6,heading:Math.PI/2,
  widthUncertainty:0.2,positionUncertainty:0.1,headingUncertaintyDeg:2
 });
 assert.equal(planner.validateObstacles(poseBound).length,0);
 const maxRadius=items=>Math.max(...items.flatMap(item=>item.polygon.map(point=>Math.hypot(point.x-10,point.y+2))));
 assert.ok(maxRadius(poseBound)>maxRadius(widthBound));
 assert.ok(poseBound[0].gateUncertainty.totalInflation>0);
 assert.ok(poseBound[0].gateUncertainty.headingInflation>0);
 assert.ok(poseBound[0].gateUncertainty.positionInflation>0);
});

test('gate obstacle uncertainty rejects invalid bounds and a collapsed conservative opening',()=>{
 const base={centerX:0,centerY:0,clearWidth:3,postWidth:0.2,depth:0.6,heading:0};
 assert.throws(()=>planner.createGateObstacles({...base,widthUncertainty:3}));
 assert.throws(()=>planner.createGateObstacles({...base,positionUncertainty:-0.01}));
 assert.throws(()=>planner.createGateObstacles({...base,headingUncertaintyDeg:46}));
});

test('oriented gate jamb polygons participate in whole-vehicle clearance',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const narrow=planner.createGateObstacles({
  centerX:0,centerY:0,clearWidth:3,postWidth:0.2,depth:0.6,heading:Math.PI/2
 });
 const wide=planner.createGateObstacles({
  centerX:0,centerY:0,clearWidth:4,postWidth:0.2,depth:0.6,heading:Math.PI/2
 });
 const narrowClearance=planner.clearance(q,narrow,params).distance;
 const wideClearance=planner.clearance(q,wide,params).distance;
 assert.ok(Math.abs(narrowClearance-0.225)<1e-8,'3 m opening should leave 0.225 m from the 2.55 m footprint');
 assert.ok(Math.abs(wideClearance-0.725)<1e-8,'4 m opening should leave 0.725 m from the 2.55 m footprint');
 assert.ok(wideClearance>narrowClearance);
});

test('convex polygon inflation is finite, outward and rejects concave geometry',()=>{
 const gate=planner.createGateObstacles({
  centerX:0,centerY:0,clearWidth:4,postWidth:0.2,depth:0.6,heading:0.37
 });
 const original=gate[0].polygon;
 const expanded=planner.inflateConvexPolygon(original,0.1);
 assert.equal(expanded.length,4);
 assert.ok(planner.validateObstacles([{polygon:expanded}]).length===0);
 const bounds=points=>({
  minX:Math.min(...points.map(point=>point.x)),
  maxX:Math.max(...points.map(point=>point.x)),
  minY:Math.min(...points.map(point=>point.y)),
  maxY:Math.max(...points.map(point=>point.y))
 });
 const before=bounds(original),after=bounds(expanded);
 assert.ok(after.minX<before.minX&&after.maxX>before.maxX);
 assert.ok(after.minY<before.minY&&after.maxY>before.maxY);
 const concave=[{x:0,y:0},{x:2,y:0},{x:1,y:0.5},{x:2,y:2},{x:0,y:2}];
 assert.ok(planner.validateObstacles([{polygon:concave}]).length>0);
 const star=Array.from({length:5},(_,index)=>{
  const vertex=(index*2)%5;
  const angle=-Math.PI/2+2*Math.PI*vertex/5;
  return {x:Math.cos(angle),y:Math.sin(angle)};
 });
 assert.ok(planner.validateObstacles([{polygon:star}]).length>0,'self-intersecting star polygons must not be accepted as collision obstacles');
});

test('planner routes the full articulated vehicle through an oriented polygon gate',()=>{
 const gate=planner.createGateObstacles({
  centerX:12,centerY:0,clearWidth:6,postWidth:0.25,depth:0.6,heading:Math.PI/2
 });
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:25,y:0},
  params,obstacles:gate,integrate,
  maxExpansions:5000,minClearance:0.30,
  bounds:{minX:-18,maxX:32,minY:-12,maxY:12}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.minimumClearanceLowerBound>=0.30-1e-6);
 for(const q of result.states.slice(1)){
  assert.ok(planner.clearance(q,gate,params).distance>=0.30-1e-6,
   'tractor, cab and trailer contours must clear both oriented gate jambs');
 }
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
 assert.equal(result.steeringWheelDirection,'left');
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
 assert.equal(result.steeringWheelDirection,'right');
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
   const trailerTrack=p.trailerTrack??(p.trailerWidth+0.24);
   const tireWidth=p.tireWidth??0.44;
   const tireHalfLength=(p.tireLength??0.72)/2;
   const axleSpacing=p.trailerAxleSpacing??1.3;
   const wheelLongitudinals=[axleSpacing,0,-axleSpacing]
    .flatMap(offset=>[offset-tireHalfLength,offset+tireHalfLength]);
   const halfWidth=Math.max(p.trailerWidth/2,(trailerTrack+tireWidth)/2);
   let trailerRate=0;
   for(const coupling of [-a,a]){
    for(const longitudinal of [front,rear,...wheelLongitudinals]){
     for(const lateral of [-halfWidth,halfWidth]){
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
  positionTolerance:0.1,headingTolerance:0.04,
  maxTerminalArticulation:0.5
 });
 assert.equal(result.candidatesAttempted,5);
 assert.ok(result.candidatesFound>=1,JSON.stringify(result));
 assert.ok(result.candidateSummaries.some(candidate=>candidate.label==='вперёд, руль влево'),
  'the matching constrained forward-left route should be found');
 assert.ok(result.candidateSummaries.every(candidate=>Number.isFinite(candidate.cost)));
 assert.ok(result.candidateSummaries.every(candidate=>candidate.steeringWheelDirection===candidate.firstTurn));
 if(result.ok){
  assert.ok(['single-candidate','preferred'].includes(result.status));
  assert.ok(result.states.length>1);
  assert.ok(result.recommendation);
  assert.equal(result.trajectoryClearance.parts.length,3);
 }else{
  assert.equal(result.reason,'ambiguous-start');
  assert.equal(result.comparison.recommendation,null);
  assert.ok(result.candidateRoutes.every(candidate=>candidate.trajectoryClearance?.parts?.length===3),
   'ambiguous alternatives must retain their per-part trajectory diagnostics');
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
 
test('recommendation sensitivity reports which obstacle limits each selected scenario',()=>{
 const nominal={
  ok:true,
  recommendation:{initialGear:1,firstTurn:'left',firstTurnGear:1,tractorYawDirection:'left'},
  minimumClearanceLowerBound:0.42,
  minimumClearanceDetail:{
   distance:0.48,obstacle:'столб у левого косяка',
   vehiclePart:'тягач',component:'левое переднее колесо тягача'
  },
  sweptClearanceDetail:{
   kind:'segment-lower-bound',lowerBound:0.42,
   referenceObstacle:'столб у левого косяка',referenceVehiclePart:'тягач'
  }
 };
 const envelope={
  ok:true,
  recommendation:{initialGear:1,firstTurn:'right',firstTurnGear:1,tractorYawDirection:'right'},
  minimumClearanceLowerBound:0.31,
  minimumClearanceDetail:{
   distance:0.36,obstacle:'правый столб',
   vehiclePart:'полуприцеп',component:'правое колесо 3-й оси полуприцепа'
  },
  sweptClearanceDetail:{
   kind:'segment-lower-bound',lowerBound:0.31,
   referenceObstacle:'правый столб',referenceVehiclePart:'полуприцеп'
  }
 };
 const result=planner.compareRecommendationSensitivity(nominal,envelope);
 assert.equal(result.status,'changed');
 assert.equal(result.clearanceEvidence.limitingObstacleChanged,true);
 assert.equal(result.clearanceEvidence.limitingVehiclePartChanged,true);
 assert.equal(result.clearanceEvidence.nominal.limitingObstacle,'столб у левого косяка');
 assert.equal(result.clearanceEvidence.nominal.limitingVehiclePart,'тягач');
 assert.equal(result.clearanceEvidence.nominal.minimumClearanceLowerBound,0.42);
 assert.equal(result.clearanceEvidence.nominal.sampled.component,'левое переднее колесо тягача');
 assert.equal(result.clearanceEvidence.reserve.limitingObstacle,'правый столб');
 assert.equal(result.clearanceEvidence.reserve.limitingVehiclePart,'полуприцеп');
 assert.equal(result.clearanceEvidence.reserve.minimumClearanceLowerBound,0.31);
 assert.equal(result.clearanceEvidence.reserve.swept.kind,'segment-lower-bound');
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
 
test('recommendation sensitivity blocks a materially different first-turn lead-in distance',()=>{
 const nominal={
  ok:true,recommendation:{
   initialGear:1,firstTurn:'left',firstTurnGear:1,tractorYawDirection:'left',
   firstTurnAfterGearChange:false,firstTurnDistance:0.45
  }
 };
 const envelope={
  ok:true,recommendation:{
   initialGear:1,firstTurn:'left',firstTurnGear:1,tractorYawDirection:'left',
   firstTurnAfterGearChange:false,firstTurnDistance:0.90
  }
 };
 const result=planner.compareRecommendationSensitivity(nominal,envelope);
 assert.equal(result.ok,true);
 assert.equal(result.status,'changed');
 assert.ok(result.changedFields.includes('firstTurnDistance'));
 assert.equal(result.firstTurnDistanceTolerance,0.25);
});

test('recommendation sensitivity ignores small first-turn distance variation within tolerance',()=>{
 const action={
  initialGear:1,firstTurn:'left',firstTurnGear:1,tractorYawDirection:'left',
  firstTurnAfterGearChange:false,firstTurnDistance:0.45
 };
 const result=planner.compareRecommendationSensitivity(
  {ok:true,recommendation:action},
  {ok:true,recommendation:{...action,firstTurnDistance:0.60}}
 );
 assert.equal(result.status,'stable');
 assert.deepEqual(result.changedFields,[]);
});

test('near-equal routes with materially different turn lead-in distances are ambiguous',()=>{
 const earlyTurn=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.45,y:0,gear:1,steer:0.12},
  {x:0.9,y:0.02,gear:1,steer:0.12}
 ];
 const delayedTurn=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.3,y:0,gear:1,steer:0},
  {x:0.6,y:0,gear:1,steer:0},
  {x:0.9,y:0.02,gear:1,steer:0.12},
  {x:1.2,y:0.05,gear:1,steer:0.12}
 ];
 const result=planner.compareStartCandidates([
  {states:earlyTurn,cost:10,label:'ранний поворот'},
  {states:delayedTurn,cost:10.1,label:'поздний поворот'}
 ],{costTolerance:0.25});
 assert.equal(result.ok,true);
 assert.equal(result.status,'ambiguous');
 assert.equal(result.recommendation,null);
 assert.ok(result.ambiguousFields.includes('firstTurnDistance'));
 assert.ok(result.bestCandidate.firstTurnDistance<0.5);
 assert.ok(result.comparableAlternatives[0].firstTurnDistance>0.8);
});

test('uncertainty-aware alternatives add twice the per-side reserve to required clearance',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const result=planner.planAlternativesWithReserve({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
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

test('geometry uncertainty planning uses expanded obstacle and vehicle envelopes',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const result=planner.planAlternativesWithGeometryUncertainty({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  maxExpansions:1200,alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,minClearance:0.30,
  measurementUncertainty:0,
  obstaclePositionUncertainty:0.05,
  obstacleSizeUncertainty:0.02,
  vehicleFootprintUncertainty:0.03
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.geometrySensitivity.status,'stable-within-bounded-envelope');
 assert.ok(Math.abs(result.geometrySensitivity.obstacleExpansionPerSide-0.06)<1e-12);
 assert.equal(result.geometrySensitivity.footprintExpansionPerEdge,0.03);
 assert.equal(result.geometrySensitivity.kinematicParameterUncertaintyIncluded,false);
});

test('geometry uncertainty envelope rejects a nominal route with insufficient expanded starting clearance',()=>{
 const obstacle={x:2,y:2,w:1,h:0.5,name:'near-side wall'};
 const start={x:0,y:0,psi:0,psiT:0};
 const initial=planner.clearance(start,[obstacle],params).distance;
 assert.ok(initial>0.30&&initial<0.50);
 const result=planner.planAlternativesWithGeometryUncertainty({
  start,goal:{x:2,y:0},params,obstacles:[obstacle],integrate,
  maxExpansions:1200,alternativeMaxExpansions:1200,
  minClearance:0.30,measurementUncertainty:0,costTolerance:0,
  obstaclePositionUncertainty:0.15,
  obstacleSizeUncertainty:0.10,
  vehicleFootprintUncertainty:0.10,
  bounds:{minX:-18,maxX:18,minY:-12,maxY:12}
 });
 assert.equal(result.ok,false,JSON.stringify(result));
 assert.equal(result.reason,'geometry-envelope-no-route');
 assert.equal(result.geometrySensitivity.status,'no-route-under-envelope');
});

test('geometry uncertainty planning rejects invalid bounds and does not mutate inputs',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 const obstacle={x:8,y:4,w:1,h:1,name:'fixture'};
 const obstacleCopy={...obstacle};
 const paramsCopy={...params};
 const base={start,goal:{x:3,y:0},params,obstacles:[obstacle],integrate,alternativeMaxExpansions:100};
 assert.equal(planner.planAlternativesWithGeometryUncertainty({...base,obstaclePositionUncertainty:-0.01}).reason,'invalid-input');
 assert.equal(planner.planAlternativesWithGeometryUncertainty({...base,vehicleFootprintUncertainty:1.1}).reason,'invalid-input');
 assert.deepEqual(obstacle,obstacleCopy);
 assert.deepEqual(params,paramsCopy);
});

test('swept point-displacement bound grows with the outward footprint envelope',()=>{
 const nominal=planner.sweptDisplacementPerMeter(0.2,params);
 const expanded=planner.sweptDisplacementPerMeter(0.2,{...params,footprintUncertainty:0.15});
 assert.ok(expanded>nominal,`expanded bound ${expanded} must exceed nominal ${nominal}`);
});


test('swept displacement bound contains every body-corner displacement across articulation and reverse motion',()=>{
 const distance=0.05;
 const expandedParams={...params,footprintUncertainty:0.15};
 const articulationAngles=Array.from({length:25},(_,index)=>-Math.PI+2*Math.PI*index/24);
 const steeringAngles=[-0.55,-0.30,0,0.30,0.55];
 let checkedCorners=0;
 for(const beta of articulationAngles){
  for(const steer of steeringAngles){
   for(const gear of [-1,1]){
    const start={x:0,y:0,psi:beta,psiT:0};
    const bound=planner.sweptDisplacementPerMeter(steer,expandedParams,start,distance);
    const finish=physics.rk4(start,gear,steer,distance,expandedParams);
    const before=physics.vehiclePolygons(start,expandedParams);
    const after=physics.vehiclePolygons(finish,expandedParams);
    for(let body=0;body<before.length;body++){
     for(let corner=0;corner<before[body].length;corner++){
      const displacement=Math.hypot(
       after[body][corner].x-before[body][corner].x,
       after[body][corner].y-before[body][corner].y
      );
      assert.ok(
       displacement<=bound*distance+1e-9,
       'body '+body+' corner '+corner+' displacement '+displacement+
        ' exceeded bound '+(bound*distance)+' at beta '+beta+
        ', steer '+steer+', gear '+gear
      );
      checkedCorners++;
     }
    }
   }
  }
 }
 assert.equal(checkedCorners,25*5*2*3*4);
});


test('vehicle parameter bounds evaluate all endpoint combinations for one active parameter',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const originalParams={...params};
 const bounds={wheelbase:{min:3.7999,max:3.8}};
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,minClearance:0.30,
  measurementUncertainty:0,
  obstaclePositionUncertainty:0,
  obstacleSizeUncertainty:0,
  vehicleFootprintUncertainty:0,
  vehicleParameterBounds:bounds
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.parameterSensitivity.status,'stable-across-joint-endpoint-combinations');
 assert.equal(result.parameterSensitivity.coverage,'full-endpoint-combinations');
 assert.equal(result.parameterSensitivity.testedScenarioCount,3);
 assert.equal(result.parameterSensitivity.scenarioCount,3);
 assert.equal(result.parameterSensitivity.midpointScenarioCount,1);
 assert.equal(result.parameterSensitivity.testedMidpointScenarioCount,1);
 assert.equal(result.parameterSensitivity.interiorCoverage,'one-factor-midpoints');
 const midpoint=result.parameterSensitivity.completedScenarios.find(s=>s.kind==='one-factor-midpoint');
 assert.ok(midpoint,'the interval midpoint should be tested separately');
 assert.equal(midpoint.endpointSides.wheelbase,'midpoint');
 assert.ok(Math.abs(midpoint.parameterValues.wheelbase-3.79995)<1e-10);
 assert.equal(result.parameterSensitivity.allParameterCombinationsEvaluated,true);
 assert.equal(result.parameterSensitivity.pairwiseEndpointCoverage,false);
 assert.deepEqual(params,originalParams);
 assert.deepEqual(bounds,{wheelbase:{min:3.7999,max:3.8}});
});

test('vehicle parameter bounds pairwise covering array checks every endpoint pair for seven active dimensions',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const bounds={
  wheelbase:{min:3.799,max:3.801},
  hitchOffset:{min:0.574,max:0.576},
  kingpinAxles:{min:7.999,max:8.001},
  trailerLength:{min:13.599,max:13.601},
  trailerWidth:{min:2.549,max:2.551},
  trailerAxleSpacing:{min:1.299,max:1.301},
  tractorSteeringTrack:{min:2.049,max:2.051}
 };
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,minClearance:0.30,
  measurementUncertainty:0,
  obstaclePositionUncertainty:0,
  obstacleSizeUncertainty:0,
  vehicleFootprintUncertainty:0,
  vehicleParameterBounds:bounds
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 const sensitivity=result.parameterSensitivity;
 assert.equal(sensitivity.status,'stable-across-pairwise-endpoint-scenarios');
 assert.equal(sensitivity.coverage,'pairwise-endpoint-covering-array');
 assert.equal(sensitivity.testedScenarioCount,9);
 assert.equal(sensitivity.midpointScenarioCount,0);
 assert.equal(sensitivity.testedMidpointScenarioCount,0);
 assert.equal(sensitivity.interiorCoverage,'one-factor-midpoints');
 assert.equal(sensitivity.allParameterCombinationsEvaluated,false);
 assert.equal(sensitivity.pairwiseEndpointCoverage,true);
 const keys=Object.keys(bounds);
 const expected=['max/max','max/min','min/max','min/min'];
 for(let i=0;i<keys.length;i++)for(let j=i+1;j<keys.length;j++){
  const observed=new Set(sensitivity.completedScenarios.map(s=>
   s.endpointSides[keys[i]]+'/'+s.endpointSides[keys[j]]
  ));
  assert.deepEqual([...observed].sort(),expected);
 }
});

test('pairwise endpoint generator covers all pairs for nineteen cab, mirror, tire and track dimensions',()=>{
 const rows=planner.parameterEndpointVectors(19);
 assert.equal(rows.length,33);
 assert.ok(rows.some(row=>row.every(value=>value===1)),'joint all-high endpoint should be included');
 const expected=['0/0','0/1','1/0','1/1'];
 for(let i=0;i<19;i++)for(let j=i+1;j<19;j++){
  const observed=new Set(rows.map(row=>row[i]+'/'+row[j]));
  assert.deepEqual([...observed].sort(),expected,'pair '+i+'/'+j);
 }
 const maximum=planner.parameterEndpointVectors(31);
 assert.equal(maximum.length,33);
 for(let i=0;i<31;i++)for(let j=i+1;j<31;j++){
  const observed=new Set(maximum.map(row=>row[i]+'/'+row[j]));
  assert.deepEqual([...observed].sort(),expected,'maximum pair '+i+'/'+j);
 }
 assert.throws(()=>planner.parameterEndpointVectors(32),/от 0 до 31/);
});

test('vehicle parameter bounds accept a calibrated front tire track interval',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,minClearance:0.30,
  measurementUncertainty:0,obstaclePositionUncertainty:0,
  obstacleSizeUncertainty:0,vehicleFootprintUncertainty:0,
  vehicleParameterBounds:{tractorFrontTrack:{min:2.049,max:2.051}}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.parameterSensitivity.activeParameterCount,1);
 assert.equal(result.parameterSensitivity.testedScenarioCount,2);
 assert.equal(result.parameterSensitivity.testedMidpointScenarioCount,0);
 assert.equal(result.parameterSensitivity.coverage,'full-endpoint-combinations');
});

test('vehicle parameter bounds include a possible mirror projection from a zero nominal width',()=>{
 const geometryParams={
  ...params,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0,mirrorLongitudinal:3.0,mirrorLength:0.8
 };
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,geometryParams);
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params:geometryParams,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,minClearance:0.30,
  measurementUncertainty:0,obstaclePositionUncertainty:0,
  obstacleSizeUncertainty:0,vehicleFootprintUncertainty:0,
  vehicleParameterBounds:{mirrorOutboard:{min:0,max:0.02}}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.parameterSensitivity.activeParameterCount,1);
 assert.equal(result.parameterSensitivity.testedScenarioCount,3);
 assert.equal(result.parameterSensitivity.testedMidpointScenarioCount,1);
 assert.equal(result.parameterSensitivity.coverage,'full-endpoint-combinations');
 assert.equal(result.parameterSensitivity.suppliedBounds.mirrorOutboard.min,0);
 assert.equal(result.parameterSensitivity.suppliedBounds.mirrorOutboard.max,0.02);
});

test('vehicle parameter bounds accept all nineteen truck cab, mirror, tire and trailer dimensions',()=>{
 const geometryParams={
  ...params,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0,mirrorLongitudinal:3.0,mirrorLength:0.8
 };
 const halfWidths={
  wheelbase:0.01,hitchOffset:0.005,kingpinAxles:0.05,trailerLength:0.05,
  trailerWidth:0.01,trailerAxleSpacing:0.01,tractorSteeringTrack:0.01,
  tractorFrontTrack:0.01,tractorRearTrack:0.01,trailerTrack:0.01,
  tireLength:0.01,tireWidth:0.01,cabOffset:0.01,cabFront:0.01,
  cabRear:0.01,cabWidth:0.01,mirrorOutboard:0.02,
  mirrorLongitudinal:0.01,mirrorLength:0.01
 };
 const limits=physics.VEHICLE_PARAMETER_LIMITS;
 const bounds=Object.fromEntries(Object.entries(halfWidths).map(([key,halfWidth])=>[
  key,{
   min:Math.max(limits[key].min,geometryParams[key]-halfWidth),
   max:Math.min(limits[key].max,geometryParams[key]+halfWidth)
  }
 ]));
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:100,y:0},
  params:geometryParams,obstacles:[],integrate,
  maxExpansions:1,alternativeMaxExpansions:1,
  measurementUncertainty:0,obstaclePositionUncertainty:0,
  obstacleSizeUncertainty:0,vehicleFootprintUncertainty:0,
  vehicleParameterBounds:bounds
 });
 assert.equal(Object.keys(bounds).length,19);
 assert.ok(result.parameterSensitivity,JSON.stringify(result));
 assert.equal(result.parameterSensitivity.activeParameterCount,19);
 assert.equal(result.parameterSensitivity.coverage,'pairwise-endpoint-covering-array');
 assert.equal(result.parameterSensitivity.pairwiseEndpointCoverage,true);
 assert.equal(result.parameterSensitivity.allParameterCombinationsEvaluated,false);
});

test('vehicle parameter bounds reject a physically invalid joint endpoint combination',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,
  vehicleParameterBounds:{
   kingpinAxles:{min:8,max:10.2},
   trailerLength:{min:12.9,max:13.6}
  }
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'parameter-scenario-unavailable');
 assert.equal(result.parameterSensitivity.status,'joint-scenario-invalid');
 assert.equal(result.parameterSensitivity.failedScenario,'совместная комбинация 02/4');
 assert.deepEqual(result.parameterSensitivity.failedParameters.map(item=>item.key),['kingpinAxles','trailerLength']);
 assert.ok(result.parameterSensitivity.validationErrors.length>0);
});

test('vehicle parameter bounds report not-varied instead of implying zero physical uncertainty',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[],integrate,
  alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,
  vehicleParameterBounds:{wheelbase:{min:3.8,max:3.8}}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.parameterSensitivity.status,'not-varied');
 assert.equal(result.parameterSensitivity.testedScenarioCount,0);
 assert.equal(result.parameterSensitivity.kinematicParameterUncertaintyIncluded,false);
});

test('vehicle parameter bounds reject unknown, inverted, out-of-range, and non-containing intervals',()=>{
 const base={
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:2,y:0},
  params,obstacles:[],integrate,alternativeMaxExpansions:100
 };
 for(const bounds of [
  {unknown:{min:0,max:1}},
  {wheelbase:{min:4,max:3}},
  {wheelbase:{min:0,max:3.8}},
  {hitchOffset:{min:-0.1,max:0.575}},
  {kingpinAxles:{min:7,max:7.5}},
  {trailerAxleSpacing:{min:0.2,max:1.3}},
  {tractorSteeringTrack:{min:0.9,max:2.05}}
 ]){
  const result=planner.planAlternativesWithVehicleParameterBounds({
   ...base,vehicleParameterBounds:bounds
  });
  assert.equal(result.ok,false,JSON.stringify({bounds,result}));
  assert.equal(result.reason,'invalid-input');
 }
});


test('gate uncertainty compares nominal and joint width-centre-heading envelopes',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 const goal={x:7,y:0};
 const gateGeometry={
  centerX:3.5,centerY:0,clearWidth:8,postWidth:0.25,depth:0.6,heading:Math.PI/2,
  widthUncertainty:0.4,positionUncertainty:0.1,headingUncertaintyDeg:1
 };
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start,goal,params,integrate,
  obstacles:planner.createGateObstacles(gateGeometry),
  gateGeometry,
  alternativeMaxExpansions:3500,
  costTolerance:0.001,
  positionTolerance:0.9,
  maxTerminalArticulation:12*Math.PI/180,minClearance:0.3,
  measurementUncertainty:0,
  obstaclePositionUncertainty:0,
  obstacleSizeUncertainty:0,
  vehicleFootprintUncertainty:0
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.geometrySensitivity.status,'stable-within-bounded-envelope');
 const gate=result.geometrySensitivity.gateGeometryUncertainty;
 assert.equal(gate.enabled,true);
 assert.equal(gate.nominalClearWidth,8);
 assert.equal(gate.conservativeClearWidth,7.6);
 assert.equal(gate.widthUncertainty,0.4);
 assert.equal(gate.positionUncertaintyPerAxis,0.1);
 assert.equal(gate.headingUncertaintyDeg,1);
 assert.equal(result.geometrySensitivity.comparison.status,'stable');
 assert.ok(result.geometrySensitivity.nominalCandidatesFound>=1);
 assert.ok(result.geometrySensitivity.envelopeCandidatesFound>=1);
});

test('gate uncertainty rejects an invalid joint envelope before changing route options',()=>{
 const gateGeometry={
  centerX:2,centerY:0,clearWidth:3,postWidth:0.25,depth:0.6,heading:Math.PI/2,
  widthUncertainty:3,positionUncertainty:0.1,headingUncertaintyDeg:1
 };
 const obstacles=planner.createGateObstacles({
  ...gateGeometry,widthUncertainty:0,positionUncertainty:0,headingUncertaintyDeg:0
 });
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:4,y:0,heading:0},
  params,integrate,obstacles,gateGeometry,
  alternativeMaxExpansions:100
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'invalid-input');
 assert.match(result.errors.join(' '),/неопределённости|границы/);
});


test('ambiguous nominal gate route does not claim the uncertainty envelope was checked',()=>{
 const gateGeometry={
  centerX:3.5,centerY:0,clearWidth:8,postWidth:0.25,depth:0.6,heading:Math.PI/2,
  widthUncertainty:0.4,positionUncertainty:0.1,headingUncertaintyDeg:1
 };
 const result=planner.planAlternativesWithVehicleParameterBounds({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:7,y:0},params,integrate,
  obstacles:planner.createGateObstacles(gateGeometry),gateGeometry,
  alternativeMaxExpansions:3500,costTolerance:0.25,
  positionTolerance:0.9,maxTerminalArticulation:12*Math.PI/180,minClearance:0.3,
  measurementUncertainty:0,obstaclePositionUncertainty:0,
  obstacleSizeUncertainty:0,vehicleFootprintUncertainty:0
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'ambiguous-start');
 assert.equal(result.geometrySensitivity.status,'nominal-ambiguous');
 assert.equal(result.geometrySensitivity.envelopeEvaluated,false);
 assert.equal(result.geometrySensitivity.gateGeometryUncertainty.enabled,true);
});

test('clearance checker attributes an outboard obstacle to the configured left mirror',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const obstacle={x:3,y:1.75,w:0.2,h:0.2,name:'mirror-side post'};
 const withMirrors={
  ...params,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0.45,mirrorLongitudinal:3.0,mirrorLength:0.8
 };
 const result=planner.clearance(q,[obstacle],withMirrors);
 assert.equal(result.vehiclePart,'кабина');
 assert.equal(result.component,'левое зеркало');
 assert.ok(Math.abs(result.distance-0.075)<1e-9);
 const withoutMirrors=planner.clearance(q,[obstacle],params);
 assert.ok(withoutMirrors.distance>result.distance);
});

test('trajectory clearance keeps the mirror component in the limiting cab detail',()=>{
 const q={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const p={
  ...params,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0.45,mirrorLongitudinal:3.0,mirrorLength:0.8
 };
 const result=planner.analyzeTrajectoryClearance([q],[
  {x:3,y:1.75,w:0.2,h:0.2,name:'mirror-side post'}
 ],p);
 assert.equal(result.parts[1].minimumSampledDetail.component,'левое зеркало');
 assert.equal(result.parts[1].minimumSampledDetail.obstacle,'mirror-side post');
});

test('swept displacement bound grows to include configured mirror corners',()=>{
 const base=planner.sweptDisplacementRatesPerMeter(0.25,params);
 const withMirrors=planner.sweptDisplacementRatesPerMeter(0.25,{
  ...params,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0.9,mirrorLongitudinal:3.0,mirrorLength:0.8
 });
 assert.ok(withMirrors.cab>base.cab);
 assert.equal(withMirrors.tractor,base.tractor);
 assert.equal(withMirrors.trailer,base.trailer);
});


test('trailer swept displacement bound includes the outboard tire track',()=>{
 const narrow=planner.sweptDisplacementRatesPerMeter(0.25,{
  ...params,trailerTrack:2.0,tireWidth:0.3
 });
 const wide=planner.sweptDisplacementRatesPerMeter(0.25,{
  ...params,trailerTrack:2.79,tireWidth:0.44
 });
 assert.ok(wide.trailer>narrow.trailer);
 assert.equal(wide.tractor,narrow.tractor);
 assert.equal(wide.cab,narrow.cab);
});

test('clearance checker identifies an outboard trailer tire as the limiting component',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const p={
  ...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const axle=physics.axlePoint(q,p);
 const firstAxle=physics.trailerAxlePoints(q,p)[0];
 const obstacle={x:firstAxle.x,y:1.66,w:0.1,h:0.1,name:'outer trailer post'};
 const result=planner.clearance(q,[obstacle],p);
 assert.equal(result.vehiclePart,'полуприцеп');
 assert.equal(result.component,'левое колесо 1-й оси полуприцепа');
 assert.equal(result.obstacle,'outer trailer post');
 assert.ok(result.distance<0.05);
 assert.ok(axle.x<firstAxle.x);
});


test('tractor swept displacement bound includes wider calibrated tire tracks',()=>{
 const narrow=planner.sweptDisplacementRatesPerMeter(0.25,{
  ...params,wheelbase:5.0,tractorFrontTrack:2.05,tractorRearTrack:2.05,tireWidth:0.44
 });
 const wide=planner.sweptDisplacementRatesPerMeter(0.25,{
  ...params,wheelbase:5.0,tractorFrontTrack:2.7,tractorRearTrack:2.7,tireWidth:0.44
 });
 assert.ok(wide.tractor>narrow.tractor);
 assert.equal(wide.cab,narrow.cab);
 assert.equal(wide.trailer,narrow.trailer);
});


test('per-wheel trajectory clearance identifies the limiting outboard trailer tire',()=>{
 const p={...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44};
 const q={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const firstAxle=physics.trailerAxlePoints(q,p)[0];
 const obstacle={x:firstAxle.x,y:1.66,w:0.1,h:0.1,name:'outer trailer post'};
 const result=planner.analyzeWheelTrajectoryClearance([q],[obstacle],p);
 assert.equal(result.wheels.length,10);
 assert.equal(result.limitingWheel.name,'левое колесо 1-й оси полуприцепа');
 assert.equal(result.limitingWheel.limitingDetail.obstacle,'outer trailer post');
 assert.ok(result.limitingWheel.minimumSampledPolygonClearance<0.05);
 assert.ok(result.minimumClearanceLowerBound<=result.limitingWheel.minimumSampledEnvelopeClearance+1e-9);
 assert.match(result.envelope,/circumscribed/);
});

test('per-wheel clearance expands sampled and circumscribed envelopes for configured dual-tire groups',()=>{
 const base={
  ...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const q={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const firstAxle=physics.trailerAxlePoints(q,base)[0];
 const obstacle={x:firstAxle.x,y:1.92,w:0.1,h:0.1,name:'dual tire outer edge'};
 const single=planner.analyzeWheelTrajectoryClearance([q],[obstacle],base);
 const dual=planner.analyzeWheelTrajectoryClearance([q],[obstacle],{
  ...base,tractorRearTireGroupWidth:0.72,trailerTireGroupWidth:0.72
 });
 const singleWheel=single.wheels.find(item=>item.name==='левое колесо 1-й оси полуприцепа');
 const dualWheel=dual.wheels.find(item=>item.name==='левое колесо 1-й оси полуприцепа');
 assert.ok(singleWheel&&dualWheel);
 assert.equal(singleWheel.footprintModel,'single-tire-fallback');
 assert.equal(dualWheel.footprintModel,'configured-axle-side-group-envelope');
 assert.equal(dualWheel.footprintWidth,0.72);
 assert.ok(dualWheel.minimumSampledPolygonClearance<singleWheel.minimumSampledPolygonClearance);
 assert.ok(dualWheel.minimumSampledEnvelopeClearance<singleWheel.minimumSampledEnvelopeClearance);
 assert.ok(dualWheel.minimumClearanceLowerBound<singleWheel.minimumClearanceLowerBound);
});

test('trailer swept displacement rate includes the configured tire-group outer width',()=>{
 const base={
  ...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const single=planner.sweptDisplacementRatesPerMeter(0.2,base);
 const dual=planner.sweptDisplacementRatesPerMeter(0.2,{
  ...base,tractorRearTireGroupWidth:0.72,trailerTireGroupWidth:0.72
 });
 assert.ok(dual.trailer>single.trailer);
 assert.ok(dual.tractor>=single.tractor);
});

test('per-wheel trajectory clearance applies a segment lower bound between sampled poses',()=>{
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const next={x:0.05,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0.1};
 const obstacle={x:4.55,y:1.025,w:0.2,h:0.2,name:'front wheel post'};
 const result=planner.analyzeWheelTrajectoryClearance([start,next],[obstacle],{
  ...params,tractorFrontTrack:2.05,tireLength:0.72,tireWidth:0.44
 });
 const wheel=result.wheels.find(item=>item.name==='левое переднее колесо тягача');
 assert.ok(wheel);
 assert.equal(wheel.limitingDetail.kind,'segment-circumscribed-envelope-lower-bound');
 assert.ok(wheel.minimumClearanceLowerBound<=wheel.minimumSampledEnvelopeClearance+1e-9);
 assert.equal(wheel.limitingDetail.obstacleAtReference,'front wheel post');
});

test('per-wheel clearance adaptively samples long RK4 segments and tightens interval lower bounds',()=>{
 const p={...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44};
 const stepDistance=2;
 const steer=0.25;
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer,stepDistance:0};
 const middle=physics.rk4(start,0.7,steer,stepDistance*0.5/0.7,p);
 const end=physics.rk4(start,0.7,steer,stepDistance/0.7,p);
 const target=physics.wheelFootprints(middle,p,steer).find(wheel=>
  wheel.name==='левое колесо 1-й оси полуприцепа'
 );
 assert.ok(target);
 const obstacle={x:target.center.x-0.04,y:target.center.y-0.04,w:0.08,h:0.08,name:'mid-segment tire post'};
 const result=planner.analyzeWheelTrajectoryClearance([
  start,{...end,gear:1,steer,stepDistance}
 ],[obstacle],p);
 const wheel=result.wheels.find(item=>item.name===target.name);
 const segment=result.segmentDiagnostics[0];
 assert.equal(result.segmentsWithIntermediateKinematicSamples,1);
 assert.equal(result.segmentsWithoutIntermediateKinematicSamples,0);
 assert.ok(segment.intervalCount>4);
 assert.equal(result.intermediateSamplesAnalyzed,segment.intervalCount-1);
 assert.equal(segment.sampleFractions.length,segment.intervalCount-1);
 assert.ok(segment.sampleFractions.some(fraction=>Math.abs(fraction-0.5)<=0.5/segment.intervalCount));
 assert.equal(segment.status,'rk4-adaptive-intermediate-samples');
 assert.ok(segment.maxWheelCenterDisplacementPerInterval<=
  result.targetMaxWheelCenterDisplacementPerInterval+1e-9||segment.adaptiveCapReached);
 assert.equal(wheel.minimumIntermediatePolygonDetail.obstacle,'mid-segment tire post');
 assert.ok(wheel.minimumIntermediatePolygonClearance<=1e-9);
 assert.ok(wheel.minimumIntermediateEnvelopeClearance<wheel.minimumSampledEnvelopeClearance);
 assert.ok(result.minimumClearanceLowerBound<=wheel.minimumIntermediateEnvelopeClearance+1e-9);
 const detail=wheel.limitingDetail;
 assert.equal(detail.kind,'adaptive-segment-circumscribed-envelope-lower-bound');
 const expected=detail.referenceDistance-
  detail.displacementRate*detail.stepDistance/(2*detail.intervalCount);
 assert.ok(Math.abs(detail.lowerBound-expected)<1e-9);
 assert.ok(detail.lowerBound>
  detail.referenceDistance-detail.displacementRate*detail.stepDistance);
});

test('per-wheel clearance reports when adaptive RK4 sampling reaches its interval cap',()=>{
 const p={...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44};
 const stepDistance=10;
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const end=physics.rk4(start,0.7,0,stepDistance/0.7,p);
 const obstacle={x:30,y:30,w:0.1,h:0.1,name:'distant cap-test obstacle'};
 const result=planner.analyzeWheelTrajectoryClearance([
  start,{...end,gear:1,steer:0,stepDistance}
 ],[obstacle],p);
 const segment=result.segmentDiagnostics[0];
 assert.equal(segment.intervalCount,result.maxAdaptiveIntervals);
 assert.equal(segment.adaptiveCapReached,true);
 assert.equal(result.segmentsAtAdaptiveSamplingCap,1);
 assert.equal(result.intermediateSamplesAnalyzed,segment.intervalCount-1);
 assert.ok(Number.isFinite(segment.maxWheelCenterDisplacementPerInterval));
});
 
test('per-wheel clearance skips intermediate RK4 samples when the reconstructed endpoint does not match',()=>{
 const p={...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44};
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0.18,stepDistance:0};
 const inconsistent={x:0.5,y:0,psi:0,psiT:0,gear:1,steer:0.18,stepDistance:0.5};
 const obstacle={x:0.4,y:1.5,w:0.1,h:0.1,name:'endpoint mismatch post'};
 const result=planner.analyzeWheelTrajectoryClearance([start,inconsistent],[obstacle],p);
 assert.equal(result.segmentsWithIntermediateKinematicSamples,0);
 assert.equal(result.segmentsWithoutIntermediateKinematicSamples,1);
 assert.equal(result.intermediateSamplesAnalyzed,0);
 assert.equal(result.segmentDiagnostics[0].status,'kinematic-endpoint-mismatch');
 assert.ok(result.wheels.some(wheel=>Number.isFinite(wheel.minimumClearanceLowerBound)));
});

test('per-wheel trajectory clearance rejects missing or inconsistent integration distances',()=>{
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const missing={x:0.05,y:0,psi:0,psiT:0,gear:1,steer:0};
 const tooFar={x:0.2,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0.05};
 assert.throws(()=>planner.analyzeWheelTrajectoryClearance([start,missing],[],params),/stepDistance/);
 assert.throws(()=>planner.analyzeWheelTrajectoryClearance([start,tooFar],[],params),/integration stepDistance/);
});

test('per-wheel trajectory clearance reports no limiting wheel when the obstacle set is empty',()=>{
 const path=[{x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0}];
 const result=planner.analyzeWheelTrajectoryClearance(path,[],params);
 assert.equal(result.wheels.length,10);
 assert.equal(result.limitingWheel,null);
 assert.equal(result.minimumClearanceLowerBound,Infinity);
});


test('clearance checker forwards steering to protruding front tire collision geometry',()=>{
 const p={
  ...params,
  tractorFrontTrack:2.4,tractorRearTrack:2.05,trailerTrack:2.5,
  tireLength:1.2,tireWidth:0.8
 };
 const state={x:0,y:0,psi:0,psiT:0,gear:1,steer:0.6};
 const wheel=physics.wheelFootprints(state,p,state.steer).find(item=>
  item.name==='левое переднее колесо тягача'
 );
 assert.ok(wheel);
 const contact=wheel.polygon[0];
 const obstacle={x:contact.x,y:contact.y,w:0.04,h:0.04,name:'steered wheel post'};
 const result=planner.clearance(state,[obstacle],p);
 assert.equal(result.vehiclePart,'тягач');
 assert.equal(result.component,'левое переднее колесо тягача');
 assert.equal(result.obstacle,'steered wheel post');
 assert.ok(result.distance<=1e-9,'the steered tire corner should intersect the obstacle');
 const unsteered=planner.clearance({...state,steer:0},[obstacle],p);
 assert.ok(unsteered.distance>0.1,'the same obstacle must not intersect the unsteered tire envelope');
});


test('clearance checker applies the requested Ackermann wheel angle before measuring a protruding front tire',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  tractorFrontTrack:2.4,tractorSteeringTrack:2.4,tractorRearTrack:2.05,
  trailerTrack:2.5,tireLength:1.2,tireWidth:0.8
 };
 const q={x:0,y:0,psi:0,psiT:0};
 const wheel=physics.wheelFootprints(q,p,0.6).find(item=>
  item.name==='левое переднее колесо тягача'
 );
 assert.ok(wheel);
 const corner=wheel.polygon[0];
 const obstacle={x:corner.x,y:corner.y,w:0.04,h:0.04,name:'front tire corner post'};
 const straight=planner.clearance(q,[obstacle],p,0);
 const steered=planner.clearance(q,[obstacle],p,0.6);
 assert.ok(straight.distance>0.1,JSON.stringify(straight));
 assert.equal(steered.distance,0,JSON.stringify(steered));
 assert.equal(steered.component,'левое переднее колесо тягача');
 const stateSteered=planner.clearance({...q,steer:0.6},[obstacle],p);
 assert.equal(stateSteered.distance,0);
});


test('obstacle pose uncertainty envelope contains bounded X/Y translation and heading rotation',()=>{
 const anchor={x:3,y:-2};
 const polygon=[
  {x:2.5,y:-2.2},{x:3.5,y:-2.2},
  {x:3.5,y:-1.8},{x:2.5,y:-1.8}
 ];
 const original=polygon.map(point=>({...point}));
 const bounds={positionX:0.12,positionY:0.08,headingDeg:5};
 const result=planner.inflateObstaclePoseUncertainty(polygon,anchor,bounds);
 assert.ok(Math.abs(result.positionInflation-Math.hypot(0.12,0.08))<1e-12);
 assert.ok(result.headingInflation>0);
 assert.ok(Math.abs(result.totalInflation-result.positionInflation-result.headingInflation)<1e-12);
 const inside=(point,poly)=>{
  let sign=0;
  for(let i=0;i<poly.length;i++){
   const a=poly[i],b=poly[(i+1)%poly.length];
   const cross=(b.x-a.x)*(point.y-a.y)-(b.y-a.y)*(point.x-a.x);
   if(Math.abs(cross)<1e-8)continue;
   const current=Math.sign(cross);
   if(sign&&current!==sign)return false;
   sign=current;
  }
  return true;
 };
 for(const tx of [-bounds.positionX,bounds.positionX]){
  for(const ty of [-bounds.positionY,bounds.positionY]){
   for(const angle of [-5,5]){
    const radians=angle*Math.PI/180,cos=Math.cos(radians),sin=Math.sin(radians);
    for(const point of polygon){
     const dx=point.x-anchor.x,dy=point.y-anchor.y;
     const moved={x:anchor.x+dx*cos-dy*sin+tx,y:anchor.y+dx*sin+dy*cos+ty};
     assert.ok(inside(moved,result.polygon),JSON.stringify({moved,envelope:result.polygon}));
    }
   }
  }
 }
 assert.deepEqual(polygon,original,'the source polygon must not be mutated');
 assert.throws(()=>planner.inflateObstaclePoseUncertainty(polygon,anchor,{positionX:-0.01,positionY:0,headingDeg:0}),/Допуски положения/);
 assert.throws(()=>planner.inflateObstaclePoseUncertainty(polygon,anchor,{positionX:0,positionY:0,headingDeg:46}),/Допуски положения/);
});

test('bounded obstacle map pose uncertainty lowers individual clearances for the ten wheels',()=>{
 const p={...params,tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.79,tireLength:0.72,tireWidth:0.44};
 const q={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const wheel=physics.wheelFootprints(q,p,0).find(item=>item.name==='левое переднее колесо тягача');
 assert.ok(wheel);
 const center=wheel.polygon.reduce((sum,point)=>({x:sum.x+point.x/wheel.polygon.length,y:sum.y+point.y/wheel.polygon.length}),{x:0,y:0});
 const outerY=Math.max(...wheel.polygon.map(point=>point.y));
 const anchor={x:center.x,y:outerY+0.21};
 const obstaclePolygon=[
  {x:anchor.x-0.06,y:anchor.y-0.06},
  {x:anchor.x+0.06,y:anchor.y-0.06},
  {x:anchor.x+0.06,y:anchor.y+0.06},
  {x:anchor.x-0.06,y:anchor.y+0.06}
 ];
 const nominal=planner.analyzeWheelTrajectoryClearance([q],[{name:'manual post',polygon:obstaclePolygon}],p);
 const envelope=planner.inflateObstaclePoseUncertainty(obstaclePolygon,anchor,{positionX:0.12,positionY:0.08,headingDeg:4});
 const bounded=planner.analyzeWheelTrajectoryClearance([q],[{name:'manual post',polygon:envelope.polygon}],p);
 assert.equal(nominal.wheels.length,10);
 assert.equal(bounded.wheels.length,10);
 const nominalWheel=nominal.wheels.find(item=>item.name==='левое переднее колесо тягача');
 const boundedWheel=bounded.wheels.find(item=>item.name==='левое переднее колесо тягача');
 assert.ok(nominalWheel&&boundedWheel);
 assert.ok(nominalWheel.minimumSampledPolygonClearance>0,JSON.stringify(nominalWheel));
 assert.ok(boundedWheel.minimumSampledPolygonClearance<nominalWheel.minimumSampledPolygonClearance,JSON.stringify({nominalWheel,boundedWheel}));
});

test('multiple independently uncertain obstacles constrain all ten wheels along a route segment',()=>{
 const p={...params,tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.79,tireLength:0.72,tireWidth:0.44};
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const next={...start,x:0.05,stepDistance:0.05};
 const footprints=physics.wheelFootprints(start,p,0);
 const postForWheel=(wheel,side,name,bounds)=>{
  const xs=wheel.polygon.map(point=>point.x),ys=wheel.polygon.map(point=>point.y);
  const anchor={
   x:(Math.min(...xs)+Math.max(...xs))/2,
   y:side==='left'?Math.max(...ys)+0.55:Math.min(...ys)-0.55
  };
  const half=0.04;
  const polygon=[
   {x:anchor.x-half,y:anchor.y-half},
   {x:anchor.x+half,y:anchor.y-half},
   {x:anchor.x+half,y:anchor.y+half},
   {x:anchor.x-half,y:anchor.y+half}
  ];
  const inflated=planner.inflateObstaclePoseUncertainty(polygon,anchor,bounds);
  return {
   nominal:{name,polygon},
   bounded:{name,polygon:inflated.polygon}
  };
 };
 const frontLeft=footprints.find(wheel=>wheel.name==='левое переднее колесо тягача');
 const trailerRightRear=footprints.find(wheel=>wheel.name==='правое колесо 3-й оси полуприцепа');
 assert.ok(frontLeft&&trailerRightRear);
 const frontPost=postForWheel(frontLeft,'left','столб у переднего левого колеса',{positionX:0.12,positionY:0.08,headingDeg:5});
 const trailerPost=postForWheel(trailerRightRear,'right','столб у правого колеса третьей оси',{positionX:0.08,positionY:0.14,headingDeg:7});
 const nominal=planner.analyzeWheelTrajectoryClearance([start,next],[frontPost.nominal,trailerPost.nominal],p);
 const bounded=planner.analyzeWheelTrajectoryClearance([start,next],[frontPost.bounded,trailerPost.bounded],p);
 assert.equal(nominal.statesAnalyzed,2);
 assert.equal(bounded.statesAnalyzed,2);
 assert.equal(nominal.segmentsAnalyzed,1);
 assert.equal(bounded.segmentsAnalyzed,1);
 assert.equal(nominal.wheels.length,10);
 assert.equal(bounded.wheels.length,10);
 for(const wheel of bounded.wheels){
  assert.ok(Number.isFinite(wheel.minimumSampledPolygonClearance),JSON.stringify(wheel));
  assert.ok(Number.isFinite(wheel.minimumClearanceLowerBound),JSON.stringify(wheel));
 }
 const nominalFront=nominal.wheels.find(wheel=>wheel.name===frontLeft.name);
 const boundedFront=bounded.wheels.find(wheel=>wheel.name===frontLeft.name);
 const nominalTrailer=nominal.wheels.find(wheel=>wheel.name===trailerRightRear.name);
 const boundedTrailer=bounded.wheels.find(wheel=>wheel.name===trailerRightRear.name);
 assert.equal(nominalFront.minimumSampledPolygonDetail.obstacle,frontPost.nominal.name);
 assert.equal(boundedFront.minimumSampledPolygonDetail.obstacle,frontPost.bounded.name);
 assert.equal(nominalTrailer.minimumSampledPolygonDetail.obstacle,trailerPost.nominal.name);
 assert.equal(boundedTrailer.minimumSampledPolygonDetail.obstacle,trailerPost.bounded.name);
 assert.ok(boundedFront.minimumSampledPolygonClearance<nominalFront.minimumSampledPolygonClearance);
 assert.ok(boundedTrailer.minimumSampledPolygonClearance<nominalTrailer.minimumSampledPolygonClearance);
 assert.ok(boundedFront.minimumClearanceLowerBound<=nominalFront.minimumClearanceLowerBound+1e-9);
 assert.ok(boundedTrailer.minimumClearanceLowerBound<=nominalTrailer.minimumClearanceLowerBound+1e-9);
});


test('camera obstacle uncertainty compares raw nominal polygon with its conservative envelope',()=>{
 const start={x:0,y:0,psi:0,psiT:0};
 let target={...start};
 for(let i=0;i<12;i++)target=physics.rk4(target,0.7,0.24,0.05/0.7,params);
 const cameraObstacle={
  kind:'camera-obstacle',
  name:'camera post with uncertain map pose',
  nominalPolygon:[
   {x:20,y:20},{x:21,y:20},{x:21,y:21},{x:20,y:21}
  ],
  polygon:[
   {x:-1,y:-1},{x:1,y:-1},{x:1,y:1},{x:-1,y:1}
  ]
 };
 const result=planner.planAlternativesWithGeometryUncertainty({
  start,goal:{x:target.x,y:target.y,heading:target.psi},
  params,obstacles:[cameraObstacle],integrate,
  maxExpansions:1200,alternativeMaxExpansions:1200,
  positionTolerance:0.05,headingTolerance:0.02,
  maxTerminalArticulation:0.5,minClearance:0.30,
  measurementUncertainty:0,costTolerance:0.001
 });
 assert.equal(result.ok,false,JSON.stringify(result));
 assert.equal(result.reason,'geometry-envelope-no-route');
 assert.ok(result.nominalCandidateSummaries.length>0,
  'the raw nominal obstacle polygon must allow the nominal candidate search');
 assert.equal(result.geometrySensitivity.perObstaclePoseEnvelopeCount,1);
 assert.equal(result.geometrySensitivity.status,'no-route-under-envelope');
 assert.equal(result.envelopeFailure.reason,'no-candidate-route');
 assert.ok(result.envelopeFailure.failures.every(failure=>failure.reason==='start-collision'));
});


test('invalid nominal camera obstacle polygon is rejected before planning',()=>{
 const result=planner.planAlternativesWithGeometryUncertainty({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:2,y:0},
  params,obstacles:[{
   name:'invalid camera obstacle',
   polygon:[{x:0,y:0},{x:1,y:0},{x:1,y:1},{x:0,y:1}],
   nominalPolygon:[{x:0,y:0},{x:1,y:1},{x:1,y:0},{x:0,y:1}]
  }],
  integrate,maxExpansions:100,alternativeMaxExpansions:100
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'invalid-input');
 assert.match(result.errors.join(' '),/Номинальный контур препятствия/);
});


test('multi-start planner blocks a recommendation while any constrained initial action is still unexplored',()=>{
 const result=planner.planAlternatives({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:2,y:0},
  params,obstacles:[],integrate,
  maxExpansions:3,alternativeMaxExpansions:3,
  positionTolerance:0.9,headingTolerance:0.25,
  maxTerminalArticulation:0.35,minClearance:0.30,costTolerance:0.1
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'search-incomplete');
 assert.ok(result.candidatesFound>0,'some candidate routes should be found before the budget is reached');
 assert.equal(result.candidatesAttempted,5);
 assert.equal(result.recommendation,null);
 assert.ok(result.incompleteActions.length>0);
 assert.ok(result.incompleteActions.every(item=>item.candidateIndex>1&&item.reason==='search-limit'));
});

test('reserve planning preserves an incomplete nominal search instead of reporting no route',()=>{
 const result=planner.planAlternativesWithReserve({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:2,y:0},
  params,obstacles:[],integrate,
  maxExpansions:3,alternativeMaxExpansions:3,
  measurementUncertainty:0.10
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'search-incomplete');
 assert.equal(result.sensitivity.status,'nominal-search-incomplete');
 assert.equal(result.recommendation,null);
});

test('geometry uncertainty planning preserves an incomplete nominal search',()=>{
 const result=planner.planAlternativesWithGeometryUncertainty({
  start:{x:0,y:0,psi:0,psiT:0},goal:{x:2,y:0},
  params,obstacles:[],integrate,
  maxExpansions:3,alternativeMaxExpansions:3,
  obstaclePositionUncertainty:0.05
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'search-incomplete');
 assert.equal(result.geometrySensitivity.status,'nominal-search-incomplete');
 assert.equal(result.recommendation,null);
});


test('adaptive per-wheel displacement-rate bound covers dense RK4 references in forward and reverse at high articulation',()=>{
 const p={...params,trailerTrack:2.79,footprintUncertainty:0.03};
 const obstacle={x:60,y:60,w:0.2,h:0.2,name:'dense rate reference far obstacle'};
 const stepDistance=0.12,denseSteps=96,denseStep=stepDistance/denseSteps;
 for(const articulationDeg of [-70,-35,0,35,70]){
  for(const steer of [-0.42,0,0.42]){
   for(const gear of [-1,1]){
    const start={
     x:0,y:0,psi:0.2,psiT:0.2-articulationDeg*Math.PI/180,
     gear,steer,stepDistance:0
    };
    const end=physics.rk4(start,gear*0.7,steer,stepDistance/0.7,p);
    const result=planner.analyzeWheelTrajectoryClearance([
     start,{...end,gear,steer,stepDistance}
    ],[obstacle],p);
    const segment=result.segmentDiagnostics[0];
    assert.equal(segment.status,'rk4-adaptive-intermediate-samples');
    assert.ok(Number.isFinite(segment.maxWheelCenterRatePerMeter));
    let denseState=start;
    let previous=physics.wheelFootprints(denseState,p,steer);
    for(let sample=1;sample<=denseSteps;sample++){
     denseState=physics.rk4(denseState,gear*0.7,steer,denseStep/0.7,p);
     const current=physics.wheelFootprints(denseState,p,steer);
     assert.equal(current.length,10);
     for(let wheelIndex=0;wheelIndex<current.length;wheelIndex++){
      const before=previous[wheelIndex],after=current[wheelIndex];
      assert.equal(after.name,before.name);
      const observedRate=Math.hypot(
       after.center.x-before.center.x,after.center.y-before.center.y
      )/denseStep;
      assert.ok(observedRate<=segment.maxWheelCenterRatePerMeter+1e-4,
       JSON.stringify({articulationDeg,steer,gear,wheel:after.name,observedRate,bound:segment.maxWheelCenterRatePerMeter}));
     }
     previous=current;
    }
   }
  }
 }
});


test('adaptive per-wheel interval lower bounds stay below dense reverse high-articulation clearance references',()=>{
 const p={...params,trailerTrack:2.79,footprintUncertainty:0.05};
 const articulationDeg=70,steer=-0.35,gear=-1,stepDistance=0.2,denseSteps=256;
 const start={
  x:0,y:0,psi:0.15,psiT:0.15-articulationDeg*Math.PI/180,
  gear,steer,stepDistance:0
 };
 const middle=physics.rk4(start,gear*0.7,steer,stepDistance*0.5/0.7,p);
 const target=physics.wheelFootprints(middle,p,steer).find(wheel=>
  wheel.name==='левое колесо 1-й оси полуприцепа'
 );
 assert.ok(target);
 const outward={x:-Math.sin(target.heading),y:Math.cos(target.heading)};
 const obstacle={
  x:target.center.x+outward.x*0.95,
  y:target.center.y+outward.y*0.95,
  w:0.2,h:0.2,name:'dense reverse clearance reference'
 };
 const end=physics.rk4(start,gear*0.7,steer,stepDistance/0.7,p);
 const result=planner.analyzeWheelTrajectoryClearance([
  start,{...end,gear,steer,stepDistance}
 ],[obstacle],p);
 const segment=result.segmentDiagnostics[0];
 assert.equal(segment.status,'rk4-adaptive-intermediate-samples');
 assert.ok(segment.intervalCount>4);
 const tireRadius=Math.hypot(
  p.tireLength/2+p.footprintUncertainty,
  p.tireWidth/2+p.footprintUncertainty
 );
 const denseMinimumByWheel=new Map();
 let state=start;
 for(let sample=0;sample<=denseSteps;sample++){
  if(sample>0)state=physics.rk4(state,gear*0.7,steer,
   stepDistance/denseSteps/0.7,p);
  for(const wheel of physics.wheelFootprints(state,p,steer)){
   const dx=Math.max(
    obstacle.x-obstacle.w/2-wheel.center.x,
    0,
    wheel.center.x-(obstacle.x+obstacle.w/2)
   );
   const dy=Math.max(
    obstacle.y-obstacle.h/2-wheel.center.y,
    0,
    wheel.center.y-(obstacle.y+obstacle.h/2)
   );
   const circleClearance=Math.hypot(dx,dy)-tireRadius;
   denseMinimumByWheel.set(wheel.name,Math.min(
    denseMinimumByWheel.get(wheel.name)??Infinity,circleClearance
   ));
  }
 }
 for(const wheel of result.wheels){
  const denseReference=denseMinimumByWheel.get(wheel.name);
  assert.ok(Number.isFinite(denseReference));
  assert.ok(denseReference>=0,
   wheel.name+' must remain outside the tire circumscribed-circle reference in this test fixture');
  assert.ok(wheel.minimumClearanceLowerBound<=denseReference+1e-4,
   JSON.stringify({wheel:wheel.name,lowerBound:wheel.minimumClearanceLowerBound,denseReference}));
 }
});

test('planner marks the initial transmission and steering inputs as unknown until the first control is selected',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:2,y:0},params,obstacles:[],integrate,maxExpansions:2000
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.equal(result.states[0].gearKnown,false);
 assert.equal(result.states[0].steerKnown,false);
});

test('per-wheel clearance treats the first reverse selection as an unknown initial control, not a measured gear change',()=>{
 const p={...params,footprintUncertainty:0.02};
 const start={
  x:0,y:0,psi:0.12,psiT:-0.08,gear:1,steer:0,
  gearKnown:false,steerKnown:false,stepDistance:0
 };
 const stepDistance=0.18,steer=0.18;
 const end=physics.rk4(start,-0.7,steer,stepDistance/0.7,p);
 const result=planner.analyzeWheelTrajectoryClearance([
  start,{...end,gear:-1,steer,stepDistance}
 ],[{x:50,y:50,w:0.2,h:0.2,name:'initial-selection far obstacle'}],p);
 const segment=result.segmentDiagnostics[0];
 assert.equal(segment.status,'rk4-adaptive-intermediate-samples');
 assert.equal(segment.gearTransition.kind,'initial-gear-selection');
 assert.equal(segment.gearTransition.from,null);
 assert.equal(segment.gearTransition.to,-1);
 assert.equal(segment.gearChangedAtSegmentStart,false);
 assert.equal(result.segmentsWithGearChangeAtStart,0);
 assert.equal(result.segmentsWithInitialGearSelection,1);
 assert.ok(segment.endpointResidual.position<=1e-8);
 assert.ok(Math.abs(result.distanceAlongRoute-stepDistance)<1e-12);
});

test('per-wheel clearance reconstructs both directions of an in-route gear change from the shared pose',()=>{
 const p={...params,footprintUncertainty:0.02};
 const stepDistance=0.2,steer=0.16;
 const farObstacle={x:50,y:50,w:0.2,h:0.2,name:'gear-boundary far obstacle'};
 for(const [fromGear,toGear] of [[1,-1],[-1,1]]){
  const start={
   x:0,y:0,psi:0.2,psiT:-0.15,gear:fromGear,steer,
   gearKnown:true,steerKnown:true,stepDistance:0
  };
  const end=physics.rk4(start,toGear*0.7,steer,stepDistance/0.7,p);
  const wrongDirection=physics.rk4(start,fromGear*0.7,steer,stepDistance/0.7,p);
  assert.ok(Math.hypot(end.x-wrongDirection.x,end.y-wrongDirection.y)>0.1);
  const result=planner.analyzeWheelTrajectoryClearance([
   start,{...end,gear:toGear,steer,stepDistance}
  ],[farObstacle],p);
  const segment=result.segmentDiagnostics[0];
  assert.equal(segment.status,'rk4-adaptive-intermediate-samples');
  assert.equal(segment.gearChangedAtSegmentStart,true);
  assert.deepEqual(segment.gearTransition,{
   kind:'gear-change',from:fromGear,to:toGear,at:'segment-start',
   dwellTimeModelled:false,durationSeconds:null,durationKnown:false,timeStatus:'unknown',
   requiresStopBeforeChange:true,requiresOperatorConfirmation:true,
   confirmationMode:'explicit-playback-confirmation',spatialSegmentUsesGear:toGear
  });
  assert.equal(segment.motionModel,'constant-control-spatial-segment; gear-change dwell not modelled');
  assert.ok(segment.endpointResidual.position<=1e-8);
  assert.equal(result.segmentsWithGearChangeAtStart,1);
  assert.ok(Math.abs(result.distanceAlongRoute-stepDistance)<1e-12);
 }
});

test('per-wheel clearance checks the stationary steering-control pose before the following segment moves',()=>{
 const p={...params,footprintUncertainty:0.01};
 const start={
  x:0,y:0,psi:0,psiT:0,gear:1,steer:0,
  gearKnown:true,steerKnown:true,stepDistance:0
 };
 const steer=0.55,stepDistance=0.02;
 const target=physics.wheelFootprints(start,p,steer).find(wheel=>
  wheel.name==='левое переднее колесо тягача'
 );
 assert.ok(target);
 const outerCorner=target.polygon.reduce((best,point)=>point.y>best.y?point:best);
 const obstacle={
  x:outerCorner.x,y:outerCorner.y,w:0.01,h:0.01,
  name:'steering-control-start post'
 };
 const oldPose=planner.analyzeWheelTrajectoryClearance([start],[obstacle],p);
 const oldWheel=oldPose.wheels.find(wheel=>wheel.name===target.name);
 assert.ok(oldWheel.minimumSampledPolygonClearance>0);
 const end=physics.rk4(start,0.7,steer,stepDistance/0.7,p);
 const result=planner.analyzeWheelTrajectoryClearance([
  start,{...end,gear:1,steer,stepDistance}
 ],[obstacle],p);
 const segment=result.segmentDiagnostics[0];
 const wheel=result.wheels.find(item=>item.name===target.name);
 assert.equal(segment.steeringChangedAtSegmentStart,true);
 assert.equal(segment.controlStartPoseChecked,true);
 assert.equal(result.controlStartPosesAnalyzed,1);
 assert.equal(result.controlStartWheelChecks,10);
 assert.ok(wheel.minimumIntermediatePolygonClearance<=1e-9);
 assert.equal(wheel.minimumIntermediatePolygonDetail.controlStartPose,true);
 assert.equal(wheel.minimumIntermediatePolygonDetail.sampleFraction,0);
 assert.ok(Math.abs(wheel.minimumIntermediatePolygonDetail.distanceAlongRoute)<1e-12);
});


test('wheel trajectory diagnostics expose in-route gear changes as untimed stop-and-confirm events',()=>{
 const start={
  x:0,y:0,psi:0,psiT:0,gear:1,steer:0,
  gearKnown:true,steerKnown:true,stepDistance:0
 };
 const step=0.25;
 const reverse=physics.rk4(start,-0.7,0,step/0.7,params);
 const states=[
  start,
  {...reverse,gear:-1,steer:0,gearKnown:true,steerKnown:true,stepDistance:step}
 ];
 const result=planner.analyzeWheelTrajectoryClearance(states,[],params);
 assert.equal(result.segmentsWithGearChangeAtStart,1);
 assert.equal(result.temporalEvents.length,1);
 const event=result.temporalEvents[0];
 assert.equal(event.type,'gear-change');
 assert.equal(event.from,1);
 assert.equal(event.to,-1);
 assert.equal(event.segmentIndex,1);
 assert.equal(event.requiresStopBeforeChange,true);
 assert.equal(event.requiresOperatorConfirmation,true);
 assert.equal(event.confirmationMode,'explicit-playback-confirmation');
 assert.equal(event.durationSeconds,null);
 assert.equal(event.durationKnown,false);
 assert.equal(event.timeStatus,'unknown');
 assert.equal(result.totalDurationSeconds,null);
 assert.equal(result.timeModelComplete,false);
});

test('unknown initial gear is represented as route-start selection, not a timed shift',()=>{
 const start={
  x:0,y:0,psi:0,psiT:0,gear:1,steer:0,
  gearKnown:false,steerKnown:false,stepDistance:0
 };
 const step=0.25;
 const reverse=physics.rk4(start,-0.7,0,step/0.7,params);
 const states=[
  start,
  {...reverse,gear:-1,steer:0,gearKnown:true,steerKnown:true,stepDistance:step}
 ];
 const result=planner.analyzeWheelTrajectoryClearance(states,[],params);
 assert.equal(result.segmentsWithInitialGearSelection,1);
 assert.equal(result.segmentsWithGearChangeAtStart,0);
 assert.equal(result.temporalEvents.length,1);
 const event=result.temporalEvents[0];
 assert.equal(event.type,'initial-gear-selection');
 assert.equal(event.from,null);
 assert.equal(event.to,-1);
 assert.equal(event.requiresStopBeforeChange,false);
 assert.equal(event.requiresOperatorConfirmation,true);
 assert.equal(event.confirmationMode,'route-start');
 assert.equal(event.durationSeconds,null);
 assert.equal(event.durationKnown,false);
});


test('gear transition boundary is the last waypoint reached before the new gear segment',()=>{
 const states=[
  {gear:1},{gear:1},{gear:-1},{gear:-1},{gear:1},{gear:1}
 ];
 const forwardToReverse=planner.nextGearTransitionBoundary(states,1,1);
 assert.deepEqual(forwardToReverse,{from:1,to:-1,changeIndex:2,boundaryIndex:1});
 const afterReverseConfirmation=planner.nextGearTransitionBoundary(states,1,-1);
 assert.deepEqual(afterReverseConfirmation,{from:-1,to:1,changeIndex:4,boundaryIndex:3});
 const noFurtherShift=planner.nextGearTransitionBoundary(states,3,1);
 assert.equal(noFurtherShift,null);
 const initialReverse=[{gear:1},{gear:-1},{gear:-1}];
 assert.equal(planner.nextGearTransitionBoundary(initialReverse,1,-1),null);
});
 
test('per-axle trailer tire widths are used by individual clearance and swept bounds',()=>{
 const base={
  ...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const q={x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0};
 const firstAxle=physics.trailerAxlePoints(q,base)[0];
 const obstacle={x:firstAxle.x,y:1.92,w:0.1,h:0.1,name:'axle-specific outer tire edge'};
 const single=planner.analyzeWheelTrajectoryClearance([q],[obstacle],base);
 const calibrated=planner.analyzeWheelTrajectoryClearance([q],[obstacle],{
  ...base,trailerAxle1TireGroupWidth:0.74
 });
 const first=calibrated.wheels.find(item=>item.name==='левое колесо 1-й оси полуприцепа');
 const second=calibrated.wheels.find(item=>item.name==='левое колесо 2-й оси полуприцепа');
 const baselineFirst=single.wheels.find(item=>item.name==='левое колесо 1-й оси полуприцепа');
 assert.ok(first&&second&&baselineFirst);
 assert.equal(first.footprintWidth,0.74);
 assert.equal(second.footprintWidth,0.44);
 assert.ok(first.minimumSampledPolygonClearance<baselineFirst.minimumSampledPolygonClearance);
 const baselineRate=planner.sweptDisplacementRatesPerMeter(0.2,base);
 const calibratedRate=planner.sweptDisplacementRatesPerMeter(0.2,{
  ...base,trailerAxle1TireGroupWidth:0.74
 });
 assert.ok(calibratedRate.trailer>baselineRate.trailer);
});

test('swept trailer displacement bound includes each axle track and asymmetric axle offsets',()=>{
 const q={x:0,y:0,psi:0.32,psiT:-0.14};
 const baseParams={
  ...params,
  trailerAxle1Track:2.2,trailerAxle2Track:2.2,trailerAxle3Track:2.2
 };
 const base=planner.sweptDisplacementRatesPerMeter(0.24,baseParams,q,0.05);
 const widerThird=planner.sweptDisplacementRatesPerMeter(
  0.24,{...baseParams,trailerAxle3Track:3.2},q,0.05
 );
 assert.ok(widerThird.trailer>base.trailer,
  'a wider third axle track must expand the swept displacement envelope');
});

test('per-wheel adaptive displacement rate uses the configured track of its own trailer axle',()=>{
 const measureRate=thirdAxleTrack=>{
  const p={
   ...params,trailerLength:13.6,trailerWidth:2.9,trailerAxleSpacing:1.3,
   trailerTrack:2.79,trailerAxle3Track:thirdAxleTrack,tireLength:0.72,tireWidth:0.44
  };
  const steer=0.45,stepDistance=0.8;
  const start={x:0,y:0,psi:0.25,psiT:-0.2,gear:1,steer,stepDistance:0};
  const end=physics.rk4(start,0.7,steer,stepDistance/0.7,p);
  const result=planner.analyzeWheelTrajectoryClearance(
   [start,{...end,gear:1,steer,stepDistance}],
   [{x:100,y:100,w:1,h:1,name:'distant reference obstacle'}],p
  );
  assert.equal(result.segmentsWithIntermediateKinematicSamples,1);
  const wheel=result.wheels.find(item=>item.name==='левое колесо 3-й оси полуприцепа');
  assert.ok(wheel&&wheel.limitingDetail);
  return wheel.limitingDetail.displacementRate;
 };
 const commonRate=measureRate(2.79);
 const widerThirdAxleRate=measureRate(3.4);
 assert.ok(widerThirdAxleRate>commonRate);
});
