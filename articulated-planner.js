(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionPlanner=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const norm=a=>Math.atan2(Math.sin(a),Math.cos(a));
 const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
 const point=(o,a,l,t=0)=>({x:o.x+l*Math.cos(a)-t*Math.sin(a),y:o.y+l*Math.sin(a)+t*Math.cos(a)});
 function polygons(q,p){
  const h=point(q,q.psi,-p.hitchOffset);
  const cab=point(q,q.psi,2.1);
  return [
   [point(q,q.psi,5,1.275),point(q,q.psi,-1.2,1.275),point(q,q.psi,-1.2,-1.275),point(q,q.psi,5,-1.275)],
   [point(cab,q.psi,1.35,1.125),point(cab,q.psi,-0.9,1.125),point(cab,q.psi,-0.9,-1.125),point(cab,q.psi,1.35,-1.125)],
   [point(h,q.psiT,1.5,p.trailerWidth/2),point(h,q.psiT,-(p.trailerLength-1.5),p.trailerWidth/2),point(h,q.psiT,-(p.trailerLength-1.5),-p.trailerWidth/2),point(h,q.psiT,1.5,-p.trailerWidth/2)]
  ];
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
 function clearance(q,obstacles,p){
  const bodies=polygons(q,p);let min=Infinity,hit=null;
  for(const o of obstacles){const op=obstaclePoly(o);for(const body of bodies){const d=polyDist(body,op);if(d<min){min=d;hit=o.name||'препятствие';}}}
  return {distance:min,hit};
 }
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
  const step=0.20,angleStep=Math.PI/60,primitive=0.35,ds=0.05;
  const maxSteer=options.maxSteer||Math.PI/7;
  const minClearance=options.minClearance??0.30;
  const key=q=>[
   Math.round(q.x/step),Math.round(q.y/step),
   Math.round(norm(q.psi)/angleStep),Math.round(norm(q.psiT)/angleStep)
  ].join(':');
  const h=q=>Math.hypot(goal.x-q.x,goal.y-q.y)+0.35*Math.abs(norm(q.psi-q.psiT));
  const startClearance=clearance(start,obstacles,params).distance;
  if(startClearance===0)return {ok:false,reason:'start-collision',expanded:0};
  const requiredClearance=Math.min(minClearance,startClearance);
  const initial={q:{...start},g:0,f:h(start),parent:null,gear:1,steer:0,samples:[]};
  const open=[];push(open,initial);
  const best=new Map([[key(start),0]]);
  let expanded=0,finish=null,closest=initial;
  const steering=[-maxSteer,-0.24,-0.12,0,0.12,0.24,maxSteer];
  const controls=[];
  for(const gear of [-1,1])for(const steer of steering)controls.push([gear,steer]);
  while(open.length&&expanded<maxExpansions){
   const cur=pop(open),ck=key(cur.q);
   if(cur.g>(best.get(ck)??Infinity)+1e-8)continue;
   expanded++;
   const dist=Math.hypot(goal.x-cur.q.x,goal.y-cur.q.y);
   if(dist<Math.hypot(goal.x-closest.q.x,goal.y-closest.q.y))closest=cur;
   if(dist<0.9){finish=cur;break;}
   for(const [gear,steer] of controls){
    let q={...cur.q},valid=true,minD=Infinity,samples=[];
    for(let d=0;d<primitive;d+=ds){
     q=options.integrate(q,gear*0.7,steer,Math.min(ds,primitive-d),params);
     if(q.x<bounds.minX||q.x>bounds.maxX||q.y<bounds.minY||q.y>bounds.maxY||
        Math.abs(norm(q.psi-q.psiT))>Math.PI*0.92){valid=false;break;}
     const c=clearance(q,obstacles,params);minD=Math.min(minD,c.distance);samples.push({x:q.x,y:q.y,psi:q.psi,psiT:q.psiT,gear,steer});
     if(c.distance<requiredClearance){valid=false;break;}
    }
    if(!valid)continue;
    const nk=key(q);
    const gearPenalty=gear<0?0.55:0;
    const steerPenalty=Math.abs(steer)*0.08+(cur.steer!==steer?0.10:0);
    const ng=cur.g+primitive+gearPenalty+steerPenalty;
    if(ng>=(best.get(nk)??Infinity))continue;
    best.set(nk,ng);
    push(open,{q,g:ng,f:ng+h(q),parent:cur,gear,steer,minClearance:minD,samples});
   }
  }
  if(!finish)return {ok:false,reason:'no-path',expanded,closest:{x:closest.q.x,y:closest.q.y},goalDistance:Math.hypot(goal.x-closest.q.x,goal.y-closest.q.y)};
  const chain=[];for(let n=finish;n&&n.parent;n=n.parent)chain.push(n);
  chain.reverse();
  const states=[{x:start.x,y:start.y,psi:start.psi,psiT:start.psiT,gear:1,steer:0}];
  for(const n of chain)states.push(...n.samples);
  return {ok:true,states,expanded,length:finish.g,minimumClearance:Math.min(...states.slice(1).map((v)=>clearance(v,obstacles,params).distance))};
 }
 return Object.freeze({plan,clearance,polygons});
});
