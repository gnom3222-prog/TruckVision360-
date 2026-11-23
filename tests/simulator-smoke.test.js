'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const physics=require('../vehicle-physics.js');

function bootSimulator(){
 const root=path.join(__dirname,'..');
 const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
 const source=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 const values={};
 for(const match of html.matchAll(/<input\b[^>]*\bid="([^"]+)"[^>]*>/g)){
  const value=match[0].match(/\bvalue="([^"]*)"/);
  values[match[1]]=value?value[1]:'';
 }
 values.model='hybrid';
 const elements={};
 const ctx={};
 for(const name of ['beginPath','moveTo','lineTo','setLineDash','stroke','closePath','fill','fillText','arc','clearRect','fillRect']){
  ctx[name]=()=>{};
 }
 const getElementById=id=>{
  if(!elements[id]){
   elements[id]={id,value:values[id]??'',textContent:'',style:{},onclick:null,oninput:null};
  }
  return elements[id];
 };
 const canvas=getElementById('view');
 canvas.width=1000;
 canvas.height=650;
 canvas.getContext=()=>ctx;
 const windowObject={TruckVisionPhysics:physics,addEventListener:()=>{}};
 const sandbox={
  window:windowObject,
  document:{getElementById},
  requestAnimationFrame:()=>{},
  console
 };
 vm.runInNewContext(source,sandbox,{filename:'truckvision.js'});
 return elements;
}

test('simulator starts and exposes route controls',()=>{
 const elements=bootSimulator();
 assert.equal(typeof elements.planRoute.onclick,'function');
 assert.equal(typeof elements.routeStart.onclick,'function');
 assert.equal(typeof elements.routePause.onclick,'function');
 assert.equal(typeof elements.routeStop.onclick,'function');
});

test('automatic route remains blocked when obstacle checks are disabled',()=>{
 const elements=bootSimulator();
 elements.obstaclesToggle.onclick();
 elements.planRoute.onclick();
 assert.match(elements.difficultyAdvice.textContent,/проверка препятствий отключена/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Автопроезд заблокирован/);
});
