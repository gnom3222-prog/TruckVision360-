'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');

const root=path.resolve(__dirname,'..');
const read=name=>fs.readFileSync(path.join(root,name),'utf8');

test('service worker precaches exact versioned URLs used by the page and planner worker',()=>{
 const html=read('index.html');
 const app=read('truckvision.js');
 const worker=read('planner-worker.js');
 const sw=read('sw.js');
 const match=sw.match(/const CORE\s*=\s*\[([\s\S]*?)\];/);
 assert.ok(match,'Service Worker CORE list must exist');
 const core=Array.from(match[1].matchAll(/'([^']+)'/g),item=>item[1]);
 const required=[];
 for(const item of html.matchAll(/<script\s+src=["']([^"']+)["']/g)){
  if(!/^https?:/i.test(item[1]))required.push(item[1].startsWith('.')?item[1]:'./'+item[1]);
 }
 const workerUrl=app.match(/new Worker\(["']([^"']+)["']/);
 assert.ok(workerUrl,'Planner worker URL must be explicit');
 required.push(workerUrl[1]);
 const imports=worker.match(/importScripts\(([^)]*)\)/);
 assert.ok(imports,'Planner worker imports must be explicit');
 for(const item of imports[1].matchAll(/["']([^"']+)["']/g))required.push(item[1]);
 const missing=required.filter(url=>!core.includes(url));
 assert.deepEqual(missing,[]);
});

test('planner worker imports match the current page physics and planner versions',()=>{
 const html=read('index.html');
 const worker=read('planner-worker.js');
 for(const file of ['vehicle-physics.js','articulated-planner.js']){
  const pageUrl=html.match(new RegExp(`<script\\s+src=["'](${file}\\?v=[^"']+)["']`));
  const workerUrl=worker.match(new RegExp(`["'](\\./${file}\\?v=[^"']+)["']`));
  assert.ok(pageUrl,'Page must version '+file);
  assert.ok(workerUrl,'Worker must version '+file);
  assert.equal('./'+pageUrl[1],workerUrl[1]);
 }
});
