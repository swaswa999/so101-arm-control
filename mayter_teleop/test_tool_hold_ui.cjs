const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const elements=new Map(),sent=[],documentEvents={},windowEvents={};
function element(id){if(!elements.has(id))elements.set(id,{disabled:false,events:{},classList:{toggle(){}},setAttribute(){},setPointerCapture(){},addEventListener(e,f){this.events[e]=f}});return elements.get(id)}
const add=(events)=>(e,f)=>(events[e]??=[]).push(f);
let mode='tools',blockNext=false,unblock;
const document={hidden:false,getElementById:element,addEventListener:add(documentEvents)};
const context=vm.createContext({document,window:{addEventListener:add(windowEvents)},setInterval(){},fetch:async(path,options)=>{
 if(path==='/status')return{json:async()=>({controller_version:'two-tools-jaw-fast-20261025',mode})};
 const body=JSON.parse(options.body);sent.push(body);
 if(blockNext){blockNext=false;await new Promise(resolve=>unblock=resolve)}
 return{ok:true,json:async()=>({ok:true})};
}});
vm.runInContext(fs.readFileSync(__dirname+'/index.html','utf8').split('<script>')[1].split('</script>')[0],context);
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const event=(extra={})=>({preventDefault(){},pointerId:1,button:0,isPrimary:true,repeat:false,target:{tagName:'BODY'},...extra});
const fire=(type,extra={})=>{for(const f of documentEvents[type]??[])f(event(extra))};
(async()=>{
 await settle();
 for(const [key,name,dir] of [['ArrowLeft','tire',-1],['ArrowRight','tire',1],['ArrowUp','socket',1],['ArrowDown','socket',-1]]){
  fire('keydown',{code:key});await settle();assert.equal(sent.at(-1)[name],dir);assert.equal(sent.at(-1)[name+'_speed'],2000);
  fire('keyup',{code:key});await settle();assert.equal(sent.at(-1)[name],0);
 }
 fire('keydown',{code:'ArrowLeft'});fire('keydown',{code:'ArrowUp'});await settle();await settle();assert.equal(sent.at(-1).tire,-1);assert.equal(sent.at(-1).socket,1);
 fire('keydown',{code:'ArrowRight'});await settle();assert.equal(sent.at(-1).tire,0);
 fire('keyup',{code:'ArrowLeft'});await settle();assert.equal(sent.at(-1).tire,1);
 for(const f of windowEvents.blur)f();await settle();await settle();assert.equal(sent.at(-1).tire,0);assert.equal(sent.at(-1).socket,0);
 const count=sent.length;fire('keydown',{code:'ArrowRight',repeat:true});await settle();assert.equal(sent.length,count,'Repeat after blur must not restart');
 const button=element('tireforward');blockNext=true;button.events.pointerdown(event());await settle();button.events.pointerup(event());unblock();await settle();await settle();assert.equal(sent.at(-1).tire,0);
 for(const stopEvent of ['pointercancel','lostpointercapture','blur']){button.events.pointerdown(event());await settle();button.events[stopEvent](event());await settle();assert.equal(sent.at(-1).tire,0)}
 fire('keydown',{code:'ArrowDown'});await settle();document.hidden=true;for(const f of documentEvents.visibilitychange)f();await settle();await settle();assert.equal(sent.at(-1).socket,0);
 document.hidden=false;mode='jaw_calibrating';await vm.runInContext('poll()',context);const after=sent.length;fire('keydown',{code:'ArrowLeft'});await settle();assert.equal(sent.length,after,'Arrows must not run tools during jaw calibration');
 console.log('Tool hold tests passed: all arrow mappings, max speed, independent axes, opposing keys, pointer release/cancel, blur, visibility, in-flight release, and inactive-mode guard.');
})().catch(e=>{console.error(e);process.exitCode=1});
