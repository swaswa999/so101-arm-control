const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const elements = new Map(), sent = [];
function element(id) {
 if (!elements.has(id)) elements.set(id, {value:'300',disabled:false,events:{},classList:{toggle(){}},setAttribute(){},setPointerCapture(){},addEventListener(type,fn){this.events[type]=fn}});
 return elements.get(id);
}
let unblock;
const context = vm.createContext({document:{getElementById:element,hidden:false,addEventListener(){}},window:{addEventListener(){}},setInterval(){},fetch:async(path,options)=>{
 if (path==='/status') return {json:async()=>({controller_version:'two-tools-jaw-hold-20261025',mode:'jaw_calibrating'})};
 const body=JSON.parse(options.body);sent.push(body);
 if (body.jaw===1 && !unblock) await new Promise(resolve=>unblock=resolve);
 return {ok:true,json:async()=>({ok:true})};
}});
vm.runInContext(fs.readFileSync(__dirname+'/index.html','utf8').split('<script>')[1].split('</script>')[0],context);
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const event=(extra={})=>({preventDefault(){},pointerId:1,button:0,isPrimary:true,...extra});
(async()=>{
 await settle();
 const forward=element('jawforward'),backward=element('jawbackward');
 forward.events.pointerdown(event());await settle();assert.equal(sent.at(-1).jaw,1);
 // Release while the press request is still in flight must send zero immediately afterward.
 forward.events.pointerup(event());unblock();await settle();await settle();assert.equal(sent.at(-1).jaw,0);
 backward.events.pointerdown(event());await settle();assert.equal(sent.at(-1).jaw,-1);
 backward.events.pointercancel(event());await settle();assert.equal(sent.at(-1).jaw,0);
 forward.events.keydown(event({code:'Space',repeat:false}));await settle();assert.equal(sent.at(-1).jaw,1);
 forward.events.keyup(event({code:'Space'}));await settle();assert.equal(sent.at(-1).jaw,0);
 backward.events.pointerdown(event());await settle();
 backward.events.lostpointercapture(event());await settle();assert.equal(sent.at(-1).jaw,0);
 forward.events.pointerdown(event());await settle();forward.events.blur();await settle();assert.equal(sent.at(-1).jaw,0);
 console.log('Hold UI: pointer release, cancellation, lost capture, keyboard release, blur, and in-flight release passed.');
})().catch(error=>{console.error(error);process.exitCode=1});
