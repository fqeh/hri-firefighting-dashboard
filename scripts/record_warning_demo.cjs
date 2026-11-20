// Record native Gazebo and the ROS-backed dashboard through a complete evacuation.
const {chromium}=require('playwright');
const fs=require('node:fs/promises');
const {promisify}=require('node:util');
const execute=promisify(require('node:child_process').execFile);
const assert=require('node:assert/strict');
(async()=>{
 const output=process.argv[2]||'results/phase4-visible-demo';await fs.mkdir(output,{recursive:true});
 const control=await execute('python3',['scripts/forestctl.py','start','--case','D','--policy','B4',
   '--engine','expected_loss','--observe','true','--human-radius','1','--map-rate','0',
   '--lapse','0','--trust','0.9','--seed','7']);
 await fs.writeFile(output+'/launch.log',control.stdout+control.stderr);
 const browser=await chromium.launch();const errors=[];const history=[];const captures={};
 const context=await browser.newContext({viewport:{width:1440,height:1000},recordVideo:{dir:output,size:{width:1280,height:900}}});
 const world=await context.newPage(),dashboard=await context.newPage();
 for(const page of [world,dashboard])page.on('pageerror',e=>errors.push(e.message));
 const wait=async(fn,timeout=180000)=>{const end=Date.now()+timeout;while(Date.now()<end){try{const value=await fn();if(value)return value;}catch{}await new Promise(r=>setTimeout(r,250));}throw Error('Live demonstration timed out');};
 const state=async()=>{const response=await context.request.get('http://127.0.0.1:8080/api/state',{timeout:2500});return response.ok()?response.json():null;};
 const capture=async(name,s)=>{
   captures[name]=s;await fs.writeFile(`${output}/${name}.json`,JSON.stringify(s,null,2));
   await world.screenshot({path:`${output}/${name}-gazebo.png`});
   await dashboard.screenshot({path:`${output}/${name}-dashboard.png`,fullPage:true});
 };
 try{
  await wait(async()=>(await context.request.get('http://127.0.0.1:6080/vnc.html',{timeout:2000})).ok());
  await world.goto('http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale');
  await world.waitForFunction(()=>document.documentElement.classList.contains('noVNC_connected'));
  await world.waitForFunction(()=>{const c=document.querySelector('#noVNC_container canvas');if(!c)return false;
    const p=c.getContext('2d').getImageData(500,40,1,1).data;return p[0]>220&&p[1]>50&&p[1]<160&&p[2]<80;},undefined,{timeout:60000});
  await dashboard.goto('http://127.0.0.1:5173');
  await execute('python3',['scripts/forestctl.py','resume']);
  await dashboard.getByRole('heading',{name:'Warning decision',exact:true}).waitFor();
  await wait(async()=>{
   const s=await state();if(!s)return false;
   const t=s.topics['/clock']?.data.seconds||0,d=s.topics['/hri/decision']?.data,r=s.topics['/hri/rescue_route']?.data,b=s.topics['/hri/belief']?.data;
   if(!d||!r||!b)return false;
   history.push({stamp:t,decision:d,route:r,belief:b,crew:s.topics['/human/odom']?.data,uav:s.topics['/uav/odom']?.data});
   if(!captures.initial&&t<4)await capture('initial',s);
   if(!captures.blocked&&r.blocked_edges?.includes('trail_a'))await capture('blocked',s);
   if(!captures.warning&&d.warnings_sent>0)await capture('warning',s);
   if(!captures.acknowledged&&s.topics['/hri/acknowledgement'])await capture('acknowledged',s);
   const p=s.topics['/human/odom']?.data.position;
   return t>45&&p&&Math.hypot(p[0]-16,p[1]-6)<.65?s:false;
  },360000);
  await execute('python3',['scripts/forestctl.py','pause']);
  const final=await state();await capture('complete',final);
  assert.ok(captures.initial&&captures.blocked&&captures.warning&&captures.acknowledged);
  assert.ok(history.some(h=>h.route.recommended.includes('trail_a')));
  assert.ok(history.some(h=>h.route.blocked_edges.includes('trail_a')&&h.route.recommended.includes('trail_b')));
  assert.ok(history.some(h=>h.decision.observation_requests>0));
  assert.ok(history.some(h=>h.belief.last_movement_evidence));assert.deepEqual(errors,[]);
  // A paused frame can precede a pending Gazebo service timeout. Include it in
  // validation instead of accepting screenshots while a node reports failure.
  await new Promise(r=>setTimeout(r,6500));
  const session=JSON.parse(await fs.readFile('results/forest-session.json','utf8'));
  const log=await fs.readFile(session.output+'/simulator.log','utf8');
  const critical=log.split('\n').filter(l=>/\[ERROR\]|\[Err\]|Traceback|process has died|Segmentation fault|error sending response/.test(l));
  assert.deepEqual(critical,[]);
  await fs.writeFile(output+'/result.json',JSON.stringify({status:'passed',errors,
    runtime_log:session.output+'/simulator.log',critical_log_lines:critical,
    captures:Object.fromEntries(Object.entries(captures).map(([k,s])=>[k,{stamp:s.topics['/clock'].data.seconds,
      decision:s.topics['/hri/decision'].data,route:s.topics['/hri/rescue_route'].data.recommended}])),
    source:'Native Gazebo via noVNC, actual ROS telemetry, no scripted inference outputs'},null,2));
  console.log('Passed: visible forest, physical crew evacuation, blockage, alternative, posterior, OBSERVE, warning and acknowledgement.');
 }catch(error){await fs.writeFile(output+'/result.json',JSON.stringify({status:'failed',error:error.stack,errors},null,2));throw error;}
 finally{await fs.writeFile(output+'/history.json',JSON.stringify(history));await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
