// Live forest-to-industrial acceptance: dashboard controls and actual ROS telemetry.
// Fixture data is never injected into this integrated demonstration.
const {chromium}=require('playwright');
const fs=require('node:fs/promises');const assert=require('node:assert/strict');
(async()=>{
 const output=process.argv[2]||'results/phase6-visible';await fs.mkdir(output,{recursive:true});
 const browser=await chromium.launch();const context=await browser.newContext({viewport:{width:1440,height:1000},recordVideo:{dir:output,size:{width:1440,height:1000}}});
 const page=await context.newPage(),world=await context.newPage(),errors=[],history=[],captures={};
 page.on('pageerror',e=>errors.push(e.message));world.on('pageerror',e=>errors.push(e.message));
 const get=async(path)=>{const response=await context.request.get('http://127.0.0.1:5173'+path,{timeout:3000});if(!response.ok())throw Error(`${path} ${response.status()}`);return response.json();};
 const state=()=>get('/api/state'),session=()=>get('/api/session');
 const wait=async(fn,timeout=150000)=>{const deadline=Date.now()+timeout;let last;while(Date.now()<deadline){try{const x=await fn();if(x)return x;}catch(e){last=e.message;}await new Promise(r=>setTimeout(r,250));}throw Error('Timed out: '+last);};
 const capture=async(name)=>{const s=await state();captures[name]={clock:s.topics['/clock']?.data.seconds,epoch:s.epoch};await fs.writeFile(`${output}/${name}.json`,JSON.stringify(s,null,2));await page.screenshot({path:`${output}/${name}-dashboard.png`,fullPage:true});await world.screenshot({path:`${output}/${name}-gazebo.png`});return s;};
 const load=async(scenario,caseName)=>{
  await page.getByLabel('Scenario',{exact:true}).selectOption(scenario);
  const scenarioCase=page.getByLabel('Scenario case',{exact:true});
  await scenarioCase.locator(`option[value="${caseName}"]`).waitFor({state:'attached'});
  await scenarioCase.selectOption(caseName);
  await page.getByLabel('Warning policy',{exact:true}).selectOption('B4');await page.getByLabel('Manual test incidents',{exact:true}).check();
  if(scenario==='forest'){await page.getByLabel('Crew observation radius',{exact:true}).fill('1');await page.getByLabel('Passive map glances',{exact:true}).fill('0');await page.getByLabel('Human decision lapse',{exact:true}).fill('0');}
  const old=(await session()).session.output;
  await page.getByRole('button',{name:'Load scenario & policy',exact:true}).click();
  await wait(async()=>{const s=await session();if(s.error)throw Error(s.error);return !s.busy&&s.session.output!==old&&s.session.scenario===scenario;});
  await wait(async()=>{const s=await state();return s.schema_version===1?s:false;});
  await wait(async()=>(await context.request.get('http://127.0.0.1:6080/vnc.html',{timeout:3000})).ok());
  await world.goto('http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale',{waitUntil:'domcontentloaded',timeout:60000});
  await world.waitForFunction(()=>document.documentElement.classList.contains('noVNC_connected'),null,{timeout:60000});
  await page.getByRole('button',{name:'Resume simulation',exact:true}).click();
  return wait(async()=>{const s=await state();return s.connected&&s.topics['/hri/belief']&&s.topics[scenario==='forest'?'/uav/camera/image':'/robot/camera/image']?s:false;});
 };
 let result={status:'failed',errors};
 try{
  await page.goto('http://127.0.0.1:5173');await page.getByRole('button',{name:'Load scenario & policy'}).waitFor();
  const first=await load('forest','F');const initialCrew=first.topics['/human/odom'].data.position,initialUav=first.topics['/uav/odom'].data.position;
  const moving=await wait(async()=>{const s=await state();history.push(s);const h=s.topics['/human/odom']?.data.position,u=s.topics['/uav/odom']?.data.position;return h&&u&&Math.hypot(h[0]-initialCrew[0],h[1]-initialCrew[1])>.5&&Math.hypot(u[0]-initialUav[0],u[1]-initialUav[1])>.3?s:false;});
  assert.equal((moving.topics['/hri/observations']?.data.hazards||[]).filter(h=>h.kind==='tree').length,0);
  await capture('forest-moving');
  await page.getByRole('button',{name:'Inject test incident',exact:true}).click();
  const blocked=await wait(async()=>{const s=await state();history.push(s);return s.topics['/hri/rescue_route']?.data.blocked_edges.includes('trail_a')?s:false;});
  assert.ok(!blocked.topics['/hri/rescue_route'].data.edges.includes('trail_a'));
  await wait(async()=>{const s=await state();history.push(s);return s.warnings.length&&s.topics['/hri/rescue_route'].data.recommended.includes('trail_b')?s:false;});
  await page.locator('.warning-card').first().waitFor();await page.waitForTimeout(800);await capture('forest-warning');
  await page.getByRole('button',{name:'Acknowledge',exact:true}).first().click();
  await wait(async()=>{const s=await state();return s.warnings.some(w=>w.acknowledged)&&s.topics['/hri/operator_receipt']?s:false;});
  await page.getByRole('button',{name:'Pause simulation',exact:true}).click();await wait(async()=>!(await session()).busy);await capture('forest-acknowledged');
  const t=(await state()).topics['/clock'].data.seconds;await page.waitForTimeout(1500);assert.ok(Math.abs((await state()).topics['/clock'].data.seconds-t)<.05);
  const forestSession=await session();await fs.writeFile(output+'/forest-session.json',JSON.stringify(forestSession,null,2));
  const oil=await load('oil_facility','blockage');assert.notEqual(oil.epoch,first.epoch);assert.equal(oil.warnings.length,0);
  await wait(async()=>{const s=await state();return s.topics['/hri/navigation']?.data.state==='ready'?s:false;});
  await page.getByLabel('Destination',{exact:true}).selectOption('assembly');await page.getByLabel('Preferred corridor',{exact:true}).selectOption('corridor_b');
  await page.getByRole('button',{name:'Request route recommendation',exact:true}).click();
  await wait(async()=>{const s=await state();return s.topics['/hri/robot_route']?.data.request_id>0&&s.topics['/hri/navigation']?s:false;});
  const approve=async()=>{
   await wait(async()=>{
    const s=await state(),navigation=s.topics['/hri/navigation']?.data;
    return navigation&&!['requested','route_approved','navigating','canceling'].includes(navigation.state)?s:false;
   });
   await wait(async()=>{const s=await state(),r=s.topics['/hri/robot_route'].data;return Number(await page.getByRole('button',{name:'Approve recommended route',exact:true}).getAttribute('data-route-revision'))===r.revision&&await page.getByRole('button',{name:'Approve recommended route',exact:true}).isEnabled();});
   await page.getByRole('button',{name:'Approve recommended route',exact:true}).click();
  };
  await approve();
  await wait(async()=>{const s=await state();history.push(s);return s.topics['/robot/odom']?.data.position[0]>2.4?s:false;});await capture('oil-moving');
  await page.getByRole('button',{name:'Inject test incident',exact:true}).click();
  await wait(async()=>{const s=await state();history.push(s);return s.topics['/hri/robot_route']?.data.blocked_edges.includes('corridor_b')?s:false;});
  await page.getByRole('button',{name:'Request route recommendation',exact:true}).click();
  await wait(async()=>{
   const s=await state();history.push(s);
   const warning=s.warnings.find(w=>w.warning.edge==='corridor_b');
   return s.topics['/hri/robot_route'].data.edges.includes('corridor_c')&&
    warning?.warning.recommended?.includes('north_junction')&&
    !warning.warning.recommended.includes('corridor_b')&&
    warning.warning.explanation.includes('Recommended alternative: junction → north_junction')?s:false;
  });
  await page.waitForTimeout(1000);await capture('oil-warning');
  await page.getByRole('button',{name:'Acknowledge',exact:true}).first().click();await approve();
  await wait(async()=>{const s=await state();history.push(s);return s.topics['/robot/odom'].data.position[1]>2?s:false;},180000);
  await capture('oil-approved-detour');
  await wait(async()=>{const s=await state(),p=s.topics['/robot/odom']?.data.position;history.push(s);return p&&Math.hypot(p[0]-15,p[1]-6)<.4&&s.topics['/hri/navigation']?.data.state==='succeeded'?s:false;},240000);
  await capture('oil-arrival');
  await page.getByRole('button',{name:'■ Emergency stop',exact:true}).click();await wait(async()=>{const s=await state();return s.topics['/robot/emergency_stop_active']?.data.active===true;});
  const stopped=(await state()).topics['/robot/odom'].data.position;await page.waitForTimeout(1200);const stopped2=(await state()).topics['/robot/odom'].data.position;assert.ok(Math.hypot(stopped2[0]-stopped[0],stopped2[1]-stopped[1])<.15);
  await page.getByRole('button',{name:'Cancel navigation',exact:true}).click();await page.getByRole('button',{name:'Release',exact:true}).click();await wait(async()=>!(await state()).topics['/robot/emergency_stop_active'].data.active);
  await page.getByRole('button',{name:'Pause simulation',exact:true}).click();await wait(async()=>!(await session()).busy);await capture('oil-paused');
  await page.setViewportSize({width:1280,height:800});await page.screenshot({path:output+'/laptop-dashboard.png',fullPage:true});assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
  assert.deepEqual(errors,[]);const finalSession=await session();await fs.writeFile(output+'/oil-session.json',JSON.stringify(finalSession,null,2));
  result={status:'passed',errors,captures,checks:{forest_movement:true,manual_physical_incident:true,blocked_route_rejected:true,alternative_route:true,posterior_live:true,warning:true,supervisor_acknowledgement:true,pause_clock:true,scenario_switch_resets_epoch:true,oil_nav2_movement:true,operator_approval:true,oil_mission_completion:true,emergency_stop:true,laptop_layout:true},logs:[forestSession.session.output,finalSession.session.output]};
 }catch(e){result.error=e.stack;await page.screenshot({path:output+'/failure.png',fullPage:true}).catch(()=>{});process.exitCode=1;}
 finally{await fs.writeFile(output+'/history.json',JSON.stringify(history));await fs.writeFile(output+'/result.json',JSON.stringify(result,null,2));console.log(JSON.stringify(result,null,2));const closeTimeout=setTimeout(()=>process.exit(process.exitCode||0),15000);await page.close().catch(()=>{});await world.close().catch(()=>{});await context.close();await browser.close();clearTimeout(closeTimeout);}
})();
