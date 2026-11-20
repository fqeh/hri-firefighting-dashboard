// Real HTTP/ROS integration: experiment lifecycle, UAV commands, reconnect,
// truth-only artifact access, policy changes and industrial fire/gas updates.
const {chromium}=require('playwright');const fs=require('node:fs/promises');const assert=require('node:assert/strict');
(async()=>{
 const output=process.argv[2]||'results/phase6-controls';await fs.mkdir(output,{recursive:true});
 const browser=await chromium.launch(),context=await browser.newContext({viewport:{width:1366,height:900}}),page=await context.newPage();const errors=[],checks={},logs=[];
 page.on('pageerror',e=>errors.push(e.message));
 const get=async(path)=>{const r=await context.request.get('http://127.0.0.1:5173'+path,{timeout:3000});return r.json();};
 const wait=async(fn,ms=150000)=>{const end=Date.now()+ms;let error;while(Date.now()<end){try{const x=await fn();if(x)return x;}catch(e){error=e.message;}await page.waitForTimeout(250);}throw Error('Condition timed out: '+error);};
 const state=()=>get('/api/state'),session=()=>get('/api/session');
 let result={status:'failed'};
 try{
  await page.goto('http://127.0.0.1:5173',{waitUntil:'domcontentloaded'});
  await page.getByLabel('Scenario',{exact:true}).selectOption('forest');await page.getByLabel('Scenario case',{exact:true}).selectOption('F');await page.getByLabel('Warning policy',{exact:true}).selectOption('B1');
  await page.getByLabel('Manual test incidents',{exact:true}).check();await page.getByLabel('Trial duration (simulation seconds)').fill('15');
  const old=(await session()).session.output;
  await page.getByRole('button',{name:'Start experiment',exact:true}).click();
  await wait(async()=>{const s=await session();return !s.busy&&s.session.output!==old&&s.experiment?.state==='running';});
  await wait(async()=>{const s=await state();return s.topics['/hri/decision']?.data.policy==='B1'&&s.topics['/uav/odom'];});checks.policy_change=true;
  await page.getByLabel('Waypoint X',{exact:true}).fill('5');await page.getByLabel('Waypoint Y',{exact:true}).fill('5');await page.getByLabel('Waypoint Z',{exact:true}).fill('9');
  await page.getByRole('button',{name:'Set UAV waypoint',exact:true}).click();await wait(async()=>{const s=await state();return JSON.stringify(s.topics['/hri/uav_status']?.data.goal)==='[5,5,9]';});checks.uav_waypoint=true;
  const trial=await wait(async()=>{const s=await session();return !s.busy&&s.experiment?.state==='stopped'&&s.experiment?.progress===1?s:false;});checks.sim_time_experiment_completion=true;logs.push(trial.session.output);
  assert.ok(trial.artifacts.some(a=>a.name==='dashboard-experiment.json'));checks.log_links=true;
  const clock=(await state()).topics['/clock'].data.seconds;await page.waitForTimeout(1500);assert.ok(Math.abs((await state()).topics['/clock'].data.seconds-clock)<.05);
  const evaluationToggle=page.getByLabel('Enable evaluation truth counters',{exact:true});
  const evaluationInitiallyEnabled=(await session()).evaluation_enabled;
  if(!evaluationInitiallyEnabled)await evaluationToggle.click();
  const evalState=await wait(async()=>{const s=await session();return s.evaluation?.available?s:false;});
  assert.ok(Number.isFinite(evalState.evaluation.false_alarms));checks.evaluation_truth_counters=true;
  await evaluationToggle.click();
  await wait(async()=>!(await session()).evaluation_enabled);
  const denied=await context.request.get('http://127.0.0.1:5173/api/session/artifacts/'+trial.session.output+'/events.jsonl');assert.equal(denied.status(),403);
  const publicLog=await context.request.get('http://127.0.0.1:5173/api/session/artifacts/'+trial.session.output+'/public-events.jsonl');assert.ok(publicLog.ok());assert.ok(!(await publicLog.text()).includes('"topic": "/evaluation/human_belief"'));checks.hidden_belief_log_boundary=true;
  if(evaluationInitiallyEnabled){
    const restored=await context.request.post('http://127.0.0.1:5173/api/session/control',{data:{command:'evaluation',enabled:true}});
    assert.ok(restored.ok());await wait(async()=>(await session()).evaluation_enabled);
  }
  await page.screenshot({path:output+'/experiment-complete.png',fullPage:true});
  const abortEvents=route=>route.abort();
  await page.route('**/api/events',abortEvents);
  await page.reload({waitUntil:'domcontentloaded'});
  await wait(async()=>/RECONNECTING/.test(await page.locator('.connection').innerText()),10000);checks.disconnection=true;
  await page.unroute('**/api/events',abortEvents);
  await wait(async()=>/PAUSED/.test(await page.locator('.connection').innerText()),30000);checks.reconnection=true;
  const previous=(await state()).epoch;
  await page.getByRole('button',{name:'Reset run',exact:true}).click();await wait(async()=>{const s=await session();return !s.busy&&s.session.output!==trial.session.output;});await wait(async()=>(await state()).epoch!==previous);checks.full_reset=true;
  await page.getByLabel('Scenario',{exact:true}).selectOption('oil_facility');await page.getByLabel('Scenario case',{exact:true}).selectOption('combined');await page.getByLabel('Warning policy',{exact:true}).selectOption('B4');
  const old2=(await session()).session.output;await page.getByRole('button',{name:'Load scenario & policy',exact:true}).click();await wait(async()=>{const s=await session();return !s.busy&&s.session.output!==old2&&s.session.scenario==='oil_facility';});await page.getByRole('button',{name:'Resume simulation',exact:true}).click();await wait(async()=>(await state()).topics['/hri/facility']);
  await page.getByLabel('Destination',{exact:true}).selectOption('assembly');await page.getByLabel('Preferred corridor',{exact:true}).selectOption('corridor_b');await page.getByRole('button',{name:'Request route recommendation',exact:true}).click();
  for(const id of ['hazard_fire','hazard_gas']){await page.getByLabel('Test incident',{exact:true}).selectOption(id);await page.getByRole('button',{name:'Inject test incident',exact:true}).click();await wait(async()=>(await state()).topics['/hri/observations']?.data.hazards.some(h=>h.id===id));}
  const s=await wait(async()=>{
    const snapshot=await state(),edges=snapshot.topics['/hri/robot_route']?.data.edge_states;
    return edges?.corridor_b==='hazardous'&&edges?.corridor_c==='hazardous'?snapshot:false;
  });
  assert.equal(s.topics['/hri/robot_route'].data.edge_states.corridor_b,'hazardous');assert.equal(s.topics['/hri/robot_route'].data.edge_states.corridor_c,'hazardous');checks.fire_and_gas_map=true;
  await page.waitForTimeout(800);await page.screenshot({path:output+'/fire-gas-dashboard.png',fullPage:true});await fs.writeFile(output+'/fire-gas.json',JSON.stringify(s,null,2));
  await page.getByRole('button',{name:'Pause simulation',exact:true}).click();await wait(async()=>!(await session()).busy);logs.push((await session()).session.output);
  assert.deepEqual(errors,[]);result={status:'passed',checks,errors,logs};
 }catch(e){result.error=e.stack;result.checks=checks;result.errors=errors;process.exitCode=1;await page.screenshot({path:output+'/failure.png',fullPage:true}).catch(()=>{});}
 finally{await fs.writeFile(output+'/result.json',JSON.stringify(result,null,2));console.log(JSON.stringify(result,null,2));const timeout=setTimeout(()=>process.exit(process.exitCode||0),10000);await page.close();await context.close();await browser.close();clearTimeout(timeout);}
})();
