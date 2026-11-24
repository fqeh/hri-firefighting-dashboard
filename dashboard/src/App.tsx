import {useEffect,useMemo,useRef,useState,type ReactNode} from 'react';
import {arr,obj,num,str,fmt,human,percent,point,type Control,type SessionControl,type SessionSelection,type Topic} from './contracts';
import {useTelemetry,useSession,useScene,sendControl,saveEquipmentContents} from './transport';
import {SourceTag,Stamp,Fact,Camera,EquipmentRegistryPanel,WarningCard,ConceptChips} from './components';
import {DecisionPanel,LookoutStatus,CrewBeliefPanel,EscalationAlert,policyName} from './insight';
import {WorldView,LayerList,Icon,liveView,useLayers,type ViewMode} from './scene3d';

const DEFAULT_SELECTION:SessionSelection={scenario:'forest',case:'F',policy:'B4',engine:'expected_loss',seed:7,manual_incidents:true,human_radius:3,map_rate:.05,human_lapse:.03};
// Limited crew awareness (1 m sight, no map glances, no lapses) makes the false-belief mechanism observable.
const DEMO_CREW={human_radius:1,map_rate:0,human_lapse:0,manual_incidents:false,seed:7,engine:'expected_loss' as const};
const CASE_LABELS:Record<string,string>={A:'A · Normal evacuation',B:'B · Growing fire',C:'C · Fallen tree',D:'D · Fire + fallen tree',E:'E · Tree, crew already knows',F:'F · Tree, crew unaware'};
type Section='scenario'|'layers'|'robot'|'experiment';
type RightSection='explain'|'log';
const SECTIONS:[Section,string,ReactNode][]=[['scenario','Scenario',Icon.sliders],['layers','Layers',Icon.layers],['robot','Robot',Icon.robot],['experiment','Experiment',Icon.flask]];
const RIGHT_SECTIONS:[RightSection,string,ReactNode][]=[['explain','Explain',Icon.explain],['log','Log',Icon.list]];
// Per-browser layout preferences; storage can be unavailable (private mode), so every access is guarded.
function stored<T extends string>(key:string,allowed:readonly T[],fallback:T):T{try{const v=localStorage.getItem(key);return allowed.includes(v as T)?v as T:fallback;}catch{return fallback;}}
function remember(key:string,value:string){try{localStorage.setItem(key,value);}catch{/* storage unavailable */}}

export default function App(){
  const telemetry=useTelemetry(),supervisor=useSession(),session=supervisor.session;
  const [selection,setSelection]=useState<SessionSelection>(DEFAULT_SELECTION),[sending,setSending]=useState(false),[notice,setNotice]=useState('');
  const [duration,setDuration]=useState(90),[edge,setEdge]=useState(''),[destination,setDestination]=useState('assembly');
  const [x,setX]=useState(''),[y,setY]=useState(''),[z,setZ]=useState('8'),[incident,setIncident]=useState('');
  const [blockedEpoch,setBlockedEpoch]=useState('');const initialized=useRef(false);
  const [section,setSection]=useState<Section>(()=>stored('fieldwatch-section',SECTIONS.map(s=>s[0]),'scenario'));
  const [sideOpen,setSideOpen]=useState(true),[explainOpen,setExplainOpen]=useState(()=>typeof window==='undefined'||window.innerWidth>1100);
  const [rightSection,setRightSection]=useState<RightSection>('explain');
  const openRight=(s:RightSection)=>{if(s===rightSection&&explainOpen){setExplainOpen(false);return;}setRightSection(s);setExplainOpen(true);};
  const [viewMode,setViewMode]=useState<ViewMode>(()=>stored('fieldwatch-view-mode',['web','gazebo'] as const,'web'));
  const [layers,setLayer]=useLayers();
  const openSection=(s:Section)=>{if(s===section&&sideOpen){setSideOpen(false);return;}setSection(s);setSideOpen(true);remember('fieldwatch-section',s);};
  const chooseView=(m:ViewMode)=>{setViewMode(m);remember('fieldwatch-view-mode',m);};
  useEffect(()=>{if(session&&!initialized.current&&session.session.scenario){initialized.current=true;const s=session.session;setSelection({scenario:s.scenario==='oil_facility'?'oil_facility':'forest',case:str(s.case,'F'),policy:str(s.policy,'B4'),engine:s.engine==='value_iteration'?'value_iteration':'expected_loss',seed:num(s.seed)??7,manual_incidents:s.manual_incidents===true,human_radius:num(s.human_radius)??3,map_rate:num(s.map_rate)??.05,human_lapse:num(s.human_lapse)??.03});}},[session]);
  useEffect(()=>{if(session?.busy&&['switch','reset','start_experiment'].includes(str(session.operation))&&telemetry.state)setBlockedEpoch(telemetry.state.epoch);},[session?.busy]);
  const restarting=session?.busy&&['switch','reset','start_experiment'].includes(str(session.operation));
  const state=restarting||telemetry.state?.epoch===blockedEpoch?null:telemetry.state;
  const topics:Record<string,Topic>=Object.fromEntries(Object.entries(state?.topics??{}).map(([key,t])=>[key,{...t,stale:t.stale||!telemetry.online}])),data=(name:string)=>obj(topics[name]?.data);
  const params=session?.parameters??{},sources=obj(data('/hri/status').sources),scenarioStatus=obj(sources.scenario);
  const scenario=str(scenarioStatus.scenario,str(session?.session.scenario,'Awaiting scenario')),oil=scenario==='oil_facility';
  const online=telemetry.online&&Boolean(state),live=online&&Boolean(state?.connected),paused=session?.session.paused===true;
  const robotRoute=data('/hri/robot_route'),decision=data('/hri/decision'),intent=data('/hri/human_intent');
  const nav=data('/hri/navigation'),uav=data('/hri/uav_status'),robot=data(oil?'/robot/odom':'/uav/odom'),robotTopic=topics[oil?'/robot/odom':'/uav/odom'];
  const clock=num(data('/clock').seconds),warnings=state?.warnings??[];
  const active=[...warnings].reverse().find(w=>!w.acknowledged&&!w.dismissed);
  const latch=topics['/robot/emergency_stop_active'],stopConfirmed=online&&latch&&!latch.stale&&typeof latch.data.active==='boolean',stopped=stopConfirmed&&latch.data.active===true;
  const coordinate=point(robot.position),goal=oil?obj(nav.current_waypoint):{},uavGoal=arr(uav.goal);
  const options=obj(data('/hri/facility').destinations),incidents=arr(params.incidents).map(obj);
  const navAllowed=live&&!paused&&!sending&&!session?.busy&&oil&&robotRoute.navigation_allowed===true&&!topics['/hri/robot_route']?.stale&&num(robotRoute.revision)!==null;
  const sessionLocked=sending||Boolean(session?.busy)||Boolean(supervisor.error)||!session;
  async function command(c:Control|SessionControl,sessionControl=false){
    setSending(true);setNotice('');
    try{await sendControl(c,sessionControl);setNotice(`${human(c.command)} accepted by ${sessionControl?'session supervisor':'ROS gateway'}. Await the confirmed state.`);}
    catch(e){setNotice(String(e));}finally{setSending(false);}
  }
  const send=(c:Control)=>{void command(c);},control=(c:SessionControl)=>{void command(c,true);};
  // Presets only fill the form; nothing reaches the simulator until "Load scenario & policy".
  const preset=(caseId:'E'|'F')=>{setSelection({...selection,...DEMO_CREW,scenario:'forest',case:caseId,policy:'B4'});setNotice(`Preset ${caseId==='E'?'“Crew knows”':'“Crew unaware”'} filled in. Press Load scenario & policy to apply it.`);};
  const loaded=session?.session??{};
  const pending=Boolean(session)&&(['scenario','case','policy','engine','seed','manual_incidents'] as const).some(k=>String(loaded[k]??'')!==String(selection[k]))
    ||selection.scenario==='forest'&&(['human_radius','map_rate','human_lapse'] as const).some(k=>num(loaded[k])!==null&&num(loaded[k])!==selection[k]);
  const presetActive=(caseId:'E'|'F')=>selection.scenario==='forest'&&selection.case===caseId&&selection.policy==='B4'&&(Object.keys(DEMO_CREW) as (keyof typeof DEMO_CREW)[]).every(k=>selection[k]===DEMO_CREW[k]);
  useEffect(()=>{setIncident('');setEdge('');},[session?.session.generation]);
  const stopResult=data('/dashboard/emergency_stop_result');
  // Incidents already in this run (each exists once). Remembered per run because the
  // gateway timeline is a short rolling window that fire-front detections can fill.
  const injectedRef=useRef<{epoch:string;ids:Map<string,number>}>({epoch:'',ids:new Map()});
  if(state&&injectedRef.current.epoch!==state.epoch)injectedRef.current={epoch:state.epoch,ids:new Map()};
  for(const e of state?.timeline??[])if(e.data.event==='incident_injected'&&typeof e.data.id==='string'&&!injectedRef.current.ids.has(e.data.id))injectedRef.current.ids.set(e.data.id,num(e.data.stamp)??0);
  const injected=injectedRef.current.ids,available=incidents.filter(h=>!injected.has(str(h.id)));
  const chosenIncident=available.some(h=>str(h.id)===incident)?incident:str(available[0]?.id,'');
  const incidentLabel=(h:typeof incidents[number])=>{const name=str(h.display_name,`${human(h.kind)} · ${human(h.edge)}`),at=injected.get(str(h.id));
    return at!==undefined?`${name} — in the simulation since ${fmt(at)} s`:h.manual_only===false&&session?.session.manual_incidents!==true?`${name} — also appears on its own`:name;};
  const {scene,error:sceneError}=useScene(session?`${str(session.session.scenario)}:${str(session.session.generation,str(session.session.output,''))}`:'');
  const world=useMemo(()=>liveView(topics,params,decision,oil),[state,telemetry.online,session?.parameters,oil]);
  const title=scenario==='forest'?'Forest wildfire response':oil?'Industrial emergency response':'Emergency supervisor';
  const caseLabel=scenario==='forest'?CASE_LABELS[str(session?.session.case,'')]??str(session?.session.case):human(session?.session.case);

  const stopBox=<div className={`stop-box ${stopped?'latched':''}`}><div><span className="eyebrow">GROUND ROBOT EMERGENCY STOP</span><strong>{!stopConfirmed?'State unknown':stopped?'STOP LATCH ACTIVE':'Latch released · confirmed'}</strong><Stamp topic={latch}/></div><div className="button-row"><button className="danger-button" disabled={sending} onClick={()=>send({command:'emergency_stop',enabled:true})}>■ Emergency stop</button><button disabled={sending||!stopped} onClick={()=>send({command:'emergency_stop',enabled:false})}>Release</button></div>{stopResult.state&&<small>Service: {human(stopResult.state)} · {str(stopResult.message)}</small>}</div>;

  const scenarioSection=<>
    <div className="side-group"><span className="eyebrow">DEMO PRESETS · FILL IN THE FORM BELOW</span><div className="button-row two"><button className={presetActive('E')?'chosen':''} aria-pressed={presetActive('E')} disabled={sessionLocked} onClick={()=>preset('E')} title="Forest case E with policy B4 and limited crew awareness: the crew already knows Trail A is blocked">Crew knows</button><button className={presetActive('F')?'chosen':''} aria-pressed={presetActive('F')} disabled={sessionLocked} onClick={()=>preset('F')} title="Forest case F with policy B4 and limited crew awareness: the crew does not know Trail A is blocked">Crew unaware</button></div><p className="help">Forest · B4 · seed 7 · crew sees 1 m, no map glances, no lapses. Change anything below before loading.</p></div>
    <fieldset className="side-group" disabled={sessionLocked}>
      <label>Scenario<select aria-label="Scenario" value={selection.scenario} onChange={e=>setSelection({...selection,scenario:e.target.value as 'forest'|'oil_facility',case:e.target.value==='forest'?'F':'unaware'})}><option value="forest">Forest wildfire</option><option value="oil_facility">Oil facility</option></select></label>
      <label>Case<select aria-label="Scenario case" value={selection.case} onChange={e=>setSelection({...selection,case:e.target.value})}>{session?.cases[selection.scenario]?.map(c=><option key={c} value={c}>{selection.scenario==='forest'?CASE_LABELS[c]??c:human(c)}</option>)}</select></label>
      <label>Warning policy<select aria-label="Warning policy" value={selection.policy} onChange={e=>setSelection({...selection,policy:e.target.value})}>{(session?.policies??[]).map(p=><option key={p} value={p}>{policyName(p)}</option>)}</select></label>
    </fieldset>
    <details className="side-group"><summary>More settings</summary><fieldset disabled={sessionLocked}>
      <label>Decision engine<select value={selection.engine} onChange={e=>setSelection({...selection,engine:e.target.value as SessionSelection['engine']})}><option value="expected_loss">Expected loss</option><option value="value_iteration">Value iteration</option></select></label>
      <label>Random seed<input aria-label="Random seed" type="number" min="0" max="2147483647" value={selection.seed} onChange={e=>setSelection({...selection,seed:Number(e.target.value)})}/></label>
      <label className="check"><input type="checkbox" checked={selection.manual_incidents} onChange={e=>setSelection({...selection,manual_incidents:e.target.checked})}/> Manual test incidents (no scheduled hazards)</label>
      {selection.scenario==='forest'&&<div className="form-grid three"><label>Sight radius (m)<input aria-label="Crew observation radius" type="number" min="0" max="20" step="0.5" value={selection.human_radius} onChange={e=>setSelection({...selection,human_radius:Number(e.target.value)})}/></label><label>Map glances / s<input aria-label="Passive map glances" type="number" min="0" max="1" step="0.01" value={selection.map_rate} onChange={e=>setSelection({...selection,map_rate:Number(e.target.value)})}/></label><label>Decision lapse<input aria-label="Human decision lapse" type="number" min="0" max="1" step="0.01" value={selection.human_lapse} onChange={e=>setSelection({...selection,human_lapse:Number(e.target.value)})}/></label></div>}
    </fieldset></details>
    <div className={`side-group load-group ${pending?'pending':''}`}>
      <p className="load-state" role="status">{pending?'● Not loaded yet: the simulation is still running the previous configuration.':'✓ This configuration is the one running.'}</p>
      <button className="primary wide" disabled={sessionLocked} onClick={()=>control({command:'switch',...selection})}>Load scenario & policy</button>
      <p className="help">Starts a fresh, paused Gazebo run (about 1–2 minutes). Then press Resume simulation at the top.</p>
    </div>
    <fieldset className="side-group" disabled={!live||paused||sending||!available.length}><span className="eyebrow">DURING THE RUN · TEST INCIDENT</span><select aria-label="Test incident" value={chosenIncident} onChange={e=>setIncident(e.target.value)}>{incidents.length?incidents.map(h=><option key={str(h.id)} value={str(h.id)} disabled={injected.has(str(h.id))}>{incidentLabel(h)}</option>):<option value="">No injectable incident</option>}</select><button className="wide" onClick={()=>send({command:'inject',hazard_id:chosenIncident})}>{incidents.length&&!available.length?'All incidents are in the simulation':'Inject test incident'}</button><p className="help">Spawns the hazard now. Each incident can happen once per run, and several can coexist. Incidents marked “also appears on its own” are scheduled unless “Manual test incidents” is on. Hazards under smoke exist but the sensors cannot see them: check the Gazebo window.</p></fieldset>
  </>;

  const robotSection=<>
    <div className="side-group"><div className="robot-identity"><span className="robot-symbol">{oil?'▣':'✣'}</span><div><h3>{oil?'Ground robot':'UAV lookout'}</h3><span>{oil?'Differential drive · Nav2':'Scripted observation patrol'}</span></div><Stamp topic={robotTopic}/></div>
      <dl className="facts compact"><Fact label="Mission state">{human(oil?nav.state:uav.state,'Awaiting telemetry')}</Fact><Fact label="Position · world (m)">{coordinate?`${fmt(coordinate[0])}, ${fmt(coordinate[1])}, ${fmt(robot.position?arr(robot.position)[2]:null)}`:'Unavailable'} <SourceTag source={oil?'observation':'simulation_truth'}/></Fact><Fact label="Current waypoint">{oil?goal.x!==undefined?`${fmt(goal.x)}, ${fmt(goal.y)}`:'No active goal':uavGoal.length?uavGoal.map(v=>fmt(v)).join(', '):'Unavailable'}</Fact><Fact label="Sensors">{oil?`${topics['/robot/scan']?topics['/robot/scan'].stale?'LiDAR stale':`${fmt(data('/robot/scan').valid_returns,0)} LiDAR returns`:'LiDAR unavailable'} · ${topics['/robot/camera/image']?'RGB':'no RGB'}`:`${topics['/uav/camera/image']?topics['/uav/camera/image'].stale?'RGB stale':'RGB camera':'RGB unavailable'} · logical camera`}</Fact><Fact label="Battery">Not modeled</Fact></dl>
      {!oil&&<p className="help">Scripted Gazebo motion; no validated flight dynamics.</p>}</div>
    {oil&&<div className="side-group"><span className="eyebrow">OPERATOR ROUTE APPROVAL <SourceTag source="control"/></span>
      <fieldset disabled={!live||paused||sending}><label>Destination<select aria-label="Destination" value={destination} onChange={e=>setDestination(e.target.value)}>{Object.entries(options).map(([id,label])=><option key={id} value={id}>{str(label)}</option>)}</select></label><label>Preferred corridor<select aria-label="Preferred corridor" value={edge} onChange={e=>setEdge(e.target.value)}><option value="">Planner recommendation</option>{arr(data('/hri/facility').corridors).map(c=><option key={str(c)} value={str(c)}>{human(c)}</option>)}</select></label><button className="wide" onClick={()=>send({command:'select_destination',destination,...(edge?{via:edge}:{})})}>Request route recommendation</button></fieldset>
      <div className="approval-box"><strong>{robotRoute.navigation_allowed===true?'Current route awaits operator approval':'No sufficiently safe known route'}</strong><p>Revision {str(robotRoute.revision)} · {arr(robotRoute.recommended).map(v=>human(v)).join(' → ')||'Unavailable'}</p><div className="button-row"><button className="primary" data-route-revision={num(robotRoute.revision)??0} disabled={!navAllowed} onClick={()=>send({command:'approve_route',revision:Number(robotRoute.revision)})}>Approve recommended route</button><button className="quiet" disabled={!online||sending} onClick={()=>send({command:'cancel_waypoint'})}>Cancel navigation</button></div></div>
      <p className="help">Selecting a destination does not authorize motion; only an approved revision does.</p></div>}
    {oil&&stopBox}
    <details className="side-group"><summary>{oil?'Manual navigation goal':'UAV observation waypoint'}</summary><fieldset disabled={!live||paused||sending}><div className="coordinate-fields"><label>X (m)<input aria-label="Waypoint X" type="number" value={x} onChange={e=>setX(e.target.value)}/></label><label>Y (m)<input aria-label="Waypoint Y" type="number" value={y} onChange={e=>setY(e.target.value)}/></label>{!oil&&<label>Z (m)<input aria-label="Waypoint Z" type="number" min="6" max="12" value={z} onChange={e=>setZ(e.target.value)}/></label>}</div><button className="wide" disabled={!x.trim()||!y.trim()||![Number(x),Number(y),Number(z)].every(Number.isFinite)} onClick={()=>send(oil?{command:'waypoint',x:Number(x),y:Number(y)}:{command:'uav_waypoint',x:Number(x),y:Number(y),z:Number(z)})}>{oil?'Request coordinate waypoint':'Set UAV waypoint'}</button><p className="help">{oil?'Use a surveyed road node (within 0.75 m).':'World coordinates: x −5…20, y −6…12, z 6…12 m. The UAV flies there, looks for 5 s, then resumes its patrol.'} Hover the map to read coordinates.</p></fieldset></details>
    {oil&&<details className="side-group"><summary>Equipment registry</summary><EquipmentRegistryPanel topic={topics['/hri/equipment_registry']} disabled={sending||session?.busy===true||Boolean(supervisor.error)||!session} onSave={saveEquipmentContents}/></details>}
  </>;

  const experimentSection=<>
    <div className="side-group"><dl className="facts compact"><Fact label="Active scenario">{human(scenario)} · {str(session?.session.case)}</Fact><Fact label="Seed / policy">{str(scenarioStatus.seed,str(session?.session.seed))} / {str(decision.policy,str(session?.session.policy))}</Fact><Fact label="Mission outcome">{oil&&!(num(intent.decision_id)??0)?'No destination assigned':intent.completed===true?'Destination reached':intent.completed===false?'In progress':'Not reported'}</Fact><Fact label="Warnings sent">{num(decision.warnings_sent)??warnings.length}</Fact></dl></div>
    <div className="side-group"><label>Trial duration (simulation seconds)<input type="number" min="10" max="600" value={duration} onChange={e=>setDuration(Number(e.target.value))}/></label>
      <div className="button-row two"><button disabled={!session||sending||session.busy||Boolean(supervisor.error)||session.experiment?.state==='running'} onClick={()=>control({command:'start_experiment',...selection,duration})}>Start experiment</button><button disabled={!session||sending||session.busy||Boolean(supervisor.error)||session.experiment?.state!=='running'} onClick={()=>control({command:'stop_experiment'})}>Stop experiment</button></div>
      <p className="help">Starts a fresh run of the scenario selected under Scenario and records the actual ROS events. The trial pauses at its time limit; Stop keeps the logs.</p>
      <div className="progress" role="progressbar" aria-label="Experiment progress" aria-valuemin={0} aria-valuemax={100} aria-valuenow={100*(num(session?.experiment?.progress)??0)}><i style={{width:percent(session?.experiment?.progress??0)}}/></div><p className="help">{human(session?.experiment?.state,'No active experiment')} · {fmt(session?.experiment?.elapsed)} / {fmt(session?.experiment?.duration)}s sim</p></div>
    <div className="side-group"><label className="check"><input type="checkbox" checked={session?.evaluation_enabled??false} disabled={!session||session.busy||Boolean(supervisor.error)} onChange={e=>control({command:'evaluation',enabled:e.target.checked})}/> Enable evaluation truth counters</label>{session?.evaluation_enabled?<div className="evaluation-box"><SourceTag source="simulation_truth"/>{session.evaluation?.available===true?<><dl className="facts compact"><Fact label="False alarms (provisional)">{str(session.evaluation.false_alarms)}</Fact><Fact label="Missed episodes (provisional)">{str(session.evaluation.missed_warnings)}</Fact></dl><p>{str(session.evaluation.notes)}</p></>:<p>{str(session.evaluation?.reason,'Waiting for evaluation log')}</p>}</div>:<p className="help">False/missed-warning counters need evaluation truth. The normal view never shows private belief labels.</p>}
      <div className="artifact-links">{session?.artifacts.map(a=><a href={a.url} key={a.name} target="_blank" rel="noreferrer">↗ {a.name}</a>)}</div>
      <details><summary>Simulation parameters</summary><p>Configuration, not measured telemetry. Private belief fields are excluded.</p><pre>{JSON.stringify({...params,session:session?.session},null,2)}</pre></details></div>
  </>;

  const logSection=<>
    <div className="side-group"><span className="eyebrow inline">WARNING HISTORY <ConceptChips items={['trust']}/></span><div className="warning-history">{[...warnings].reverse().map(w=><WarningCard key={str(w.warning.id)} record={w} send={send} disabled={!online||sending} compact/>)}{!warnings.length&&<p className="help">No warnings in this run. Repeated alerts are suppressed by the active policy.</p>}</div><p className="help">Latest crew/operator message receipt: {str(data('/hri/acknowledgement').warning_id,'none')} (receipt, not proof of compliance).</p></div>
    <div className="side-group"><span className="eyebrow">LIVE EVENT LOG · SIMULATION TIME</span><div className="event-list">{[...(state?.timeline??[])].reverse().slice(0,40).map((e,i)=><div key={i} className="event-row"><b>{fmt(e.data.stamp)}s</b><span>{human(e.data.event??e.data.state??e.data.action)}</span><small>{human(e.data.source,e.topic)} · {str(e.data.reason??e.data.detail??e.data.command??e.data.explanation,'')}</small></div>)}{!state?.timeline.length&&<p className="help">No events yet.</p>}</div>
      {state?.validation_errors.length?<p className="error-banner">Rejected telemetry: {state.validation_errors.map(e=>`${str(e.topic)}: ${str(e.reason)}`).join('; ')}</p>:null}</div>
  </>;

  const content:Record<Section,ReactNode>={scenario:scenarioSection,layers:<><LayerList layers={layers} onChange={setLayer} oil={oil}/><p className="help side-pad">Drag the map to orbit, right-drag to pan, scroll to zoom. Layers are remembered in this browser.</p></>,robot:robotSection,experiment:experimentSection};
  const banner=restarting?'Restarting Gazebo and ROS nodes. Old-run telemetry is hidden.':supervisor.error||(!online?telemetry.error||'Reconnecting to the ROS gateway…':!live?'No current simulation telemetry. Motion controls are unavailable.':'');

  return <div className={`app-shell ${sideOpen?'':'side-closed'} ${explainOpen?'':'explain-closed'}`}>
    <header className="topbar">
      <a className="brand" href="#"><span className="brand-icon">F</span><div>FIELDWATCH<small>HUMAN–ROBOT OPERATIONS</small></div></a>
      <div className="topbar-title"><strong>{title}</strong><span>{caseLabel} · {policyName(str(decision.policy,str(session?.session.policy)))} · Gazebo / ROS 2</span></div>
      <div className="topbar-actions"><button className={paused?'primary':''} disabled={sessionLocked} onClick={()=>control({command:paused?'resume':'pause'})}>{paused?'▶ Resume simulation':'❚❚ Pause simulation'}</button><button disabled={sessionLocked} onClick={()=>control({command:'reset'})}>Reset run</button></div>
      <div className="topbar-status"><div className="cb-clock"><strong>{fmt(clock)}<span>s</span></strong><small>SIM TIME · SEED {str(scenarioStatus.seed,str(session?.session.seed))}</small></div>
        <div className={`connection ${live?'connected':''}`} role="status"><i/>{restarting?'SWITCHING SCENARIO':paused&&online?'SIMULATION PAUSED':live?'ROS CONNECTED':online?'TELEMETRY STALE':'RECONNECTING'}</div></div>
    </header>
    {(banner||session?.error)&&<div className="shell-banner" role="status">{banner}{session?.error&&<span className="error-text">Session operation failed: {session.error}</span>}</div>}
    <div className="workspace">
      <nav className="rail" role="tablist" aria-label="Sidebar sections">{SECTIONS.map(([id,label,icon])=><button key={id} role="tab" aria-selected={sideOpen&&section===id} className={sideOpen&&section===id?'active':''} onClick={()=>openSection(id)} title={label}>{icon}<span>{label}</span>{id==='experiment'&&session?.experiment?.state==='running'?<em className="rail-count running">●</em>:null}</button>)}</nav>
      <aside className="sidebar" hidden={!sideOpen} aria-label="Configuration">
        <div className="sidebar-head"><h2>{SECTIONS.find(s=>s[0]===section)?.[1]}</h2><button className="icon-button" aria-label="Collapse sidebar" title="Collapse" onClick={()=>setSideOpen(false)}>{Icon.collapse}</button></div>
        <div className="sidebar-body">{SECTIONS.map(([id])=><section key={id} hidden={section!==id} aria-label={id}>{content[id]}</section>)}</div>
        <div className="sidebar-foot command-feedback" role="status">{notice||(paused?'Paused. Press Resume simulation to run.':'Controls are connected to the ROS gateway and session supervisor.')}</div>
      </aside>
      <main className="map-area">
        <EscalationAlert topic={topics['/hri/escalation']} send={send} disabled={!online||sending} compact/>
        <WorldView mode={viewMode} onMode={chooseView} scene={scene} sceneError={sceneError} view={world} oil={oil} clock={clock} layers={layers} gazebo={{session:str(session?.session.generation,str(session?.session.output,'')),ready:Boolean(session)&&!restarting,live,frozen:session?.session.view_paused===true,disabled:sessionLocked,onFreeze:(freeze:boolean)=>control({command:'view',enabled:!freeze}),quietFreeze:(freeze:boolean)=>sendControl({command:'view',enabled:!freeze},true).then(()=>true,()=>false)}}/>
      </main>
      <aside className="explain" hidden={!explainOpen} aria-label={rightSection==='log'?'Warning and event log':'Why the robot acts'}>
        <section hidden={rightSection!=='explain'} className="explain-stack" aria-label="explain">
          <EscalationAlert topic={topics['/hri/escalation']} send={send} disabled={!online||sending}/>
          <Camera key={`${state?.epoch}-${oil}`} topic={topics[oil?'/robot/camera/image':'/uav/camera/image']} uav={!oil} online={online}/>
          <LookoutStatus topics={topics} decision={decision} warnings={warnings} oil={oil}/>
          {active&&<WarningCard record={active} send={send} disabled={!online||sending}/>}
          <CrewBeliefPanel topics={topics} decision={decision} oil={oil}/>
          <DecisionPanel decision={decision} topic={topics['/hri/decision']} oil={oil}/>
        </section>
        <section hidden={rightSection!=='log'} className="explain-stack" aria-label="log">{logSection}</section>
      </aside>
      <nav className="rail right" role="tablist" aria-label="Right panel">{RIGHT_SECTIONS.map(([id,label,icon])=><button key={id} role="tab" aria-selected={explainOpen&&rightSection===id} className={explainOpen&&rightSection===id?'active':''} onClick={()=>openRight(id)} title={label}>{icon}<span>{label}</span>{id==='log'&&warnings.length?<em className="rail-count">{warnings.length}</em>:null}{id==='explain'&&(active||data('/hri/escalation').active===true)&&!(explainOpen&&rightSection==='explain')?<em className="rail-count">!</em>:null}</button>)}</nav>
    </div>
  </div>;
}
