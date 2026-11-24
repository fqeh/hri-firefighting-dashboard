// World view: a GIS-style layered 3D scene drawn in the browser, with Gazebo's
// own (software-rendered) window kept one click away.
import {useEffect,useRef,useState,type ReactNode,type SyntheticEvent} from 'react';
import {arr,obj,num,str,percent,point,type Dict,type Topic,type Point,type Scene,type Source,type Vec3} from './contracts';
import {SourceTag} from './components';
import {beliefGap,GAZEBO_FULL,GAZEBO_EMBED} from './insight';
import {WorldRenderer,PALETTE,type BeliefLook,type CameraStatus,type LayerId,type LayerState,type LiveView} from './world3d';

type LayerInfo={id:LayerId;label:(who:string,eye:string)=>string;source?:Source;color:string;detail:string;forestOnly?:boolean};
export const LAYERS:LayerInfo[]=[
  {id:'world',label:()=>'Gazebo world',source:'simulation_truth',color:'#7f9a5a',detail:'Terrain, trails, buildings and moving agents from the Gazebo world file and live poses.'},
  {id:'trees',label:()=>'Trees',source:'simulation_truth',color:'#2f6b3a',detail:'Forest canopy and undergrowth from the Gazebo world. Hide it to see the trails and the crew underneath.',forestOnly:true},
  {id:'seen',label:(_,eye)=>`What the ${eye} sees`,source:'observation',color:PALETTE.sensor,detail:'Sensor coverage, observed hazards and route segments marked clear, blocked or unknown.'},
  {id:'belief',label:who=>`What the ${who} believes`,source:'inference',color:PALETTE.belief,detail:'Bayesian ToM estimate drawn as a floating map above the trails; dashed: the route they are taking.'},
  {id:'route',label:()=>'Recommended route',source:'inference',color:PALETTE.route,detail:'Risk-weighted route from the planner.'},
  {id:'truth',label:()=>'Fire model (truth)',source:'simulation_truth',color:PALETTE.fire,detail:'Ground-truth fire envelope from the simulator. Evaluation context only; never an input to inference.',forestOnly:true},
  {id:'labels',label:()=>'Labels',color:'#334155',detail:'Names of agents, hazards and trail nodes.'},
];
export const DEFAULT_LAYERS:LayerState={world:{on:true,opacity:1},trees:{on:true,opacity:1},seen:{on:true,opacity:1},belief:{on:true,opacity:1},route:{on:true,opacity:1},truth:{on:false,opacity:1},labels:{on:true,opacity:1}};
const STORE='fieldwatch-layers-v1';
function loadLayers():LayerState{
  try{const saved=JSON.parse(localStorage.getItem(STORE)??'null');
    if(saved&&typeof saved==='object')return Object.fromEntries(Object.entries(DEFAULT_LAYERS).map(([k,v])=>{const s=saved[k];return [k,s&&typeof s.on==='boolean'&&num(s.opacity)!==null?{on:s.on,opacity:Math.min(1,Math.max(0,s.opacity))}:v];})) as LayerState;
  }catch{/* private mode or corrupt value: use defaults */}
  return DEFAULT_LAYERS;
}

const HYPOTHESIS:Record<string,BeliefLook>={open_safe:'open',blocked:'blocked',open_hazardous:'hazard',unknown_stale:'unknown'};
export function beliefLook(posterior:Dict):{look:BeliefLook;p:number}{
  const best=Object.entries(posterior).filter(([k,v])=>k in HYPOTHESIS&&num(v)!==null).sort((a,b)=>Number(b[1])-Number(a[1]))[0];
  return best?{look:HYPOTHESIS[best[0]],p:Number(best[1])}:{look:'none',p:0};
}

const vec3=(v:unknown):Vec3|null=>{const p=point(v);return p?[p[0],p[1],num(arr(v)[2])??0]:null;};
// Pure projection of telemetry into what the 3D view draws (unit-tested).
export function liveView(topics:Record<string,Topic>,parameters:Dict,decision:Dict,oil:boolean):LiveView{
  const data=(n:string)=>obj(topics[n]?.data);
  const route=data('/hri/rescue_route'),robotRoute=data('/hri/robot_route');
  const nodes=Object.fromEntries(Object.entries(obj(route.nodes??parameters.nodes)).flatMap(([k,v])=>{const p=point(v);return p?[[k,p]]:[];})) as Record<string,Point>;
  const edges=arr(parameters.edges).map(obj).flatMap(e=>{const a=nodes[str(e.source)],b=nodes[str(e.target)];return a&&b?[{id:str(e.id),a,b}]:[];});
  const who=oil?'operator':'crew',origin=point(parameters.robot_start)??[0,0];
  const entities=([['human','Crew'],['uav','UAV'],['robot','Ground robot']] as const).flatMap(([key,name])=>{
    const topic=topics[`/${key}/odom`],p=vec3(topic?.data.position);if(!topic||!p)return [];
    const world=key==='robot'?point(robotRoute.crew_position)??[p[0]+origin[0],p[1]+origin[1]]:[p[0],p[1]];
    return [{key,label:name,position:[world[0],world[1],p[2]] as Vec3,yaw:num(topic.data.yaw)??0,stale:topic.stale}];
  });
  const hazards=arr(data('/hri/observations').hazards).map(obj).flatMap(h=>{const p=vec3(h.position);
    return p?[{id:str(h.id),kind:str(h.kind,'unknown'),edge:str(h.edge,''),position:p,radius:num(h.radius)??1,stale:h.observation_stale===true}]:[];});
  const beliefs=obj(data('/hri/belief').beliefs);
  const belief=Object.fromEntries(Object.entries(beliefs).map(([edge,b])=>[edge,beliefLook(obj(obj(b).posterior))]));
  const g=beliefGap(topics,decision);
  const gap=!g.hazardSeen?null:{edge:g.focus,kind:g.gap?'gap' as const:'agree' as const,
    text:g.gap?`≠ ${who} thinks it is open · ${percent(g.p)}`:g.approaching?`heading there · ${percent(g.p)} below the bar`:g.offTask?'= not on their task':`= ${who}’s route avoids it`};
  const fireZones=arr(data('/forest/fire_state').zones).map(z=>arr(obj(z).polygon).flatMap(p=>point(p)?[point(p)!]:[]));
  const smoke=arr(data('/hri/observations').occluders).map(obj).flatMap(o=>{const p=vec3(o.position);return p?[{id:str(o.id),edge:str(o.edge,''),position:p,radius:num(o.radius)??3.5}]:[];});
  return {nodes,edges,entities,hazards,edgeStates:Object.fromEntries(Object.entries(obj(route.edge_states)).map(([k,v])=>[k,str(v,'unknown')])),
    belief,intent:arr(data('/hri/human_intent').path).map(v=>str(v)),recommended:arr(route.recommended).map(v=>str(v)),gap,fireZones,who,smoke};
}

// Simulation speed relative to wall clock, from successive /clock readings.
export function useRealTimeFactor(clock:number|null){
  const samples=useRef<[number,number][]>([]),[rtf,setRtf]=useState<number|null>(null);
  useEffect(()=>{
    if(clock===null)return;
    const now=performance.now()/1000,s=samples.current;
    if(s.length&&clock<s[s.length-1][1])s.length=0;  // new run
    s.push([now,clock]);while(s.length>2&&now-s[0][0]>8)s.shift();
    const [t0,c0]=s[0];setRtf(now-t0>1.5?(clock-c0)/(now-t0):null);
  },[clock]);
  return rtf;
}

export const Icon={
  sliders:<svg viewBox="0 0 24 24"><path d="M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0"/><circle cx="16" cy="6" r="2"/><circle cx="10" cy="12" r="2"/><circle cx="18" cy="18" r="2"/></svg>,
  robot:<svg viewBox="0 0 24 24"><rect x="5" y="8" width="14" height="10" rx="2"/><path d="M12 4v4M9 13h.01M15 13h.01M3 12v3M21 12v3"/></svg>,
  flask:<svg viewBox="0 0 24 24"><path d="M9 3h6M10 3v6L4.5 18.5A1.7 1.7 0 0 0 6 21h12a1.7 1.7 0 0 0 1.5-2.5L14 9V3"/><path d="M7 15h10"/></svg>,
  list:<svg viewBox="0 0 24 24"><path d="M9 6h11M9 12h11M9 18h11M4 6h.01M4 12h.01M4 18h.01"/></svg>,
  explain:<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="9"/><path d="M12 11v6M12 7.5h.01"/></svg>,
  collapse:<svg viewBox="0 0 24 24"><path d="m15 6-6 6 6 6"/></svg>,
  expand:<svg viewBox="0 0 24 24"><path d="m9 6 6 6-6 6"/></svg>,
  eye:<svg viewBox="0 0 24 24"><path d="M2 12s3.6-7 10-7 10 7 10 7-3.6 7-10 7S2 12 2 12Z"/><circle cx="12" cy="12" r="3"/></svg>,
  eyeOff:<svg viewBox="0 0 24 24"><path d="M3 3l18 18M10.6 5.1A10.8 10.8 0 0 1 12 5c6.4 0 10 7 10 7a17 17 0 0 1-3.2 4.1M6.6 6.6C3.7 8.5 2 12 2 12s3.6 7 10 7a9.7 9.7 0 0 0 5.4-1.6"/></svg>,
  home:<svg viewBox="0 0 24 24"><path d="M3 11.5 12 4l9 7.5M5.5 9.5V20h13V9.5"/></svg>,
  top:<svg viewBox="0 0 24 24"><rect x="4" y="4" width="16" height="16" rx="2"/><path d="M4 12h16M12 4v16"/></svg>,
  follow:<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/><circle cx="12" cy="12" r="8"/></svg>,
  full:<svg viewBox="0 0 24 24"><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></svg>,
  layers:<svg viewBox="0 0 24 24"><path d="m12 3 9 5-9 5-9-5 9-5Z"/><path d="m3 13 9 5 9-5"/></svg>,
};

// Layer visibility/opacity, shared by the sidebar list and the renderer; remembered per browser.
export function useLayers():[LayerState,(id:LayerId,patch:Partial<LayerState[LayerId]>)=>void]{
  const [layers,setLayers]=useState<LayerState>(loadLayers);
  useEffect(()=>{try{localStorage.setItem(STORE,JSON.stringify(layers));}catch{/* storage unavailable */}},[layers]);
  return [layers,(id,patch)=>setLayers(l=>({...l,[id]:{...l[id],...patch}}))];
}

export function LayerList({layers,onChange,oil}:{layers:LayerState;onChange:(id:LayerId,patch:Partial<LayerState[LayerId]>)=>void;oil:boolean}){
  const who=oil?'operator':'crew',eye=oil?'robot':'UAV';
  return <div className="layer-list" aria-label="Map layers">
    {LAYERS.filter(l=>!(l.forestOnly&&oil)).map(l=>{const s=layers[l.id],name=l.label(who,eye);return <div key={l.id} className={`layer-row ${s.on?'':'off'}`} data-layer={l.id}>
      <div className="layer-line">
        <button className="layer-eye" aria-pressed={s.on} aria-label={`${s.on?'Hide':'Show'} ${name}`} onClick={()=>onChange(l.id,{on:!s.on})}>{s.on?Icon.eye:Icon.eyeOff}</button>
        <i className="layer-swatch" style={{background:l.color}}/>
        <span className="layer-title">{name}</span>{l.source&&<SourceTag source={l.source}/>}
      </div>
      <label className="layer-opacity"><span>Opacity</span><input type="range" min="0" max="1" step="0.05" value={s.opacity} disabled={!s.on} aria-label={`${name} opacity`} onChange={e=>onChange(l.id,{opacity:Number(e.target.value)})}/><output>{s.opacity.toFixed(2)}</output></label>
      <p className="layer-detail">{l.detail}</p>
    </div>;})}
  </div>;
}

export function WorldView3D({scene,sceneError,view,oil,layers,status}:{scene:Scene|null;sceneError:string;view:LiveView;oil:boolean;layers:LayerState;status:ReactNode}){
  const frame=useRef<HTMLDivElement>(null),host=useRef<HTMLDivElement>(null),engine=useRef<WorldRenderer|null>(null);
  const [failure,setFailure]=useState(''),[follow,setFollow]=useState(false),[camera,setCamera]=useState<CameraStatus|null>(null);
  const who=oil?'operator':'crew';
  useEffect(()=>{
    if(!host.current)return;
    try{const r=new WorldRenderer(host.current,layers,setCamera);engine.current=r;setFailure('');
      // ?capture exposes camera placement to scripts that render presentation figures.
      const capture=new URLSearchParams(window.location.search).has('capture');
      if(capture)Object.assign(window,{fieldwatchCapture:{focus:r.focusOn.bind(r),entity:r.entityPosition.bind(r),layer:r.setLayerVisible.bind(r)}});
      return()=>{r.dispose();engine.current=null;if(capture)Reflect.deleteProperty(window,'fieldwatchCapture');};}
    catch(e){setFailure(e instanceof Error?e.message:String(e));}
  },[]);
  useEffect(()=>{if(scene)engine.current?.setScene(scene);},[scene,failure]);
  useEffect(()=>{engine.current?.update(view);},[view,scene,failure]);
  useEffect(()=>{engine.current?.setLayers(layers);},[layers]);
  useEffect(()=>{engine.current?.setFollow(follow?(oil?'robot':'human'):null);},[follow,oil]);
  const fullscreen=()=>{const el=frame.current;if(!el)return;if(document.fullscreenElement)void document.exitFullscreen();else void el.requestFullscreen?.();};
  return <div className="world3d" ref={frame}>
    <div className="world3d-stage" ref={host}>
      {failure&&<div className="world3d-message"><strong>The web 3D view needs WebGL</strong><span>{failure}. Switch to “Gazebo window”.</span></div>}
      {!failure&&!scene&&<div className="world3d-message"><strong>Loading the Gazebo world…</strong><span>{sceneError||'Reading the world geometry from the session supervisor.'}</span></div>}
      <div className="view-controls" role="toolbar" aria-label="Camera">
        <button title="Reset view" aria-label="Reset view" onClick={()=>engine.current?.view('home')}>{Icon.home}</button>
        <button title="Top-down view" aria-label="Top-down view" onClick={()=>engine.current?.view('top')}>{Icon.top}</button>
        <button title={`Follow the ${oil?'robot':who}`} aria-label={`Follow the ${oil?'robot':who}`} aria-pressed={follow} className={follow?'active':''} onClick={()=>setFollow(!follow)}>{Icon.follow}</button>
        <button title="Full screen" aria-label="Full screen" onClick={fullscreen}>{Icon.full}</button>
      </div>
    </div>
    <div className="world3d-status">
      <span>Cursor: {camera?.cursor?`x ${camera.cursor[0].toFixed(1)} m, y ${camera.cursor[1].toFixed(1)} m`:'—'}</span>
      <span>Pitch {camera?camera.pitch.toFixed(0):'—'}°</span><span>Bearing {camera?camera.bearing.toFixed(0):'—'}°</span><span>Distance {camera?camera.distance.toFixed(0):'—'} m</span>
      <span className="world3d-status-right">{status}</span>
    </div>
  </div>;
}

// onFreeze: the explicit Pause/Resume button (with command feedback); quietFreeze: implicit
// toggles when switching renderer, which must not surface errors the user never asked for.
type GazeboProps={session:string;ready:boolean;live:boolean;frozen:boolean;onFreeze:(freeze:boolean)=>void;quietFreeze:(freeze:boolean)=>Promise<boolean>;disabled:boolean};
export type ViewMode='web'|'gazebo';

// The map area: browser-rendered 3D scene, or Gazebo's own window over noVNC.
export function WorldView({mode,onMode,scene,sceneError,view,oil,clock,layers,gazebo}:{mode:ViewMode;onMode:(m:ViewMode)=>void;scene:Scene|null;sceneError:string;view:LiveView;oil:boolean;clock:number|null;layers:LayerState;gazebo:GazeboProps}){
  const [attempt,setAttempt]=useState(0),rtf=useRealTimeFactor(clock);
  const {session,ready,live,frozen,onFreeze,quietFreeze,disabled}=gazebo;
  // Gazebo's own window is rendered on the server CPU; while the web view is
  // shown it is paused once per run, so the simulation gets that CPU back.
  const autoPaused=useRef(''),[retry,setRetry]=useState(0);
  const pause=()=>{autoPaused.current=session;void quietFreeze(true).then(ok=>{if(!ok){autoPaused.current='';setRetry(r=>r+1);}});};
  useEffect(()=>{
    if(mode!=='web'||!live||!ready||frozen||disabled||autoPaused.current===session||retry>5)return;
    const timer=setTimeout(pause,3000);
    return()=>clearTimeout(timer);
  },[mode,live,ready,frozen,disabled,session,retry]);
  const choose=(next:ViewMode)=>{
    if(next===mode)return;onMode(next);
    if(next==='gazebo'&&frozen&&!disabled)void quietFreeze(false);
    if(next==='web'&&!frozen&&!disabled&&live)pause();
  };
  const fit=(e:SyntheticEvent<HTMLIFrameElement>)=>{
    try{const doc=e.currentTarget.contentDocument;const bar=doc?.getElementById('top_bar');if(bar)bar.style.display='none';
      if(doc?.body)doc.body.style.background='#142c30';}catch{/* cross-origin when not proxied: leave noVNC's own bar */}
  };
  const status=<>{clock!==null&&<>Sim {clock.toFixed(1)} s</>}{rtf!==null&&<> · {rtf.toFixed(2)}× real time</>}{frozen&&<> · Gazebo window paused to save CPU</>}</>;
  return <div className="world-view" id="gazebo">
    <div className="world-mode">
      <div className="segmented" role="tablist" aria-label="World view renderer">
        <button role="tab" aria-selected={mode==='web'} className={mode==='web'?'active':''} onClick={()=>choose('web')}>3D web view</button>
        <button role="tab" aria-selected={mode==='gazebo'} className={mode==='gazebo'?'active':''} onClick={()=>choose('gazebo')}>Gazebo window</button>
      </div>
      {mode==='gazebo'&&<div className="world-mode-tools">
        <button className={`small ${frozen?'primary':''}`} disabled={disabled||!ready} title="The Gazebo window is rendered on the server CPU; pausing it frees cores for the simulation. Physics and sensors keep running." onClick={()=>onFreeze(!frozen)}>{frozen?'▶ Resume':'❚❚ Pause'}</button>
        <button className="small" onClick={()=>setAttempt(attempt+1)}>Reconnect</button>
        <a className="small-link" href={GAZEBO_FULL} target="_blank" rel="noreferrer">Full screen ↗</a></div>}
    </div>
    {mode==='web'?<WorldView3D scene={scene&&scene.scenario===(oil?'oil_facility':'forest')?scene:null} sceneError={sceneError} view={view} oil={oil} layers={layers} status={status}/>
      :<div className="world3d"><div className={`gazebo-frame ${frozen?'frozen':''}`}>{frozen&&<div className="gazebo-frozen">Gazebo window paused to save CPU · the simulation keeps running</div>}{ready?<iframe key={`${session}-${attempt}`} src={GAZEBO_EMBED} title="Gazebo 3D world (native GUI over noVNC)" onLoad={fit}/>
        :<div className="empty"><strong>Waiting for the simulator</strong><span>The 3D view connects when the Gazebo container is running.</span></div>}</div>
        <div className="world3d-status"><span>Gazebo’s own GUI, live over noVNC. Drag to orbit, scroll to zoom.</span><span className="world3d-status-right">{status}</span></div></div>}
  </div>;
}
