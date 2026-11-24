// Protocol v1 matches hri_dashboard.contracts + dashboard_session.py.
// Runtime validation is mandatory; ROS JSON is not trusted TypeScript.
export type Json = null | boolean | number | string | Json[] | { [key: string]: Json };
export type Dict = { [key: string]: Json };
export type Point = [number, number];
export type Source = 'observation'|'inference'|'simulation_truth'|'control'|'configuration';
export type Topic = {data: Dict; received_at:number; age_seconds:number; stale:boolean; source:Source; sim_stamp:number|null};
export type WarningRecord = {warning:Dict; acknowledged:boolean; dismissed:boolean; operator_stamp?:number};
export type State = {schema_version:1; epoch:string; connected:boolean; updated_at:number; topics:Record<string,Topic>; timeline:{topic:string;received_at:number;data:Dict}[]; warnings:WarningRecord[]; validation_errors:Dict[]};
export type Scenario = 'forest'|'oil_facility';
export type Session = {schema_version:1; session:Dict; busy:boolean; operation:string|null; error:string|null; ros_reachable:boolean; sim_time:number|null; parameters:Dict; cases:Record<Scenario,string[]>; policies:string[]; experiment:Dict|null; evaluation_enabled:boolean; evaluation:Dict|null; artifacts:{name:string;url:string}[]};
export type Control = {command:'pause'|'resume'|'cancel_waypoint'} | {command:'waypoint';x:number;y:number} | {command:'uav_waypoint';x:number;y:number;z:number} | {command:'select_destination';destination:string;via?:string} | {command:'approve_route';revision:number} | {command:'inject';hazard_id:string} | {command:'emergency_stop';enabled:boolean} | {command:'acknowledge_warning';hazard_id:string;warning_id:string} | {command:'dismiss_warning';warning_id:string} | {command:'send_operator'};
export type SessionSelection = {scenario:Scenario;case:string;policy:string;engine:'expected_loss'|'value_iteration';seed:number;manual_incidents:boolean;human_radius:number;map_rate:number;human_lapse:number};
export type SessionControl = ({command:'switch'} & SessionSelection) | ({command:'start_experiment';duration:number}&SessionSelection) | {command:'reset'|'pause'|'resume'|'stop_experiment'} | {command:'evaluation'|'view';enabled:boolean};
export const obj=(v:unknown):Dict=>v!==null&&typeof v==='object'&&!Array.isArray(v)?v as Dict:{};
export const num=(v:unknown):number|null=>typeof v==='number'&&Number.isFinite(v)?v:null;
export const str=(v:unknown,fallback='—'):string=>typeof v==='string'?v:typeof v==='number'?String(v):fallback;
export const arr=(v:unknown):Json[]=>Array.isArray(v)?v:[];
export const point=(v:unknown):Point|null=>Array.isArray(v)&&num(v[0])!==null&&num(v[1])!==null?[v[0],v[1]]:null;
export const human=(v:unknown,fallback='—')=>str(v,fallback).replaceAll('_',' ');
export const fmt=(v:unknown,digits=1)=>num(v)?.toFixed(digits)??'—';
export const percent=(v:unknown)=>num(v)===null?'—':`${Math.round(num(v)!*100)}%`;
const record=(v:unknown)=>v!==null&&typeof v==='object'&&!Array.isArray(v);
function finiteTree(v:unknown,depth=0):boolean {
  if(depth>24)return false;
  if(typeof v==='number')return Number.isFinite(v);
  if(Array.isArray(v))return v.length<=20000&&v.every(x=>finiteTree(x,depth+1));
  if(record(v))return Object.values(v!).every(x=>finiteTree(x,depth+1));
  return v===null||['boolean','string'].includes(typeof v);
}
export function parseState(raw:unknown):State {
  if(!record(raw)||!finiteTree(raw))throw Error('Invalid telemetry envelope');
  const s=raw as State;
  if(s.schema_version!==1||typeof s.epoch!=='string'||typeof s.connected!=='boolean'||num(s.updated_at)===null||!record(s.topics)||!Array.isArray(s.timeline)||!Array.isArray(s.warnings)||!Array.isArray(s.validation_errors))throw Error('Unsupported telemetry contract');
  for(const [name,t] of Object.entries(s.topics)){
    if(!record(t)||!record(t.data)||num(t.received_at)===null||num(t.age_seconds)===null||t.age_seconds<0||typeof t.stale!=='boolean'||!['observation','inference','simulation_truth','control','configuration'].includes(t.source)||!(t.sim_stamp===null||num(t.sim_stamp)!==null))throw Error(`Invalid topic: ${name}`);
    if(name==='/hri/belief'){
      for(const b of [t.data,...Object.values(obj(t.data.beliefs))]){
        const p=obj(b).posterior;if(p===undefined)continue;
        const values=Object.values(obj(p));
        if(values.length!==4||!['open_safe','blocked','open_hazardous','unknown_stale'].every(k=>k in obj(p))||values.some(v=>num(v)===null||num(v)!<0||num(v)!>1)||Math.abs(values.reduce<number>((a,v)=>a+Number(v),0)-1)>1e-5)throw Error('Invalid belief posterior');
      }
    }
    if(name==='/hri/equipment_registry'){
      const entries=t.data.equipment;
      if(t.data.schema_version!==1||!Array.isArray(entries)||entries.length===0||entries.length>128)throw Error('Invalid equipment registry');
      const ids=new Set<string>(),models=new Set<string>();
      for(const raw of entries){
        if(!record(raw))throw Error('Invalid equipment entry');
        const value=obj(raw),contents=obj(value.contents),position=value.position_m;
        if(typeof value.equipment_id!=='string'||!value.equipment_id||ids.has(value.equipment_id)
          ||typeof value.gazebo_model!=='string'||!value.gazebo_model||models.has(value.gazebo_model)
          ||!Array.isArray(position)||position.length!==3||position.some((coordinate:unknown)=>num(coordinate)===null)
          ||!record(value.contents)||['label','material_class','source'].some(k=>typeof contents[k]!=='string'))throw Error('Invalid equipment entry');
        ids.add(value.equipment_id);models.add(value.gazebo_model);
      }
    }
  }
  if(s.timeline.some(e=>!record(e)||typeof e.topic!=='string'||!record(e.data)||num(e.received_at)===null))throw Error('Invalid event timeline');
  if(s.warnings.some(w=>!record(w)||!record(w.warning)||typeof w.acknowledged!=='boolean'||typeof w.dismissed!=='boolean'))throw Error('Invalid warnings');
  return s;
}
export function parseSession(raw:unknown):Session {
  if(!record(raw)||!finiteTree(raw))throw Error('Invalid session response');
  const s=raw as Session;
  if(s.schema_version!==1||!record(s.session)||!record(s.parameters)||typeof s.busy!=='boolean'||!record(s.cases)||!Array.isArray(s.artifacts)||typeof s.evaluation_enabled!=='boolean')throw Error('Unsupported session contract');
  if(!Array.isArray(s.cases.forest)||!Array.isArray(s.cases.oil_facility)||s.artifacts.some(a=>typeof a.name!=='string'||typeof a.url!=='string'||!a.url.startsWith('/api/session/artifacts/results/')))throw Error('Invalid session catalog');
  return s;
}

// Static Gazebo world geometry for the web 3D view (scripts/scene_export.py).
export type Vec3=[number,number,number];
export type ScenePrimitive={shape:'box';size:Vec3}|{shape:'cylinder';radius:number;length:number}|{shape:'sphere';radius:number}|{shape:'ellipsoid';radii:Vec3}|{shape:'mesh';vertices:number[];indices:number[]};
export type SceneItem=ScenePrimitive&{name:string;position:Vec3;quaternion:[number,number,number,number];color:[number,number,number,number]};
export type SceneSensor={horizontal_fov:number;far:number;pitch:number};
export type SceneSpawn={position:Vec3;yaw:number};
export type Scene={schema_version:1;scenario:string;world:string|null;background:number[]|null;scenery:SceneItem[];entities:Record<string,SceneItem[]>;sensors:Record<string,SceneSensor>;spawns?:Record<string,SceneSpawn>;hazards:Record<string,SceneItem[]>;source:string};
const numbers=(v:unknown,n:number)=>Array.isArray(v)&&v.length===n&&v.every(x=>num(x)!==null);
const positive=(v:unknown)=>num(v)!==null&&num(v)!>0;
function validItem(raw:unknown):boolean{
  if(!record(raw))return false;
  const i=raw as Dict;
  if(typeof i.name!=='string'||!numbers(i.position,3)||!numbers(i.quaternion,4)||!numbers(i.color,4))return false;
  switch(i.shape){
    case 'box':return numbers(i.size,3)&&arr(i.size).every(positive);
    case 'cylinder':return positive(i.radius)&&positive(i.length);
    case 'sphere':return positive(i.radius);
    case 'ellipsoid':return numbers(i.radii,3)&&arr(i.radii).every(positive);
    case 'mesh':{const v=arr(i.vertices),idx=arr(i.indices);return v.length%3===0&&v.length<=300000&&idx.length%3===0&&idx.length<=600000&&v.every(x=>num(x)!==null)&&idx.every(x=>Number.isInteger(x)&&Number(x)>=0&&Number(x)<v.length/3);}
    default:return false;
  }
}
export function parseScene(raw:unknown):Scene{
  if(!record(raw))throw Error('Invalid scene response');
  const s=raw as Scene;
  if(s.schema_version!==1||typeof s.scenario!=='string'||!Array.isArray(s.scenery)||s.scenery.length>5000||!record(s.entities)||!record(s.hazards)||!record(s.sensors))throw Error('Unsupported scene contract');
  const groups=[s.scenery,...Object.values(s.entities),...Object.values(s.hazards)];
  if(groups.some(g=>!Array.isArray(g)||!g.every(validItem)))throw Error('Invalid scene geometry');
  if(Object.values(s.sensors).some(v=>!record(v)||!positive(v.horizontal_fov)||!positive(v.far)||num(v.pitch)===null))throw Error('Invalid scene sensor');
  if(s.background!==null&&!numbers(s.background,4))throw Error('Invalid scene background');
  if(s.spawns!==undefined&&(!record(s.spawns)||Object.values(s.spawns).some(v=>!record(v)||!numbers(v.position,3)||num(v.yaw)===null)))throw Error('Invalid scene spawn');
  return s;
}
