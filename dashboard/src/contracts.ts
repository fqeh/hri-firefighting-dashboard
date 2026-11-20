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
export type Control = {command:'pause'|'resume'|'cancel_waypoint'} | {command:'waypoint';x:number;y:number} | {command:'uav_waypoint';x:number;y:number;z:number} | {command:'select_destination';destination:string;via?:string} | {command:'approve_route';revision:number} | {command:'inject';hazard_id:string} | {command:'emergency_stop';enabled:boolean} | {command:'acknowledge_warning';hazard_id:string;warning_id:string} | {command:'dismiss_warning';warning_id:string};
export type SessionSelection = {scenario:Scenario;case:string;policy:string;engine:'expected_loss'|'value_iteration';seed:number;manual_incidents:boolean;human_radius:number;map_rate:number;human_lapse:number};
export type SessionControl = ({command:'switch'} & SessionSelection) | ({command:'start_experiment';duration:number}&SessionSelection) | {command:'reset'|'pause'|'resume'|'stop_experiment'} | {command:'evaluation';enabled:boolean};
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
