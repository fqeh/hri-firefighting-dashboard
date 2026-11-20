import {useEffect,useState,type ReactNode} from 'react';
import {arr,obj,num,str,fmt,human,percent,type Dict,type Topic,type WarningRecord,type Control} from './contracts';
import type {EquipmentContentsUpdate} from './transport';

export function SourceTag({source}:{source:string}){return <span className={`source-tag ${source}`} title="Data origin">{source==='simulation_truth'?'SIM TRUTH':source==='configuration'?'MAP PRIOR':source.toUpperCase()}</span>;}
export function Stamp({topic}:{topic?:Topic}){return <span className={`stamp ${topic?.stale?'stale':''}`}>{topic?`${fmt(topic.sim_stamp)}s sim${topic.stale?' · STALE':''}`:'Awaiting update'}</span>;}
export function Panel({title,tag,children,className='',id}:{title:string;tag?:ReactNode;children:ReactNode;className?:string;id?:string}){return <section className={`panel ${className}`} id={id}><div className="panel-title"><h2>{title}</h2>{tag}</div>{children}</section>;}
export function Fact({label,children}:{label:string;children:ReactNode}){return <div className="fact"><dt>{label}</dt><dd>{children}</dd></div>;}

export function Camera({topic,uav,online}:{topic?:Topic;uav:boolean;online:boolean}){
  const [failed,setFailed]=useState(false),[src,setSrc]=useState('');
  const stamp=topic?.sim_stamp;
  useEffect(()=>{if(online&&topic&&!topic.stale&&!topic.data.error){setFailed(false);setSrc(`${uav?'/api/uav-camera.png':'/api/camera.png'}?stamp=${stamp}`);}},[stamp,uav,online]);
  return <Panel title={uav?'UAV observation camera':'Ground robot camera'} tag={<SourceTag source="observation"/>}>
    <div className="camera-frame">{src&&!failed?<img src={src} alt={uav?'Live Gazebo UAV RGB camera':'Live Gazebo ground robot RGB camera'} onError={()=>setFailed(true)}/>:<div className="empty"><strong>{failed?'Camera request failed':'Waiting for camera frames'}</strong><span>{str(topic?.data.error,'No current ROS image available. Reconnection is automatic.')}</span></div>}
      <div className="camera-caption"><span>RGB · {fmt(topic?.data.width,0)} × {fmt(topic?.data.height,0)}</span><Stamp topic={topic}/></div>
      {(!online||topic?.stale)&&src&&<div className="camera-stale">LAST FRAME · {online?'PAUSED OR STALE':'DISCONNECTED'}</div>}
    </div><p className="panel-note">Actual simulated RGB sensor. No thermal sensor or inferred hotspot overlay.</p>
  </Panel>;
}

export function BeliefPanel({topic,evidence,decision}:{topic?:Topic;evidence?:Topic;decision:Dict}){
  const [selected,setSelected]=useState(''),message=obj(topic?.data),beliefs=obj(message.beliefs);
  const belief=selected&&beliefs[selected]?obj(beliefs[selected]):message,posterior=obj(belief.posterior);
  const dominant=Object.entries(posterior).sort((a,b)=>Number(b[1])-Number(a[1]))[0];
  const action=obj(message.last_operator_evidence??message.last_movement_evidence);
  return <Panel title="Human belief estimate" tag={<SourceTag source="inference"/>} id="belief">
    <div className="panel-body">
      <div className="inline spread"><label>Route <select aria-label="Belief route" value={selected} onChange={e=>setSelected(e.target.value)}><option value="">Current focus · {human(message.edge)}</option>{Object.keys(beliefs).map(e=><option key={e} value={e}>{human(e)}</option>)}</select></label><Stamp topic={topic}/></div>
      <div className="belief-head"><div><span className="eyebrow">DANGEROUS MISMATCH</span><strong data-testid="mismatch">{percent(belief.p_mismatch)}</strong></div><p>{dominant?<>Most likely: <b>{human(dominant[0])}</b><br/>{percent(dominant[1])} posterior probability</>:'Waiting for observed actions and information opportunities.'}</p></div>
      {Object.entries(posterior).map(([name,value])=><div className="posterior-row" key={name}><span>{human(name)}</span><strong>{percent(value)}</strong><div role="meter" aria-label={human(name)} aria-valuemin={0} aria-valuemax={1} aria-valuenow={num(value)??0}><i style={{width:percent(value)}}/></div></div>)}
      <dl className="facts two"><Fact label="Normalized uncertainty">{num(belief.uncertainty)===null?'—':percent(Number(belief.uncertainty)/Math.log(4))}</Fact><Fact label="Last relevant evidence">{fmt(belief.last_relevant_observation)}s sim</Fact></dl>
      <details open><summary>Evidence behind this estimate</summary><p>{Object.keys(action).length?`${human(action.origin)} → ${human(action.target)} at ${fmt(action.stamp)}s; ${str(action.evidence,message.subject==='operator'?'explicit operator waypoint selection':'movement observed in Gazebo')}`:'No informative action has been confirmed.'}</p><p>{human(evidence?.data.event,'Awaiting new information')} {evidence?.sim_stamp!==null&&evidence?.sim_stamp!==undefined?`· ${fmt(evidence.sim_stamp)}s sim`:''}</p><p>{str(decision.explanation,'No warning decision has arrived.')}</p></details>
      <p className="panel-note">An estimate of the {message.subject==='operator'?'operator’s':'crew’s'} understanding, not their hidden true belief. Unknown and stale hypotheses remain explicit.</p>
    </div>
  </Panel>;
}

export function EquipmentRegistryPanel({topic,disabled,onSave}:{topic?:Topic;disabled:boolean;onSave:(update:EquipmentContentsUpdate)=>Promise<void>}){
  const registry=obj(topic?.data),entries=arr(registry.equipment).map(obj);
  const [selectedId,setSelectedId]=useState(''),[label,setLabel]=useState(''),[materialClass,setMaterialClass]=useState(''),[source,setSource]=useState('');
  const [saving,setSaving]=useState(false),[error,setError]=useState(''),[saved,setSaved]=useState('');
  const selected=entries.find(entry=>entry.equipment_id===selectedId)??entries[0],contents=obj(selected?.contents);
  const id=str(selected?.equipment_id,'');
  useEffect(()=>{
    if(id){setSelectedId(id);setLabel(str(contents.label,''));setMaterialClass(str(contents.material_class,''));setSource(str(contents.source,''));}
  },[id,contents.label,contents.material_class,contents.source]);
  async function save(){
    if(!id)return;
    setSaving(true);setError('');setSaved('');
    try{await onSave({equipment_id:id,contents:{label,material_class:materialClass,source}});setSaved('Saved. Configuration telemetry will refresh shortly.');}
    catch(reason){setError(String(reason));}
    finally{setSaving(false);}
  }
  return <Panel title="Industrial equipment registry" tag={<SourceTag source="configuration"/>} id="equipment-registry">
    <div className="panel-body">
      {!topic?<p className="empty"><strong>Registry unavailable</strong><span>Waiting for the industrial simulation configuration.</span></p>:<>
        <div className="inline spread"><label>Equipment<select aria-label="Equipment" value={id} onChange={event=>setSelectedId(event.target.value)}>{entries.map(entry=><option key={str(entry.equipment_id)} value={str(entry.equipment_id)}>{str(entry.equipment_id)} · {human(entry.category)}</option>)}</select></label><Stamp topic={topic}/></div>
        {selected?<><dl className="facts two">
          <Fact label="Gazebo model">{str(selected.gazebo_model)}</Fact><Fact label="Geometry reference">{str(selected.geometry_reference)}</Fact>
          <Fact label="Position · world (m)">{arr(selected.position_m).map(value=>fmt(value)).join(', ')}</Fact>
          <Fact label="Inventory">{human(obj(selected.inventory).status,'Unknown')}</Fact><Fact label="Temperature">{human(obj(selected.temperature).status,'Unknown')}</Fact>
          <Fact label="Pressure">{human(obj(selected.pressure).status,'Unknown')}</Fact><Fact label="Integrity">{human(obj(selected.integrity).status,'Unknown')}</Fact>
          <Fact label="Leak / safety systems">{human(obj(selected.leak).status,'Unknown')} / {human(obj(selected.safety_systems).status,'Unknown')}</Fact>
        </dl><p className="panel-note">{str(registry.source_statement)} Registry metadata is configuration, not measured telemetry. Unsupported values remain explicitly not modeled.</p>
        <details><summary>Edit contents metadata</summary><div className="form-grid equipment-editor">
          <label>Contents label<input aria-label="Contents label" maxLength={120} value={label} onChange={event=>setLabel(event.target.value)}/></label>
          <label>Material class<input aria-label="Material class" maxLength={80} value={materialClass} onChange={event=>setMaterialClass(event.target.value)}/></label>
          <label>Metadata source<input aria-label="Metadata source" maxLength={200} value={source} onChange={event=>setSource(event.target.value)}/></label>
        </div><button disabled={disabled||saving||topic.stale||!label.trim()||!materialClass.trim()||!source.trim()} onClick={()=>void save()}>{saving?'Saving…':'Save metadata'}</button>
        {error&&<p role="alert">{error}</p>}{saved&&<p role="status">{saved}</p>}</details></>:<p>No equipment entries are available.</p>}
      </>}
    </div>
  </Panel>;
}

export function WarningCard({record,send,disabled,compact=false}:{record:WarningRecord;send:(c:Control)=>void;disabled:boolean;compact?:boolean}){
  const w=record.warning,id=str(w.id),critical=w.severity==='critical',urgent=w.action==='explicit';
  return <article className={`warning-card ${critical?'critical':urgent?'warning':'cue'} ${compact?'compact':''}`} data-warning-id={id}>
    <div className="warning-top"><strong>{critical?'CRITICAL':urgent?'EXPLICIT WARNING':'SOFT CUE'} · {human(w.edge)}</strong><span>{fmt(w.stamp)}s sim · {record.acknowledged?'ACKNOWLEDGED':record.dismissed?'DISMISSED':'UNACKNOWLEDGED'}</span></div>
    <p>{str(w.explanation)}</p><div className="inline spread"><span className="warning-route">Suggested route: {arr(w.recommended).map(v=>human(v)).join(' → ')||'No verified alternative'}</span><div className="button-row">
      <button disabled={disabled||record.acknowledged} onClick={()=>send({command:'acknowledge_warning',hazard_id:str(w.hazard_id),warning_id:id})}>{record.acknowledged?'Acknowledged':'Acknowledge'}</button>
      {!record.dismissed&&<button className="quiet" disabled={disabled} onClick={()=>send({command:'dismiss_warning',warning_id:id})}>Dismiss</button>}
    </div></div><small>Hazard: {human(w.hazard_id)} · Policy {str(w.policy)} · {w.policy==='oracle'?'Oracle truth reference':'Inferred mismatch'} {percent(w.p_mismatch)}. Dismissal does not clear the hazard.</small>
  </article>;
}
