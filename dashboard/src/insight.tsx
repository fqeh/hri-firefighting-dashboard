// Explanatory panels for presentations: they only re-display published
// inference/decision telemetry; nothing here computes or invents state.
import {arr,obj,num,str,fmt,human,percent,type Dict,type Topic,type WarningRecord,type Control} from './contracts';
import {Panel,SourceTag,Stamp,ConceptChips} from './components';

const ACTION_LABEL:Record<string,string>={SILENT:'Stay silent',SOFT_CUE:'Soft cue',EXPLICIT_WARNING:'Explicit warning',OBSERVE:'Fly closer to confirm'};
export const POLICY_LABEL:Record<string,string>={B1:'Show everything, never alert',B2:'Alert when near a hazard',B3:'Alert on every new hazard',B4:'Belief-aware (ours)',oracle:'Knows the crew’s true belief (reference)'};
// "B4 · Belief-aware (ours)", matching the "F · Tree, crew unaware" case labels.
export const policyName=(p:string)=>POLICY_LABEL[p]?`${p==='oracle'?'Oracle':p} · ${POLICY_LABEL[p]}`:p;
const title=(edge:unknown)=>human(edge).replace(/\b\w/g,c=>c.toUpperCase());

// The belief-gap rule shared by the status card and the web 3D view: a hazard the
// robot has seen, a crew still heading into it, and an estimate above the bar to speak.
export type BeliefGap={focus:string;focusState:string;hazardSeen:boolean;approaching:boolean;offTask:boolean;p:number|null;threshold:number|null;gap:boolean};
export function beliefGap(topics:Record<string,Topic>,decision:Dict):BeliefGap{
  const route=obj(topics['/hri/rescue_route']?.data),belief=obj(topics['/hri/belief']?.data),observed=obj(route.edge_states);
  const context=obj(decision.context),p=num(context.probability),threshold=num(obj(decision.threshold).probability);
  const focus=str(decision.edge,str(belief.edge,'')),focusState=str(observed[focus],'unknown');
  const hazardSeen=Boolean(focus)&&['blocked','hazardous'].includes(focusState);
  const approaching=context.approaching===true,offTask=context.task_relevant===false;
  const gap=hazardSeen&&approaching&&p!==null&&(threshold===null?p>=.5:p>=threshold);
  return {focus,focusState,hazardSeen,approaching,offTask,p,threshold,gap};
}

// One sentence explaining the gap state, shared by the panels and the status card.
export function gapSentence(g:BeliefGap,oil:boolean){
  const who=oil?'operator':'crew',eye=oil?'robot':'UAV';
  const sees=`The ${eye} sees ${title(g.focus)} ${g.focusState==='blocked'?'blocked':'on fire'}`;
  return !g.hazardSeen?`No hazard on the ${who}’s routes has been observed yet. Both views agree as far as the ${eye} can tell.`
    :g.gap?`${sees}. The ${who} is still heading that way: ${percent(g.p)} likely they don’t know.`
    :g.approaching?`${sees} and the ${who} is heading there, but the estimate (${percent(g.p)}) is below the bar to speak (${percent(g.threshold)}).`
    :g.offTask?`${sees}, but it is not on the ${who}’s current task, so a wrong belief there would not endanger them yet.`
    :`${sees}, but the ${who}’s route already avoids it, so a wrong belief there would not endanger them.`;
}

export function DecisionPanel({decision,topic,oil=false}:{decision:Dict;topic?:Topic;oil?:boolean}){
  const who=oil?'operator':'crew',Who=oil?'Operator':'Crew';
  const context=obj(decision.context),p=num(context.probability),threshold=obj(decision.threshold),t=num(threshold.probability);
  const losses=Object.entries(obj(decision.losses)).filter(([,v])=>num(v)!==null) as [string,number][];
  const max=Math.max(1e-9,...losses.map(([,v])=>v)),best=losses.length?losses.reduce((a,b)=>b[1]<a[1]?b:a)[0]:null;
  const action=str(decision.action,'SILENT'),policy=str(decision.policy,'B4'),baselines=obj(decision.baselines);
  const counted=context.approaching===true&&context.task_relevant!==false;
  const distance=num(context.distance),urgency=num(context.urgency);
  const noHazard=!decision.hazard_id;
  return <Panel title={oil?'Should the robot warn the operator?':'Should the lookout speak?'} tag={<ConceptChips items={['mdp','vi']}/>} className="decision-panel" id="decision">
    <div className="panel-body">
      {noHazard?<p className="empty">No observed hazard on the {who}’s task yet. The {oil?'robot':'lookout'} stays silent and keeps watching.</p>:<>
      <div className="decision-verdict"><span className={`verdict ${action==='SILENT'?'silent':action==='OBSERVE'?'observe':'warn'}`}>{ACTION_LABEL[action]??human(action)}</span><span>Policy {policy} · {human(decision.classification)}</span><Stamp topic={topic}/></div>
      <div className="gauge-block">
        <div className="gauge-label"><span>P({who} holds a <b>dangerous false belief</b>)</span><strong>{percent(p)}</strong></div>
        <div className="gauge" role="meter" aria-label="Probability of a dangerous false belief" aria-valuemin={0} aria-valuemax={1} aria-valuenow={p??0}>
          <i className={`gauge-fill ${!counted?'muted':t!==null&&p!==null&&p>=t?'over':''}`} style={{width:percent(p)}}/>
          {t!==null&&<span className="gauge-threshold" style={{left:percent(t)}} data-testid="threshold"><em>speak above {percent(t)}</em></span>}
        </div>
        <p className="gauge-note">{!counted?`The ${who}’s route avoids this hazard, so silence carries no modeled risk and the bar does not apply.`
          :t===null?'No alert can beat silence for this hazard.'
          :`${Who} ${distance!==null?`${distance.toFixed(1)} m`:''} from the hazard${urgency===2?' (immediate danger)':urgency===1?' (close)':''}. The closer they are, the lower the bar to speak: a miss costs far more than a false alarm.`}</p>
      </div>
      <div className="loss-bars"><span className="eyebrow">EXPECTED LOSS OF EACH ACTION · LOWER IS BETTER</span>
        {losses.map(([name,value])=><div key={name} className={`loss-row ${name===best?'best':''} ${name===action?'chosen':''}`} data-action={name}>
          <span>{ACTION_LABEL[name]??human(name)}</span><div><i style={{width:`${Math.max(1.5,100*value/max)}%`}}/></div><b>{value.toFixed(2)}</b>
        </div>)}
        <p className="gauge-note">Silence costs P × harm of a missed warning. A warning costs the harm it fails to prevent, plus (1 − P) × the cost of a false alarm, plus the interruption.</p>
      </div>
      </>}
      <div className="policy-compare"><span className="eyebrow">SAME MOMENT, FOUR POLICIES</span>
        <div className="policy-grid">{['B1','B2','B3','B4'].map(b=>{const a=str(baselines[b],'');const warn=a==='SOFT_CUE'||a==='EXPLICIT_WARNING';
          return <div key={b} className={`policy-cell ${b===policy?'active':''}`} data-policy={b}><b>{b}</b><span>{POLICY_LABEL[b]}</span><em className={!a?'na':warn?'warn':'silent'}>{!a?'—':warn?'WARN':'SILENT'}</em></div>;})}</div>
        <p className="gauge-note">Computed from the same public context, ignoring repeat cooldowns. B3 also warns {oil?'an operator':'a crew'} that already knows: that is the false alarm B4 avoids.</p>
      </div>
      <details><summary>Why this decision</summary><p>{str(decision.reason,'Awaiting decision telemetry.')}</p><p>{str(decision.explanation,'')}</p></details>
    </div>
  </Panel>;
}

// Gazebo's native GUI, streamed by the container's noVNC and proxied by Vite at /gazebo.
export const GAZEBO_FULL='/gazebo/vnc.html?autoconnect=true&resize=scale&reconnect=true&path=gazebo/websockify';
export const GAZEBO_EMBED='/gazebo/vnc_lite.html?path=gazebo/websockify&scale=true';

// Status of the lookout as one explicit state plus the pipeline it runs:
// observe (sensors) → infer (Theory of Mind) → decide (MDP) → warn (trust).
type StepState='idle'|'ok'|'alert';
export function LookoutStatus({topics,decision,warnings,oil=false}:{topics:Record<string,Topic>;decision:Dict;warnings:WarningRecord[];oil?:boolean}){
  const who=oil?'operator':'crew',Who=oil?'Operator':'Crew',eye=oil?'robot':'UAV',Eye=oil?'Robot':'UAV';
  const g=beliefGap(topics,decision),action=str(decision.action,'SILENT'),speaks=action==='SOFT_CUE'||action==='EXPLICIT_WARNING';
  const hazards=arr(obj(topics['/hri/observations']?.data).hazards).map(obj);
  const kind=str(hazards.find(h=>str(h.edge)===g.focus)?.kind,'');
  const smoke=arr(obj(topics['/hri/observations']?.data).occluders).map(obj),smokeEdge=smoke.length?title(str(smoke[0].edge,'')):'';
  const smokeNote=smoke.length?` Right now smoke hides ${smokeEdge} from the ${eye}: the lookout cannot warn about hazards it cannot see.`:'';
  const latest=warnings.length?warnings[warnings.length-1]:null,active=[...warnings].reverse().find(w=>!w.acknowledged&&!w.dismissed);
  const escalation=obj(topics['/hri/escalation']?.data),trapped=escalation.active===true;
  const state=trapped?'trapped':active?'warned':g.gap?'gap':g.hazardSeen?'aware':'monitoring';
  const heading={trapped:'No escape route',monitoring:'Monitoring',aware:`Hazard seen · ${who} not at risk`,gap:'False belief detected',warned:'Warning sent'}[state];
  const explanation=state==='trapped'?`${str(escalation.message)} ${escalation.operator_dispatched_at!=null?'An operator has been dispatched.':'Waiting for the operator.'}`
    :state==='monitoring'
    ?`The ${eye} patrols the ${who}’s routes while the lookout infers what the ${who} believes from ${oil?'their commands':'how they move'}. It speaks only when a hazard is on their route and they probably don’t know about it.${smokeNote}`
    :state==='warned'?`The lookout warned the ${who} at ${fmt(active!.warning.stamp)} s. Waiting for their acknowledgement. ${gapSentence(g,oil)}`
    :gapSentence(g,oil);
  const steps:{name:string;lecture:string;value:string;state:StepState}[]=[
    {name:'Observe',lecture:'Sensors',state:g.hazardSeen||smoke.length?'alert':'ok',
      value:g.hazardSeen?`${Eye} sees ${title(g.focus)} ${g.focusState==='blocked'?'blocked':'hazardous'}${kind?` (${human(kind)})`:''}${smoke.length?` · smoke over ${smokeEdge}`:''}`:smoke.length?`Smoke hides ${smokeEdge} from the ${eye}`:hazards.length?`${hazards.length} hazard${hazards.length===1?'':'s'} seen · none on the ${who}’s route`:`No hazard seen on the ${who}’s routes`},
    {name:'Infer',lecture:'W2 · ToM + HMM',state:!g.hazardSeen?'idle':g.gap?'alert':'ok',
      value:!g.hazardSeen?'Nothing to infer yet':g.gap?`${Who} thinks it is open · ${percent(g.p)}`:g.approaching?`Heading there · ${percent(g.p)}, below the bar`:g.offTask?'Not on their task':'Their route avoids it'},
    {name:'Decide',lecture:'W3 · MDP',state:speaks?'alert':decision.action?'ok':'idle',
      value:decision.action?`${ACTION_LABEL[action]??human(action)} · ${str(decision.policy,'—')}${speaks&&g.threshold!==null?` (above ${percent(g.threshold)})`:''}`:'Waiting for the first decision'},
    {name:'Warn',lecture:'W1·D2 · Trust',state:active?'alert':latest?'ok':'idle',
      value:latest?`${latest.warning.action==='soft'?'Soft cue':'Explicit warning'} at ${fmt(latest.warning.stamp)} s · ${latest.acknowledged?'acknowledged':latest.dismissed?'dismissed':'awaiting acknowledgement'}`:'No warning sent'},
  ];
  return <section className={`status-card ${state}`} aria-label="Lookout status" data-testid="lookout-status">
    <div className="status-head"><span className="eyebrow">LOOKOUT STATUS</span><strong>{state==='gap'?'≠ ':state==='aware'?'= ':''}{heading}</strong><p>{explanation}</p></div>
    <ol className="pipeline">{steps.map((s,i)=><li key={s.name} className={s.state}><i>{s.state==='ok'?'✓':s.state==='alert'?'!':i+1}</i><div><b>{s.name}<em>{s.lecture}</em></b><span>{s.value}</span></div></li>)}</ol>
  </section>;
}

const HYPOTHESES:[string,string][]=[['open_safe','Open and safe'],['blocked','Blocked'],['open_hazardous','Open but hazardous'],['unknown_stale','Unknown or out of date']];
// The crew's inferred belief about the trail that matters right now (HMM posterior over four hypotheses).
export function CrewBeliefPanel({topics,decision,oil=false}:{topics:Record<string,Topic>;decision:Dict;oil?:boolean}){
  const who=oil?'operator':'crew',topic=topics['/hri/belief'],message=obj(topic?.data),g=beliefGap(topics,decision);
  const edge=g.focus||str(message.edge,''),belief=obj(obj(message.beliefs)[edge]??message),posterior=obj(belief.posterior);
  const action=obj(message.last_operator_evidence??message.last_movement_evidence);
  return <Panel title={`What the ${who} believes`} tag={<ConceptChips items={['tom','hmm']}/>} className="crew-belief" id="belief">
    <div className="panel-body">
      <p className="belief-sentence">{gapSentence(g,oil)}</p>
      <div className="belief-edge"><span>Belief about <b>{edge?title(edge):'the current route'}</b></span><Stamp topic={topic}/></div>
      {Object.keys(posterior).length?HYPOTHESES.map(([key,label])=><div className={`posterior-row h-${key}`} key={key}><span>{label}</span><strong>{percent(posterior[key])}</strong><div role="meter" aria-label={label} aria-valuemin={0} aria-valuemax={1} aria-valuenow={num(posterior[key])??0}><i style={{width:percent(posterior[key])}}/></div></div>)
        :<p className="help">Waiting for the first belief estimate.</p>}
      <details><summary>Evidence behind this estimate</summary><p>{Object.keys(action).length?`${human(action.origin)} → ${human(action.target)} at ${fmt(action.stamp)} s; ${str(action.evidence,oil?'explicit operator waypoint selection':'movement observed in Gazebo')}.`:'No informative action has been confirmed yet.'}</p><p>{str(decision.explanation,'')}</p></details>
      <p className="panel-note">An estimate of the {who}’s understanding, not their hidden true belief.</p>
    </div>
  </Panel>;
}

// Escalation to the human operator: every escape route is closed, so warning the crew is
// no longer enough. Published by the rescue route planner (/hri/escalation).
export function EscalationAlert({topic,send,disabled,compact=false}:{topic?:Topic;send:(c:Control)=>void;disabled:boolean;compact?:boolean}){
  const e=obj(topic?.data);
  if(e.active!==true)return null;
  const dispatched=num(e.operator_dispatched_at),edges=arr(e.blocked_edges).map(v=>title(v));
  const action=dispatched===null
    ?<button className="danger-button" disabled={disabled} onClick={()=>send({command:'send_operator'})}>Send operator</button>
    :<span className="escalation-done">Operator dispatched at {fmt(dispatched)} s</span>;
  if(compact)return <div className="escalation-banner" role="alert"><strong>🚨 No escape route</strong><span>All trails to the safety zone are blocked.</span>{action}</div>;
  return <section className="escalation" role="alert" data-testid="escalation">
    <div className="escalation-head"><span className="eyebrow">ESCALATION · HUMAN OPERATOR</span><span>since {fmt(e.since)} s</span></div>
    <strong>🚨 No escape route: send an operator</strong>
    <p>{str(e.message)}</p>
    <p className="escalation-edges">Blocked: {edges.join(' · ')||'—'}</p>
    {action}
    <small>The rescue itself is not simulated; the dispatch is logged with its time.</small>
  </section>;
}
