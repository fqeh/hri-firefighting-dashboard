import {useId} from 'react';
import {arr,obj,point,str,num,human,type Dict,type Topic,type Point} from './contracts';

export function WorldMap({topics,parameters,showTruth}:{topics:Record<string,Topic>;parameters:Dict;showTruth:boolean}){
  const grid=useId(),routes=obj(topics['/hri/rescue_route']?.data),intent=obj(topics['/hri/human_intent']?.data);
  const nodes=obj(routes.nodes??parameters.nodes),edges=arr(parameters.edges).map(obj);
  const robotRoute=obj(topics['/hri/robot_route']?.data),origin=point(parameters.robot_start)??[0,0];
  const positions=['human','uav','robot'].flatMap(name=>{
    const topic=topics[`/${name}/odom`],p=point(topic?.data.position);if(!p)return [];
    const world:Point=name==='robot'?(point(robotRoute.crew_position)??[p[0]+origin[0],p[1]+origin[1]]):p;
    return [{name:name==='human'?'Crew':name==='uav'?'UAV':'Ground robot',point:world,source:topic.source,stale:topic.stale,stamp:topic.sim_stamp}];
  });
  const hazards=arr(topics['/hri/observations']?.data.hazards).map(obj).filter(h=>point(h.position));
  const all=[...Object.values(nodes).flatMap(p=>point(p)?[point(p)!]:[]),...positions.map(p=>p.point),...hazards.map(h=>point(h.position)!)];
  if(!all.length)return <div className="empty">Waiting for a public world map and ROS positions.</div>;
  const minX=Math.min(...all.map(p=>p[0]))-3,maxX=Math.max(...all.map(p=>p[0]))+3,minY=Math.min(...all.map(p=>p[1]))-3,maxY=Math.max(...all.map(p=>p[1]))+3;
  const scale=Math.min(700/(maxX-minX),310/(maxY-minY));
  const offsetX=(760-(maxX-minX)*scale)/2,offsetY=(370-(maxY-minY)*scale)/2;
  const xy=(p:Point):Point=>[offsetX+(p[0]-minX)*scale,370-offsetY-(p[1]-minY)*scale];
  const line=(names:unknown)=>arr(names).flatMap(n=>{const p=point(nodes[str(n)]);return p?[xy(p).join(',')]:[];}).join(' ');
  const states=obj(routes.edge_states),safe=point(nodes[str(parameters.goal)]);
  return <svg viewBox="0 0 760 370" role="img" aria-label="Live world map: ROS positions, observed hazards, intended and recommended routes" className="world-map">
    <defs><pattern id={grid} width="28" height="28" patternUnits="userSpaceOnUse"><path d="M28 0H0V28" fill="none" stroke="#34474a" strokeWidth=".5"/></pattern></defs>
    <rect width="760" height="370" fill={`url(#${grid})`}/>
    {edges.map(e=>{const a=point(nodes[str(e.source)]),b=point(nodes[str(e.target)]);if(!a||!b)return null;
      const state=str(states[str(e.id)],'unknown'),[x1,y1]=xy(a),[x2,y2]=xy(b);
      return <g key={str(e.id)} data-edge={str(e.id)} data-state={state}><title>{human(e.id)}: {state} · inferred from public observations</title>
        <line x1={x1} y1={y1} x2={x2} y2={y2} stroke="#22393c" strokeWidth="14" strokeLinecap="round"/>
        <line x1={x1} y1={y1} x2={x2} y2={y2} className={`edge edge-${state}`}/>
        {state==='blocked'&&<text x={(x1+x2)/2} y={(y1+y2)/2-10} className="blocked-label">× BLOCKED</text>}
      </g>;
    })}
    {showTruth&&arr(topics['/forest/fire_state']?.data.zones).map((v,i)=>{const zone=obj(v);return <polygon key={i} points={arr(zone.polygon).flatMap(p=>point(p)?[xy(point(p)!).join(',')]:[]).join(' ')} fill="#ff802744" stroke="#ffb166" strokeDasharray="2 4"><title>Fire model ground truth · unvalidated research envelope</title></polygon>;})}
    <polyline points={line(intent.path)} className="intended-line"><title>Intended route · observable action/intent</title></polyline>
    <polyline points={line(routes.recommended)} className="recommended-line"><title>Recommended route · risk inference</title></polyline>
    {safe&&<g transform={`translate(${xy(safe).join(',')})`}><rect x="-16" y="-16" width="32" height="32" rx="7" fill="#124b3b" stroke="#67deaa"/><path d="M-7 0h14M0-7v14" stroke="#91f4c5" strokeWidth="3"/><text x="22" y="-10" className="safe-label">SAFETY ZONE</text><title>Designated safety zone · public map configuration; current safety is assessed by the route planner</title></g>}
    {hazards.map(h=>{const [x,y]=xy(point(h.position)!);return <g key={str(h.id)} opacity={h.observation_stale===true?.5:1} data-hazard={str(h.id)}>
      <title>{`${human(h.kind)} · observed by ${str(h.source)} at ${str(h.last_seen_stamp)}s. ${h.observation_stale===true?'Last seen; not a current observation.':''}`}</title>
      <circle cx={x} cy={y} r={Math.max(9,(num(h.radius)??1)*scale)} className={h.kind==='gas'?'gas-zone':'hazard-zone'}/>
      <text x={x+9} y={y-14} className="hazard-label">{human(h.kind).toUpperCase()}{h.observation_stale===true?' · LAST SEEN':''}</text>
    </g>;})}
    {Object.entries(nodes).map(([id,v])=>{const p=point(v);if(!p)return null;const [x,y]=xy(p);return <g key={id}><circle cx={x} cy={y} r="3" fill="#c2d5d1"/><text x={x+6} y={y+20} className="node-label">{human(id)}</text></g>;})}
    {showTruth&&arr(parameters.personnel).map((v,i)=>{const p=point(v);if(!p)return null;const [x,y]=xy(p);return <g key={i}><circle cx={x} cy={y} r="6" fill="#c4b1fc"/><text x={x+10} y={y+4} className="node-label">Personnel · configured</text><title>Static personnel location · simulation configuration truth, not robot observation</title></g>;})}
    {positions.map(e=>{const [x,y]=xy(e.point);const color=e.name==='Crew'?'#d0b2ff':e.name==='UAV'?'#f8d47e':'#8ce3e4';return <g key={e.name} opacity={e.stale?.45:1} data-entity={e.name}>
      <title>{`${e.name} · ${e.source} · ${str(e.stamp)}s${e.stale?' · stale':''}`}</title><circle cx={x} cy={y} r="13" fill={color} opacity=".18"/><circle cx={x} cy={y} r="5" fill={color} stroke="#152b2d" strokeWidth="2"/>
      <text x={x+14} y={y-7} fill={color} fontSize="12" fontWeight="650">{e.name}{e.stale?' · stale':''}</text>
    </g>;})}
    <text x="18" y="350" className="map-unit">WORLD FRAME · METRES</text>
  </svg>;
}
