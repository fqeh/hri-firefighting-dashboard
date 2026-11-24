// Browser-side 3D view of the Gazebo world (three.js, Z-up like Gazebo).
// The static geometry is Gazebo's own SDF (scripts/scene_export.py); everything
// that moves or is inferred comes from the same ROS telemetry the panels use.
// Rendering here uses the viewer's GPU, so the server's software-rendered
// Gazebo window can stay paused.
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {CSS2DRenderer,CSS2DObject} from 'three/addons/renderers/CSS2DRenderer.js';
import type {Point,Scene,SceneItem,SceneSensor,SceneSpawn,Vec3} from './contracts';

export type LayerId='world'|'trees'|'seen'|'belief'|'route'|'truth'|'labels';
export type LayerState=Record<LayerId,{on:boolean;opacity:number}>;
export type BeliefLook='open'|'blocked'|'hazard'|'unknown'|'none';
export type EntityView={key:string;label:string;position:Vec3;yaw:number;stale:boolean};
export type HazardView={id:string;kind:string;edge:string;position:Vec3;radius:number;stale:boolean};
export type LiveView={
  nodes:Record<string,Point>;edges:{id:string;a:Point;b:Point}[];
  entities:EntityView[];hazards:HazardView[];edgeStates:Record<string,string>;
  belief:Record<string,{look:BeliefLook;p:number}>;intent:string[];recommended:string[];
  gap:{edge:string;kind:'gap'|'agree';text:string}|null;fireZones:Point[][];who:string;
  smoke:{id:string;edge:string;position:Vec3;radius:number}[];
};
export type CameraStatus={cursor:Point|null;pitch:number;bearing:number;distance:number};

export const BELIEF_HEIGHT=6.5;
// Forest vegetation gets its own layer so the canopy can be hidden to see the trails.
export const isVegetation=(name:string)=>/^(tree|understory)_\d/.test(name);
export const PALETTE={clear:'#3fbf7f',blocked:'#e5484d',hazardous:'#f59e0b',unknown:'#9aa8a6',open:'#4ade80',belief:'#a78bfa',intent:'#c4b5fd',route:'#14b8a6',sensor:'#38bdf8',fire:'#ff7a1a',gas:'#c9d64a'};
const HAZARD_RING:Record<string,string>={tree:PALETTE.blocked,debris:PALETTE.blocked,fire:PALETTE.fire,gas:PALETTE.gas};
const LOOK_COLOR:Record<BeliefLook,string>={open:PALETTE.open,blocked:PALETTE.blocked,hazard:PALETTE.hazardous,unknown:PALETTE.unknown,none:PALETTE.belief};
const SPAWN_LABEL:Record<string,string>={human:'Crew',uav:'UAV',robot:'Ground robot'};
const TITLE=(s:string)=>s.replaceAll('_',' ').replace(/\b\w/g,c=>c.toUpperCase());

function geometry(item:SceneItem):THREE.BufferGeometry{
  switch(item.shape){
    case 'box':return new THREE.BoxGeometry(...item.size);
    case 'cylinder':return new THREE.CylinderGeometry(item.radius,item.radius,item.length,28).rotateX(Math.PI/2);
    case 'sphere':return new THREE.SphereGeometry(item.radius,24,16);
    case 'ellipsoid':return new THREE.SphereGeometry(1,24,16).scale(...item.radii);
    case 'mesh':{const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(item.vertices,3));g.setIndex(item.indices);g.computeVertexNormals();return g;}
  }
}

function disposeTree(root:THREE.Object3D){
  root.traverse(o=>{
    if(o instanceof CSS2DObject)o.element.remove();  // labels live outside the WebGL canvas
    const mesh=o as THREE.Mesh;
    mesh.geometry?.dispose();
    const m=mesh.material;if(Array.isArray(m))m.forEach(x=>x.dispose());else m?.dispose();
  });
}
function clear(group:THREE.Group){for(const child of [...group.children]){group.remove(child);disposeTree(child);}}

function label(text:string,kind:string):CSS2DObject{
  const el=document.createElement('div');el.className=`label3d ${kind}`;el.textContent=text;
  return new CSS2DObject(el);
}

export class WorldRenderer{
  private renderer:THREE.WebGLRenderer;private labels=new CSS2DRenderer();
  private scene=new THREE.Scene();private camera=new THREE.PerspectiveCamera(42,1.6,.1,600);
  private controls:OrbitControls;private sun=new THREE.DirectionalLight('#fff6e8',2.2);
  private layer:Record<LayerId,THREE.Group>={world:new THREE.Group(),trees:new THREE.Group(),seen:new THREE.Group(),belief:new THREE.Group(),route:new THREE.Group(),truth:new THREE.Group(),labels:new THREE.Group()};
  private part:Record<string,THREE.Group>={};
  private entityTemplates:Record<string,SceneItem[]>={};private spawns:Record<string,SceneSpawn>={};private hazardTemplates:Record<string,SceneItem[]>={};private sensors:Record<string,SceneSensor>={};
  private entities=new Map<string,{group:THREE.Group;target:THREE.Vector3;yaw:number;tag:CSS2DObject;placed:boolean}>();
  private hazards=new Map<string,{group:THREE.Group;ring:THREE.Mesh;kind:string;phase:number}>();
  private signatures:Record<string,string>={};private framed=false;private bounds={center:new THREE.Vector3(),span:20};
  private routeCurve:THREE.CurvePath<THREE.Vector3>|null=null;private flow:THREE.Mesh[]=[];
  private thought:{marker:THREE.Mesh;stem:THREE.Line}|null=null;
  private follow:string|null=null;private followLast=new THREE.Vector3();
  private flight:{from:THREE.Vector3;to:THREE.Vector3;fromTarget:THREE.Vector3;toTarget:THREE.Vector3;start:number}|null=null;
  private timer=new THREE.Timer();private ready=false;private cursor:Point|null=null;private lastStatus=0;
  private resize:ResizeObserver;private ray=new THREE.Raycaster();private ground=new THREE.Plane(new THREE.Vector3(0,0,1),0);

  constructor(private host:HTMLElement,private layers:LayerState,private onStatus:(s:CameraStatus)=>void){
    this.renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
    this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFShadowMap;
    this.renderer.domElement.className='world3d-canvas';
    this.labels.domElement.className='world3d-labels';
    host.append(this.renderer.domElement,this.labels.domElement);
    this.camera.up.set(0,0,1);this.camera.position.set(-6,-24,20);
    this.controls=new OrbitControls(this.camera,this.renderer.domElement);
    Object.assign(this.controls,{enableDamping:true,dampingFactor:.08,screenSpacePanning:false,minDistance:3,maxDistance:180,maxPolarAngle:Math.PI*.47});
    this.scene.add(new THREE.HemisphereLight('#e8f3ff','#5b5a3c',1.4));
    this.sun.position.set(-25,-35,45);this.sun.castShadow=true;this.sun.shadow.mapSize.set(2048,2048);this.sun.shadow.bias=-.0004;
    this.scene.add(this.sun,this.sun.target);
    for(const group of Object.values(this.layer))this.scene.add(group);
    for(const [name,layer] of [['scenery','world'],['trees','trees'],['entities','world'],['sensors','seen'],['hazards','seen'],['hazardLabels','seen'],['smoke','seen'],['seenEdges','seen'],['mind','belief'],['intent','belief'],['gap','belief'],['route','route'],['truth','truth'],['nodeLabels','labels']] as [string,LayerId][]){
      this.part[name]=new THREE.Group();this.layer[layer].add(this.part[name]);
    }
    this.resize=new ResizeObserver(()=>this.fit());this.resize.observe(host);this.fit();
    this.renderer.domElement.addEventListener('pointermove',this.pointer);
    this.renderer.domElement.addEventListener('pointerleave',this.leave);
    this.setLayers(layers);
    this.renderer.setAnimationLoop(this.frame);
  }

  dispose(){
    this.renderer.setAnimationLoop(null);this.resize.disconnect();this.controls.dispose();
    this.renderer.domElement.removeEventListener('pointermove',this.pointer);
    this.renderer.domElement.removeEventListener('pointerleave',this.leave);
    for(const group of Object.values(this.layer)){clear(group);}
    this.renderer.dispose();this.renderer.domElement.remove();this.labels.domElement.remove();this.timer.dispose();
  }

  private fit=()=>{
    const w=Math.max(1,this.host.clientWidth),h=Math.max(1,this.host.clientHeight);
    this.renderer.setSize(w,h,false);this.labels.setSize(w,h);
    this.camera.aspect=w/h;this.camera.updateProjectionMatrix();
  };

  private material(layer:LayerId,color:string|THREE.Color,alpha=1,extra:THREE.MeshStandardMaterialParameters={}){
    const m=new THREE.MeshStandardMaterial({color,roughness:.88,metalness:0,...extra});
    m.userData.base=alpha;m.userData.orig=alpha;m.userData.layer=layer;this.tint(m);return m;
  }
  private tint(m:THREE.Material){
    const opacity=(m.userData.base as number)*this.layers[m.userData.layer as LayerId].opacity;
    m.opacity=opacity;m.transparent=opacity<.999;m.depthWrite=opacity>=.999||(m.userData.base as number)>=.999&&opacity>.6;
  }

  private build(items:SceneItem[],layer:LayerId,shadows=true):THREE.Group{
    const group=new THREE.Group();
    for(const item of items){
      const color=new THREE.Color().setRGB(item.color[0],item.color[1],item.color[2],THREE.SRGBColorSpace);
      const mesh=new THREE.Mesh(geometry(item),this.material(layer,color,item.color[3],item.color[3]<1?{side:THREE.DoubleSide}:{}));
      mesh.position.set(...item.position);mesh.quaternion.set(...item.quaternion);
      const flat=item.shape==='box'&&Math.min(...item.size)<.05;
      mesh.castShadow=shadows&&!flat&&item.color[3]>=.999&&item.shape!=='mesh';mesh.receiveShadow=true;
      mesh.name=item.name;group.add(mesh);
    }
    return group;
  }

  setScene(scene:Scene){
    clear(this.part.scenery);clear(this.part.trees);clear(this.part.smoke);clear(this.part.entities);clear(this.part.hazards);clear(this.part.sensors);clear(this.part.hazardLabels);
    this.entities.clear();this.hazards.clear();this.signatures={};this.framed=false;
    if(this.thought){for(const o of [this.thought.marker,this.thought.stem]){o.removeFromParent();disposeTree(o);}this.thought=null;}
    const bg=scene.background?new THREE.Color().setRGB(scene.background[0],scene.background[1],scene.background[2],THREE.SRGBColorSpace):new THREE.Color('#a9c6d8');
    this.scene.background=bg;this.scene.fog=new THREE.Fog(bg,70,220);
    this.part.scenery.add(this.build(scene.scenery.filter(i=>!isVegetation(i.name)),'world'));
    this.part.trees.add(this.build(scene.scenery.filter(i=>isVegetation(i.name)),'trees'));
    this.entityTemplates=scene.entities;this.hazardTemplates=scene.hazards;this.sensors=scene.sensors;this.spawns=scene.spawns??{};this.ready=true;
  }

  private frame=(time:number)=>{
    this.timer.update(time);
    if(!this.host.isConnected||this.host.clientWidth===0)return;  // hidden tab panel
    const dt=Math.min(this.timer.getDelta(),.1),t=this.timer.getElapsed(),k=1-Math.exp(-dt*6);
    for(const [key,e] of this.entities){
      if(!e.placed){e.group.position.copy(e.target);e.group.rotation.z=e.yaw;e.placed=true;}
      e.group.position.lerp(e.target,k);
      const d=Math.atan2(Math.sin(e.yaw-e.group.rotation.z),Math.cos(e.yaw-e.group.rotation.z));e.group.rotation.z+=d*k;
      const sensor=this.part.sensors.getObjectByName(`sensor:${key}`);
      if(sensor){sensor.position.set(e.group.position.x,e.group.position.y,key==='uav'?e.group.position.z:.04);sensor.rotation.z=e.group.rotation.z;
        if(key==='uav')sensor.scale.setScalar(Math.max(.1,e.group.position.z)/8);}  // frustum built for 8 m altitude
    }
    const crew=this.entities.get('human');
    if(this.thought&&crew){
      const p=crew.group.position;this.thought.marker.position.set(p.x,p.y,BELIEF_HEIGHT+.35);
      this.thought.stem.position.set(p.x,p.y,0);
    }
    for(const h of this.hazards.values()){
      const pulse=1+.07*Math.sin(t*3+h.phase);h.ring.scale.set(pulse,pulse,1);
      if(h.kind==='fire')h.group.children[0]?.scale.set(1,1,1+.09*Math.sin(t*9+h.phase));
    }
    this.part.smoke.children.forEach((cloud,i)=>{if(cloud.userData.drift)cloud.rotation.z=t*.05+i;});
    if(this.routeCurve&&this.flow.length){
      const length=this.routeCurve.getLength();
      this.flow.forEach((dot,i)=>{const u=((t*1.6)/length+i/this.flow.length)%1;dot.position.copy(this.routeCurve!.getPointAt(u));});
    }
    if(this.follow){
      const e=this.entities.get(this.follow);
      if(e){const delta=e.group.position.clone().sub(this.followLast);this.camera.position.add(delta);this.controls.target.add(delta);this.followLast.copy(e.group.position);}
    }
    if(this.flight){
      const u=Math.min(1,(performance.now()-this.flight.start)/700),s=u*u*(3-2*u);
      this.camera.position.lerpVectors(this.flight.from,this.flight.to,s);this.controls.target.lerpVectors(this.flight.fromTarget,this.flight.toTarget,s);
      if(u>=1)this.flight=null;
    }
    this.controls.update();
    this.renderer.render(this.scene,this.camera);
    if(this.layers.labels.on)this.labels.render(this.scene,this.camera);
    if(performance.now()-this.lastStatus>150){this.lastStatus=performance.now();this.emit();}
  };

  private emit(){
    const dir=this.controls.target.clone().sub(this.camera.position);
    this.onStatus({cursor:this.cursor,pitch:THREE.MathUtils.radToDeg(Math.atan2(-dir.z,Math.hypot(dir.x,dir.y))),
      bearing:(THREE.MathUtils.radToDeg(Math.atan2(dir.x,dir.y))+360)%360,distance:dir.length()});
  }
  private pointer=(e:PointerEvent)=>{
    const r=this.renderer.domElement.getBoundingClientRect();
    this.ray.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),this.camera);
    const hit=this.ray.ray.intersectPlane(this.ground,new THREE.Vector3());this.cursor=hit?[hit.x,hit.y]:null;
  };
  private leave=()=>{this.cursor=null;};

  setLayers(layers:LayerState){
    this.layers=layers;
    for(const [id,group] of Object.entries(this.layer) as [LayerId,THREE.Group][]){
      group.visible=layers[id].on;
      group.traverse(o=>{const m=(o as THREE.Mesh).material;if(m&&!Array.isArray(m)&&m.userData.layer)this.tint(m);});
    }
    this.labels.domElement.style.display=layers.labels.on?'':'none';
    this.labels.domElement.style.opacity=String(layers.labels.opacity);
  }

  setFollow(key:string|null){this.follow=key;const e=key?this.entities.get(key):null;if(e)this.followLast.copy(e.group.position);}

  view(kind:'home'|'top'){
    // Narrow (portrait) viewports see less horizontally, so back the camera off accordingly.
    const {center}=this.bounds,span=this.bounds.span*Math.max(1,1.15/this.camera.aspect);
    const to=kind==='top'?center.clone().add(new THREE.Vector3(0,-.02*span,1.35*span)):center.clone().add(new THREE.Vector3(-.3*span,-1.2*span,1.3*span));
    this.flight={from:this.camera.position.clone(),to,fromTarget:this.controls.target.clone(),toTarget:center.clone(),start:performance.now()};
  }

  // Presentation figures (capture mode): place the camera on a point or a moving agent.
  focusOn(target:Vec3,distance:number,pitchDeg=45,bearingDeg=20){
    const p=THREE.MathUtils.degToRad(pitchDeg),b=THREE.MathUtils.degToRad(bearingDeg),t=new THREE.Vector3(...target);
    this.flight=null;this.follow=null;this.controls.target.copy(t);
    this.camera.position.set(t.x-Math.sin(b)*Math.cos(p)*distance,t.y-Math.cos(b)*Math.cos(p)*distance,t.z+Math.sin(p)*distance);
    this.controls.update();
  }
  setLayerVisible(id:LayerId,on:boolean){this.setLayers({...this.layers,[id]:{...this.layers[id],on}});}
  entityPosition(key:string):Vec3|null{const e=this.entities.get(key);return e?[e.group.position.x,e.group.position.y,e.group.position.z]:null;}

  update(view:LiveView){
    if(!this.ready)return;  // entities are built from the scene's Gazebo models
    if(!this.framed&&Object.keys(view.nodes).length){
      const box=new THREE.Box2();for(const p of Object.values(view.nodes))box.expandByPoint(new THREE.Vector2(...p));
      const c=box.getCenter(new THREE.Vector2()),size=box.getSize(new THREE.Vector2());
      this.bounds={center:new THREE.Vector3(c.x,c.y,0),span:Math.max(14,size.x,size.y)};
      const s=this.sun.shadow.camera,half=this.bounds.span*1.6;Object.assign(s,{left:-half,right:half,top:half,bottom:-half,far:200});s.updateProjectionMatrix();
      this.sun.target.position.copy(this.bounds.center);this.sun.position.copy(this.bounds.center).add(new THREE.Vector3(-25,-35,45));
      this.framed=true;this.view('home');this.flight&&(this.flight.start-=700);
    }
    this.updateEntities(view);
    this.once('nodes',[view.nodes,view.edges],()=>this.drawNodes(view));
    this.once('hazards',view.hazards,()=>this.drawHazards(view));
    this.once('seenEdges',[view.edges,view.edgeStates],()=>this.drawSeenEdges(view));
    this.once('mind',[view.edges,view.nodes,view.belief,view.who],()=>this.drawMind(view));
    this.once('intent',[view.intent,view.nodes],()=>this.drawPath(this.part.intent,view.intent,view.nodes,'intent'));
    this.once('gap',[view.gap,view.edges],()=>this.drawGap(view));
    this.once('route',[view.recommended,view.nodes],()=>this.drawRoute(view));
    this.once('truth',view.fireZones,()=>this.drawTruth(view));
    this.once('smoke',view.smoke,()=>this.drawSmoke(view));
  }
  private once(key:string,value:unknown,draw:()=>void){
    const signature=JSON.stringify(value,(_,v)=>typeof v==='number'?Math.round(v*20)/20:v);
    if(this.signatures[key]!==signature){this.signatures[key]=signature;draw();}
  }

  private updateEntities(view:LiveView){
    // Until Gazebo publishes odometry (e.g. paused at t=0), show agents where the world file spawns them.
    const live=new Set(view.entities.map(e=>e.key));
    const waiting=Object.entries(this.spawns).filter(([key])=>!live.has(key)&&this.entityTemplates[key])
      .map(([key,s])=>({key,label:`${SPAWN_LABEL[key]??key} · start`,position:s.position,yaw:s.yaw,stale:false}));
    const seen=new Set<string>();
    for(const e of [...view.entities,...waiting]){
      seen.add(e.key);let entry=this.entities.get(e.key);
      if(!entry){
        const template=this.entityTemplates[e.key];
        const group=template?this.build(template,'world'):new THREE.Group();
        if(!template){const m=new THREE.Mesh(new THREE.CapsuleGeometry(.3,.9,4,12).rotateX(Math.PI/2),this.material('world','#d0b2ff'));m.position.z=.8;group.add(m);}
        const tag=label(e.label,`entity ${e.key}`);tag.position.set(0,0,e.key==='uav'?.9:1.75);group.add(tag);
        this.part.entities.add(group);
        entry={group,target:new THREE.Vector3(),yaw:0,tag,placed:false};this.entities.set(e.key,entry);
        this.addSensor(e.key);
        if(e.key==='human')this.addThought();
      }
      entry.target.set(...e.position);entry.yaw=e.yaw;
      entry.tag.element.textContent=e.label+(e.stale?' · stale':'');entry.tag.element.classList.toggle('stale',e.stale);
    }
    for(const [key,entry] of this.entities)if(!seen.has(key)){this.part.entities.remove(entry.group);disposeTree(entry.group);this.entities.delete(key);
      const s=this.part.sensors.getObjectByName(`sensor:${key}`);if(s){this.part.sensors.remove(s);disposeTree(s);}}
  }

  // The logical-camera coverage the observations actually come from.
  private addSensor(key:string){
    const s=this.sensors[key];if(!s||key==='human')return;
    const group=new THREE.Group();group.name=`sensor:${key}`;
    const fill=this.material('seen',PALETTE.sensor,.07,{side:THREE.DoubleSide,emissive:PALETTE.sensor,emissiveIntensity:.4});
    if(Math.abs(s.pitch)>1){  // looking down: square frustum to the ground
      const h=8,half=h*Math.tan(s.horizontal_fov/2);
      const cone=new THREE.Mesh(new THREE.CylinderGeometry(.02,half*Math.SQRT2,h,4,1,true).rotateY(Math.PI/4).rotateX(Math.PI/2).translate(0,0,-h/2),fill);
      group.add(cone);
      const outline=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.PlaneGeometry(2*half,2*half)),new THREE.LineBasicMaterial({color:PALETTE.sensor,transparent:true}));
      outline.material.userData={base:1,orig:1,layer:'seen'};this.tint(outline.material);
      outline.position.z=-h+.08;group.add(outline);
    }else{
      const wedge=new THREE.Mesh(new THREE.CircleGeometry(s.far,48,-s.horizontal_fov/2,s.horizontal_fov),fill);
      group.add(wedge);
    }
    this.part.sensors.add(group);
  }

  // Dashed stem from the crew up to their inferred "mental map".
  private addThought(){
    const marker=new THREE.Mesh(new THREE.SphereGeometry(.32,20,14),this.material('belief',PALETTE.belief,1,{emissive:PALETTE.belief,emissiveIntensity:.5}));
    const stem=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,1.3),new THREE.Vector3(0,0,BELIEF_HEIGHT+.2)]),new THREE.LineDashedMaterial({color:PALETTE.belief,dashSize:.3,gapSize:.22,transparent:true,opacity:.85}));
    stem.computeLineDistances();stem.material.userData={base:.85,orig:.85,layer:'belief'};
    this.part.mind.parent!.add(marker,stem);this.thought={marker,stem};
  }

  private ribbon(group:THREE.Group,a:Point,b:Point,z:number,width:number,color:string,layer:LayerId,alpha=1,dashed=false){
    const dx=b[0]-a[0],dy=b[1]-a[1],length=Math.hypot(dx,dy);if(length<1e-3)return;
    const yaw=Math.atan2(dy,dx),material=this.material(layer,color,alpha,{emissive:color,emissiveIntensity:.25,side:THREE.DoubleSide});
    const pieces=dashed?Math.max(1,Math.floor(length/.9)):1,piece=dashed?length/pieces*.55:length;
    for(let i=0;i<pieces;i++){
      const u=dashed?(i+.5)/pieces:.5;
      const m=new THREE.Mesh(new THREE.PlaneGeometry(piece,width),material);
      m.position.set(a[0]+dx*u,a[1]+dy*u,z);m.rotation.z=yaw;m.renderOrder=2;group.add(m);
    }
  }

  private drawNodes(view:LiveView){
    clear(this.part.nodeLabels);
    for(const [id,p] of Object.entries(view.nodes)){const tag=label(TITLE(id),'node');tag.position.set(p[0],p[1],.2);this.part.nodeLabels.add(tag);}
  }

  private drawSeenEdges(view:LiveView){
    const g=this.part.seenEdges;clear(g);
    for(const e of view.edges){
      const state=view.edgeStates[e.id]??'unknown';
      const color=state==='blocked'?PALETTE.blocked:state==='hazardous'?PALETTE.hazardous:state==='clear'?PALETTE.clear:PALETTE.unknown;
      this.ribbon(g,e.a,e.b,.045,state==='blocked'?.75:.45,color,'seen',state==='unknown'?.75:.95,state==='unknown');
      if(state==='blocked'||state==='hazardous'){
        const tag=label(state==='blocked'?`✕ ${TITLE(e.id)} blocked`:`${TITLE(e.id)} hazardous`,state);
        tag.position.set((e.a[0]+e.b[0])/2,(e.a[1]+e.b[1])/2,.3);tag.center.set(.5,-.6);g.add(tag);
      }
    }
  }

  private drawHazards(view:LiveView){
    const ids=new Set(view.hazards.map(h=>h.id));
    for(const [id,h] of this.hazards)if(!ids.has(id)){this.part.hazards.remove(h.group);disposeTree(h.group);this.hazards.delete(id);}
    for(const h of view.hazards){
      let entry=this.hazards.get(h.id);
      if(!entry){
        // A fully developed fire with smoke has its own model; logs and debris lie across their trail.
        const template=this.hazardTemplates[h.id.startsWith('hazard_fire_smoke')?'fire_smoke':h.kind];
        const group=new THREE.Group(),body=template?this.build(template,'seen'):new THREE.Group();
        if(!template)body.add(new THREE.Mesh(new THREE.ConeGeometry(.6,1.6,20).rotateX(Math.PI/2).translate(0,0,.8),this.material('seen',PALETTE.blocked)));
        const ring=new THREE.Mesh(new THREE.RingGeometry(Math.max(.3,h.radius-.12),h.radius,64),this.material('seen',HAZARD_RING[h.kind]??PALETTE.blocked,.8,{side:THREE.DoubleSide,emissive:HAZARD_RING[h.kind]??PALETTE.blocked,emissiveIntensity:.6}));
        group.add(body,ring);this.part.hazards.add(group);
        entry={group,ring,kind:h.kind,phase:Math.random()*6};this.hazards.set(h.id,entry);
      }
      entry.group.position.set(...h.position);entry.ring.position.z=-h.position[2]+.06;
      const edge=view.edges.find(e=>e.id===h.edge);
      if(edge&&(h.kind==='tree'||h.kind==='debris'))entry.group.rotation.z=Math.atan2(edge.b[1]-edge.a[1],edge.b[0]-edge.a[0]);
      entry.group.traverse(o=>{const m=(o as THREE.Mesh).material as THREE.Material|undefined;if(m?.userData.layer){m.userData.base=(m.userData.orig as number)*(h.stale?.45:1);this.tint(m);}});
    }
    // One label per kind and route segment ("FIRE ×12"), like the 2D map.
    clear(this.part.hazardLabels);
    const groups=new Map<string,{kind:string;count:number;stale:boolean;at:Vec3}>();
    for(const h of view.hazards){const kind=h.id.startsWith('hazard_fire_smoke')?'fire + smoke':h.kind,key=`${kind}:${h.edge}`,g=groups.get(key);
      if(!g)groups.set(key,{kind,count:1,stale:h.stale,at:h.position});else{g.count++;g.stale&&=h.stale;}}
    for(const g of groups.values()){
      const tag=label(`${g.kind.toUpperCase()}${g.count>1?` ×${g.count}`:''}${g.stale?' · last seen':''}`,'hazard');
      tag.position.set(g.at[0],g.at[1],g.kind==='fire'?5.4:g.kind==='fire + smoke'?7.6:2.2);this.part.hazardLabels.add(tag);
    }
  }

  // The crew's inferred route belief, drawn as a second storey above the trails.
  private drawMind(view:LiveView){
    const g=this.part.mind;clear(g);if(!view.edges.length)return;
    const box=new THREE.Box2();for(const p of Object.values(view.nodes))box.expandByPoint(new THREE.Vector2(...p));
    box.expandByScalar(2.6);const c=box.getCenter(new THREE.Vector2()),size=box.getSize(new THREE.Vector2());
    const floor=new THREE.Mesh(new THREE.PlaneGeometry(size.x,size.y),this.material('belief',PALETTE.belief,.1,{side:THREE.DoubleSide,emissive:PALETTE.belief,emissiveIntensity:.3}));
    floor.position.set(c.x,c.y,BELIEF_HEIGHT);floor.renderOrder=1;g.add(floor);
    const rim=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.PlaneGeometry(size.x,size.y)),new THREE.LineBasicMaterial({color:PALETTE.belief,transparent:true,opacity:.7}));
    rim.material.userData={base:.7,orig:.7,layer:'belief'};rim.position.copy(floor.position);g.add(rim);
    const title=label(`What the ${view.who} believes · inferred`,'plane-title');title.position.set(box.max.x-.3,box.max.y-.3,BELIEF_HEIGHT);title.center.set(1,0);g.add(title);
    // Faint pillars tie each trail junction to its copy in the belief map.
    const pillars=new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(Object.values(view.nodes).flatMap(p=>[new THREE.Vector3(p[0],p[1],.1),new THREE.Vector3(p[0],p[1],BELIEF_HEIGHT)])),
      new THREE.LineDashedMaterial({color:PALETTE.belief,dashSize:.25,gapSize:.3,transparent:true}));
    pillars.computeLineDistances();pillars.material.userData={base:.55,orig:.55,layer:'belief'};this.tint(pillars.material);g.add(pillars);
    for(const e of view.edges){
      const b=view.belief[e.id]??{look:'none' as BeliefLook,p:0};
      this.ribbon(g,e.a,e.b,BELIEF_HEIGHT+.03,b.look==='none'?.2:.38,LOOK_COLOR[b.look],'belief',b.look==='none'?.45:.85,b.look==='unknown');
    }
    for(const p of Object.values(view.nodes)){
      const dot=new THREE.Mesh(new THREE.CircleGeometry(.28,20),this.material('belief',PALETTE.belief,.9,{side:THREE.DoubleSide}));
      dot.position.set(p[0],p[1],BELIEF_HEIGHT+.05);g.add(dot);
    }
    this.layer.belief.traverse(o=>{const m=(o as THREE.Mesh).material;if(m&&!Array.isArray(m)&&m.userData.layer)this.tint(m);});
  }

  private drawPath(group:THREE.Group,path:string[],nodes:Record<string,Point>,kind:'intent'){
    clear(group);
    const points=path.flatMap(n=>nodes[n]?[nodes[n]]:[]);
    for(let i=1;i<points.length;i++)this.ribbon(group,points[i-1],points[i],.07,.3,PALETTE.intent,'belief',.95,true);
    group.name=kind;
  }

  private drawGap(view:LiveView){
    const g=this.part.gap;clear(g);const gap=view.gap,edge=gap&&view.edges.find(e=>e.id===gap.edge);
    if(!gap||!edge)return;
    const x=(edge.a[0]+edge.b[0])/2,y=(edge.a[1]+edge.b[1])/2,color=gap.kind==='gap'?PALETTE.blocked:PALETTE.clear;
    const stem=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(x,y,.1),new THREE.Vector3(x,y,BELIEF_HEIGHT)]),new THREE.LineDashedMaterial({color,dashSize:.35,gapSize:.2,linewidth:2}));
    stem.computeLineDistances();stem.material.userData={base:1,orig:1,layer:'belief'};this.tint(stem.material);g.add(stem);
    const tag=label(gap.text,gap.kind);tag.position.set(x,y,BELIEF_HEIGHT*.55);g.add(tag);
  }

  private drawRoute(view:LiveView){
    const g=this.part.route;clear(g);this.flow=[];this.routeCurve=null;
    const points=view.recommended.flatMap(n=>view.nodes[n]?[new THREE.Vector3(view.nodes[n][0],view.nodes[n][1],.32)]:[]);
    if(points.length<2)return;
    const curve=new THREE.CurvePath<THREE.Vector3>();
    for(let i=1;i<points.length;i++)curve.add(new THREE.LineCurve3(points[i-1],points[i]));
    const tube=new THREE.Mesh(new THREE.TubeGeometry(curve,points.length*24,.11,10,false),this.material('route',PALETTE.route,.95,{emissive:PALETTE.route,emissiveIntensity:.55}));
    g.add(tube);this.routeCurve=curve;
    const count=Math.max(3,Math.round(curve.getLength()/2.5));
    for(let i=0;i<count;i++){const dot=new THREE.Mesh(new THREE.SphereGeometry(.2,14,10),this.material('route','#ccfbf1',1,{emissive:'#5eead4',emissiveIntensity:1}));g.add(dot);this.flow.push(dot);}
  }

  // Smoke the sensors reported: it hides whatever lies beneath it from the UAV and robot.
  private drawSmoke(view:LiveView){
    const g=this.part.smoke;clear(g);
    for(const s of view.smoke){
      const template=this.hazardTemplates.smoke,cloud=new THREE.Group();
      if(template)cloud.add(this.build(template,'seen',false));
      else cloud.add(new THREE.Mesh(new THREE.SphereGeometry(s.radius*.7,24,16),this.material('seen','#8a8d90',.5,{depthWrite:false})));
      cloud.position.set(s.position[0],s.position[1],0);cloud.userData.drift=true;
      const tag=label('SMOKE · sensors can’t see beneath','smoke');tag.position.set(0,0,5.6);cloud.add(tag);
      g.add(cloud);
    }
  }

  private drawTruth(view:LiveView){
    const g=this.part.truth;clear(g);
    for(const zone of view.fireZones){
      if(zone.length<3)continue;
      const shape=new THREE.Shape(zone.map(p=>new THREE.Vector2(...p)));
      const fill=new THREE.Mesh(new THREE.ShapeGeometry(shape),this.material('truth',PALETTE.fire,.28,{side:THREE.DoubleSide,emissive:PALETTE.fire,emissiveIntensity:.5}));
      fill.position.z=.08;g.add(fill);
    }
  }
}
