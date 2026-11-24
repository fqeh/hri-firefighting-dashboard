import {afterEach,beforeEach,describe,it,expect,vi} from 'vitest';
import {render,screen,cleanup,fireEvent,act} from '@testing-library/react';
import {liveView,beliefLook,WorldView3D,WorldView,LayerList,useLayers,DEFAULT_LAYERS} from './scene3d';
import {parseScene,type Dict,type Topic} from './contracts';
import {isVegetation} from './world3d';
afterEach(cleanup);
// jsdom has no WebGL: the view must degrade to a message, never crash.
beforeEach(()=>{vi.spyOn(HTMLCanvasElement.prototype,'getContext').mockReturnValue(null);});

const topic=(data:Dict,source:Topic['source']='observation'):Topic=>({data,received_at:100,age_seconds:.1,stale:false,source,sim_stamp:10});
const params={nodes:{start:[0,0],fork:[4,0],trail_a:[10,0],trail_b:[4,6]},edges:[{id:'approach',source:'start',target:'fork'},{id:'trail_a',source:'fork',target:'trail_a'},{id:'trail_b',source:'fork',target:'trail_b'}],robot_start:[1,2,0]};
const topics=(path:string[]):Record<string,Topic>=>({
  '/hri/rescue_route':topic({nodes:params.nodes,edge_states:{approach:'clear',trail_a:'blocked',trail_b:'unknown'},recommended:['start','fork','trail_b']},'inference'),
  '/hri/belief':topic({edge:'trail_a',beliefs:{trail_a:{posterior:{open_safe:.8,blocked:.1,open_hazardous:.05,unknown_stale:.05}},trail_b:{posterior:{open_safe:.1,blocked:.1,open_hazardous:.1,unknown_stale:.7}}}},'inference'),
  '/hri/human_intent':topic({path}),
  '/hri/observations':topic({hazards:[{id:'tree',kind:'tree',edge:'trail_a',position:[7,0,.45],radius:2,observation_stale:false}]}),
  '/human/odom':topic({position:[2,0,0],yaw:.5},'simulation_truth'),
  '/robot/odom':topic({position:[.5,0,0],yaw:0}),
});
const decision=(approaching:boolean):Dict=>({hazard_id:'tree',edge:'trail_a',action:approaching?'EXPLICIT_WARNING':'SILENT',
  context:{probability:.8,approaching,task_relevant:true,distance:5,urgency:0},threshold:approaching?{probability:.39,action:'SOFT_CUE'}:null});
const scene={schema_version:1,scenario:'forest',world:'forest',background:[.6,.7,.8,1],source:'test',sensors:{uav:{horizontal_fov:1.2,far:25,pitch:1.57}},
  scenery:[{shape:'box',size:[1,1,1],name:'a',position:[0,0,0],quaternion:[0,0,0,1],color:[1,0,0,1]},
    {shape:'mesh',vertices:[0,0,0,1,0,0,0,1,0],indices:[0,1,2],name:'m',position:[0,0,0],quaternion:[0,0,0,1],color:[0,1,0,1]}],
  entities:{human:[{shape:'sphere',radius:.2,name:'h',position:[0,0,1],quaternion:[0,0,0,1],color:[1,1,0,1]}]},hazards:{}};

describe('web 3D world view',()=>{
  it('projects telemetry into world-frame agents, observed hazards and the crew’s belief map',()=>{
    const view=liveView(topics(['start','fork','trail_a']),params,decision(true),false);
    expect(view.entities.find(e=>e.key==='robot')?.position).toEqual([1.5,2,0]);  // odometry + robot start
    expect(view.entities.find(e=>e.key==='human')?.yaw).toBe(.5);
    expect(view.hazards[0]).toMatchObject({kind:'tree',position:[7,0,.45],radius:2});
    expect(view.belief.trail_a).toEqual({look:'open',p:.8});
    expect(view.belief.trail_b.look).toBe('unknown');
    expect(view.gap).toEqual({edge:'trail_a',kind:'gap',text:'≠ crew thinks it is open · 80%'});
    expect(view.recommended).toEqual(['start','fork','trail_b']);
    expect(view.smoke).toEqual([]);
    const smoky=liveView({...topics(['start']),'/hri/observations':topic({hazards:[],occluders:[{id:'hazard_smoke',kind:'smoke',edge:'trail_a',position:[7,-2,0],radius:3.5}]})},params,{},false);
    expect(smoky.smoke).toEqual([{id:'hazard_smoke',edge:'trail_a',position:[7,-2,0],radius:3.5}]);
    const avoiding=liveView(topics(['start','fork','trail_b']),params,decision(false),false);
    expect(avoiding.gap).toMatchObject({kind:'agree',text:'= crew’s route avoids it'});
  });
  it('separates forest vegetation from the rest of the Gazebo world',()=>{
    expect(['tree_0/link/trunk','understory_3/link/shrub'].every(isVegetation)).toBe(true);
    expect(['ground/link/visual_visual','route_trail_a/link/visual_visual','tank_0/body/surface','street_tree/link/x'].some(isVegetation)).toBe(false);
  });
  it('picks the most likely hypothesis for each trail',()=>{
    expect(beliefLook({open_safe:.1,blocked:.7,open_hazardous:.1,unknown_stale:.1})).toEqual({look:'blocked',p:.7});
    expect(beliefLook({})).toEqual({look:'none',p:0});
  });
  it('validates world geometry from the supervisor',()=>{
    expect(parseScene(scene).scenery).toHaveLength(2);
    expect(()=>parseScene({...scene,scenery:[{...scene.scenery[1],indices:[0,1,9]}]})).toThrow(/geometry/);
    expect(()=>parseScene({...scene,scenery:[{...scene.scenery[0],size:[1,-1,1]}]})).toThrow(/geometry/);
    expect(parseScene({...scene,spawns:{human:{position:[0,-2,.02],yaw:0}}}).spawns?.human.position).toEqual([0,-2,.02]);
    expect(()=>parseScene({...scene,spawns:{human:{position:[0,'x',0],yaw:0}}})).toThrow(/spawn/);
  });
  it('degrades to a message without WebGL and keeps the GIS-style layer list usable',()=>{
    const view=liveView(topics(['start']),params,decision(false),false);
    render(<WorldView3D scene={parseScene(scene)} sceneError="" view={view} oil={false} layers={DEFAULT_LAYERS} status="Sim 1.0 s"/>);
    expect(screen.getByText(/needs WebGL/)).toBeTruthy();
    cleanup();
    const Layers=({oil}:{oil:boolean})=>{const [layers,set]=useLayers();return <LayerList layers={layers} onChange={set} oil={oil}/>;};
    render(<Layers oil={false}/>);
    const eye=screen.getByRole('button',{name:'Hide What the crew believes'});
    expect(eye.getAttribute('aria-pressed')).toBe('true');
    fireEvent.click(eye);
    expect(screen.getByRole('button',{name:'Show What the crew believes'}).getAttribute('aria-pressed')).toBe('false');
    expect(screen.getByRole('button',{name:'Show Fire model (truth)'})).toBeTruthy();  // evaluation truth is opt-in
    fireEvent.click(screen.getByRole('button',{name:'Hide Trees'}));
    expect(screen.getByRole('button',{name:'Show Trees'}).getAttribute('aria-pressed')).toBe('false');
    fireEvent.change(screen.getByLabelText('Recommended route opacity'),{target:{value:'0.4'}});
    expect(screen.getByText('0.40')).toBeTruthy();
    cleanup();
    render(<Layers oil={true}/>);
    expect(screen.queryByText('Fire model (truth)')).toBeNull();
    expect(screen.queryByText('Trees')).toBeNull();  // the oil facility has no vegetation
    expect(screen.getByText('What the robot sees')).toBeTruthy();
  });
  it('pauses Gazebo’s window once per run in web mode and resumes it for the Gazebo tab',()=>{
    vi.useFakeTimers();
    try{
      const onFreeze=vi.fn(),quietFreeze=vi.fn().mockResolvedValue(true),onMode=vi.fn(),view=liveView({},params,{},false);
      const props=(frozen:boolean,mode:'web'|'gazebo'='web')=>({mode,onMode,scene:null,sceneError:'',view,oil:false,clock:null,layers:DEFAULT_LAYERS,gazebo:{session:'run-1',ready:true,live:true,frozen,disabled:false,onFreeze,quietFreeze}});
      const panel=render(<WorldView {...props(false)}/>);
      act(()=>{vi.advanceTimersByTime(3100);});
      expect(quietFreeze).toHaveBeenCalledWith(true);
      panel.rerender(<WorldView {...props(true)}/>);
      fireEvent.click(screen.getByRole('tab',{name:'Gazebo window'}));
      expect(onMode).toHaveBeenCalledWith('gazebo');expect(quietFreeze).toHaveBeenLastCalledWith(false);
      panel.rerender(<WorldView {...props(false)}/>);
      act(()=>{vi.advanceTimersByTime(5000);});
      expect(quietFreeze).toHaveBeenCalledTimes(2);  // no second automatic pause in the same run
      expect(onFreeze).not.toHaveBeenCalled();  // implicit toggles never post command feedback
    }finally{vi.useRealTimers();}
  });
});
