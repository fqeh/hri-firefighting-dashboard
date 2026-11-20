"""Native SDF construction from a fictional public facility layout, not a simulator."""
import copy,json,math,tempfile
from pathlib import Path
import xml.etree.ElementTree as ET


def shape(world,name,xyz,size,color,kind='box',solid=True,yaw=0):
    model=ET.SubElement(world,'model',name=name)
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,[*xyz,0,0,yaw]))
    link=ET.SubElement(model,'link',name='body')
    geometry=ET.Element('geometry');primitive=ET.SubElement(geometry,kind)
    if kind=='box':ET.SubElement(primitive,'size').text=' '.join(map(str,size))
    elif kind=='cylinder':
        ET.SubElement(primitive,'radius').text=str(size[0]);ET.SubElement(primitive,'length').text=str(size[1])
    elif kind=='sphere':ET.SubElement(primitive,'radius').text=str(size[0])
    visual=ET.SubElement(link,'visual',name='surface');visual.append(copy.deepcopy(geometry))
    material=ET.SubElement(visual,'material')
    for key in ('ambient','diffuse'):ET.SubElement(material,key).text=color
    if solid:ET.SubElement(link,'collision',name='solid').append(copy.deepcopy(geometry))
    return model


def build_world(sim_share,layout,config,directory):
    tree=ET.parse(sim_share/'worlds/industrial.sdf');world=tree.getroot().find('world')
    world.set('name','oil_facility')
    for child in list(world):
        if child.tag in ('model','include'):world.remove(child)
    world.find('scene/background').text='.78 .72 .60 1'
    shape(world,'desert_ground',[8,0,-.1],[75,65,.2],'.70 .60 .43 1')
    for edge in layout['edges']:
        a,b=[layout['nodes'][edge[k]] for k in ('source','target')]
        length=math.dist(a,b);yaw=math.atan2(b[1]-a[1],b[0]-a[0]);center=[(a[0]+b[0])/2,(a[1]+b[1])/2]
        width=layout['road_widths'][edge['id']]
        shape(world,'route_'+edge['id'],[*center,.008],[length,width,.015],'.18 .20 .22 1',solid=False,yaw=yaw)
        # Dashed lane markings and shoulders provide scale in the native viewer.
        for i in range(max(1,int(length/2))):
            t=(i+.5)/max(1,int(length/2));pos=[a[j]+(b[j]-a[j])*t for j in (0,1)]
            shape(world,f'lane_{edge["id"]}_{i}',[*pos,.019],[.7,.07,.01],'.90 .82 .54 1',solid=False,yaw=yaw)
    for name,pos in layout['nodes'].items():
        shape(world,'junction_pad_'+name,[*pos,.01],[1.85,.02],'.18 .20 .22 1','cylinder',False)
    for name,pos in [('assembly_east',[15,6]),('assembly_north',[3,6])]:
        shape(world,name,[*pos,.026],[3.3,3.3,.025],'.08 .53 .32 1',solid=False)
        shape(world,name+'_sign',[pos[0]-1.6,pos[1]+1.8,1.5],[.12,.12,3.],'.75 .75 .70 1')
        shape(world,name+'_panel',[pos[0]-1.6,pos[1]+1.8,2.8],[1.5,.08,.8],'.08 .65 .35 1')
        # White cross/assembly symbol, generated geometry rather than an external texture.
        shape(world,name+'_mark',[pos[0]-1.6,pos[1]+1.74,2.8],[.85,.02,.13],'.96 .96 .90 1',solid=False)
    for i,(x,y,r,h) in enumerate([(6,3.6,1.35,3.2),(9,3.6,1.2,3.6),(18,3,1.8,4),(19,-1.5,1.5,3.2)]):
        shape(world,f'tank_{i}',[x,y,h/2],[r,h],'.78 .78 .72 1','cylinder')
        shape(world,f'tank_{i}_roof',[x,y,h+.05],[r+.12,.13],'.91 .90 .83 1','cylinder')
        shape(world,f'tank_{i}_base',[x,y,.13],[r+.25,.26],'.43 .43 .39 1','cylinder')
        for z in (.8,2.1):shape(world,f'tank_{i}_band_{z}',[x,y,z],[r+.015,.08],'.65 .48 .22 1','cylinder',False)
    for i,y in enumerate((2.7,3.1,3.5)):
        pipe=shape(world,f'pipeline_{i}',[0,y,.95],[.13,7.5],'.65 .45 .16 1','cylinder')
        pipe.find('pose').text=f'0 {y} .95 0 1.57079632679 0'
    for x in (-2,0,1.4):
        shape(world,f'pipe_support_{x}',[x,3.1,.45],[.2,1.3,.9],'.30 .32 .34 1')
    for i,x in enumerate((6,8,10)):
        shape(world,f'process_column_{i}',[x,-2.8,2.4],[.42,4.8],'.55 .58 .59 1','cylinder')
        shape(world,f'process_valve_{i}',[x,-2.3,1.],[.18,.18,.18],'.82 .16 .10 1')
    shape(world,'process_gantry',[8,-2.8,3.1],[5.3,1.2,.2],'.38 .42 .43 1')
    shape(world,'control_building',[-3,-4.1,1.55],[3.5,3.2,3.1],'.76 .64 .46 1')
    shape(world,'control_roof',[-3,-4.1,3.18],[3.8,3.5,.16],'.85 .78 .61 1')
    shape(world,'control_windows',[-1.23,-4.1,1.9],[.03,2.3,.65],'.17 .35 .42 1',solid=False)
    shape(world,'utility_building',[8,9.5,1.4],[5.0,2.4,2.8],'.70 .60 .44 1')
    for i,x in enumerate((5.5,7.,8.5)):
        shape(world,f'utility_transformer_{i}',[x,8.1,.6],[.8,.65,1.2],'.39 .44 .43 1')
    # The narrow corridor walls are physical obstacles, not route annotations.
    for y in (-1.5,1.5):
        shape(world,f'corridor_b_barrier_{y}',[6,y,.45],[4.8,.2,.9],'.76 .65 .24 1')
    for i,x in enumerate((-1.9,1.9)):
        shape(world,f'entry_gate_post_{i}',[x,-1.8,.7],[.18,.18,1.4],'.77 .74 .64 1')
    shape(world,'open_gate_arm',[-1.9,-1.8,1.7],[.14,.14,2.2],'.92 .84 .30 1')
    # Fenced restricted process yard; gates/open roads remain traversable.
    for i,(x,y,dx,dy) in enumerate([(21,2,.12,15),(-5.5,2,.12,17),(8,11.5,27,.12),(8,-8,27,.12)]):
        shape(world,f'perimeter_fence_{i}',[x,y,1.],[dx,dy,2.],'.43 .46 .44 .8')
    shape(world,'restricted_yard_marker',[18,-5,.03],[3,3,.04],'.66 .32 .16 1',solid=False)
    for i,(x,y) in enumerate([(-1,-3.8),(16.8,7.5),(5,8.5)]):
        shape(world,f'personnel_{i}',[x,y,.9],[.23,1.3],'.94 .49 .10 1','cylinder')
        shape(world,f'personnel_helmet_{i}',[x,y,1.72],[.22],'.98 .87 .22 1','sphere')
    for name,model in [('robot','oil_robot')]:
        include=ET.SubElement(world,'include');ET.SubElement(include,'uri').text='model://'+model
        ET.SubElement(include,'name').text=name
        ET.SubElement(include,'pose').text=' '.join(map(str,config[name+'_start']))+' 0 0 0'
    robot=ET.parse(sim_share/'models/robot/model.sdf')
    robot.getroot().find('.//logical_camera/horizontal_fov').text='3.0'
    resource=directory/'oil_robot';resource.mkdir()
    robot.write(resource/'model.sdf')
    (resource/'model.config').write_text('<model><name>oil_robot</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    ET.indent(tree);tree.write(directory/'oil_facility.sdf',encoding='unicode')
    return directory/'oil_facility.sdf'


def prepare_world(sim_share,industrial_share,case,seed,override=''):
    layout=json.loads((industrial_share/'config/layout.json').read_text())
    path=Path(override) if override else industrial_share/f'config/scenarios/{case}.json'
    config=json.loads(path.read_text());config['seed']=int(seed)
    if config['world']!='oil_facility':raise ValueError('Expected an oil-facility scenario')
    from hri_reasoning.tom import public_map
    if public_map(config)!=public_map(layout):raise ValueError('Scenario graph must match the public physical facility layout')
    layout['robot_origin_xy']=config['robot_start'][:2]
    # Public detector calibration bounds, with no private incident IDs/positions.
    for kind in layout['hazard_radii']:
        radii=[h['radius'] for h in config['hazards'] if h['kind']==kind]
        if radii:
            if not all(math.isfinite(r) and r>0 for r in radii):raise ValueError('Hazard radius must be finite and positive')
            layout['hazard_radii'][kind]=max(radii)
    directory=Path(tempfile.mkdtemp(prefix='hri_oil_'))
    (directory/'scenario.json').write_text(json.dumps(config,indent=2)+'\n')
    if config.get('initial_survey_edges') is not None:layout['initial_survey_edges']=config['initial_survey_edges']
    (directory/'layout.json').write_text(json.dumps(layout,indent=2)+'\n')
    return build_world(sim_share,layout,config,directory),directory/'scenario.json',directory
