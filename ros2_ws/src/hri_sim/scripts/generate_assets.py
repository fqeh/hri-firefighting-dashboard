#!/usr/bin/env python3
"""Generate native SDF assets; this script is not a dynamics/environment simulator."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def box(size): return f'<box><size>{size}</size></box>'
def cylinder(r,h): return f'<cylinder><radius>{r}</radius><length>{h}</length></cylinder>'
def sphere(r): return f'<sphere><radius>{r}</radius></sphere>'
def material(color): return f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
def visual(name,geo,color,pose='0 0 0 0 0 0'):
    return f'<visual name="{name}_visual"><pose>{pose}</pose><geometry>{geo}</geometry>{material(color)}</visual>'
def collision(name,geo,pose='0 0 0 0 0 0',mu=1):
    return f'<collision name="{name}_collision"><pose>{pose}</pose><geometry>{geo}</geometry><surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>'
def inertial(m=1,i=.1):
    return f'<inertial><mass>{m}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
def static_shape(name,pose,geo,color,solid=True):
    return f'<model name="{name}"><static>true</static><pose>{pose}</pose><link name="link">{visual("visual",geo,color)}{collision("collision",geo) if solid else ""}</link></model>'
def logical_sensor(name,far,pose='0 0 .4 0 0 0',fov=1.7):
    return f'''<sensor name="logical_camera" type="logical_camera"><pose>{pose}</pose><always_on>true</always_on><update_rate>10</update_rate><topic>/{name}/detections</topic><logical_camera><near>.05</near><far>{far}</far><horizontal_fov>{fov}</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor>'''
def odometry(name,frame='world'):
    return f'''<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"><odom_frame>{frame}</odom_frame><robot_base_frame>{name}/base_link</robot_base_frame><odom_topic>/{name}/odom</odom_topic><tf_topic>/{name}/pose_tf</tf_topic><odom_publish_frequency>20</odom_publish_frequency><dimensions>3</dimensions></plugin>'''
def wheeled(name,person=False):
    chassis=box('.8 .48 .22')
    body=visual('body',chassis,'.12 .32 .38 1')+collision('body',chassis)
    body+=logical_sensor(name,4 if person else 14,pose='.3 0 .8 0 0 0' if person else '.3 0 .2 0 0 0')
    if person:
        body+=visual('responder_torso',cylinder(.2,.65),'.95 .55 .08 1','0 0 .48 0 0 0')
        body+=collision('responder_torso',cylinder(.2,.65),'0 0 .48 0 0 0')
        body+=visual('helmet',sphere(.19),'1 .86 .12 1','0 0 .98 0 0 0')
        body+=collision('helmet',sphere(.19),'0 0 .98 0 0 0')
    else:
        body+=f'''<sensor name="lidar" type="gpu_lidar"><pose>.18 0 .25 0 0 0</pose><gz_frame_id>laser_frame</gz_frame_id><topic>/robot/scan</topic><update_rate>10</update_rate><always_on>true</always_on><lidar><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal></scan><range><min>.08</min><max>30</max><resolution>.01</resolution></range></lidar></sensor>'''
        body+='''<sensor name="imu" type="imu"><topic>/robot/imu</topic><gz_frame_id>base_link</gz_frame_id><update_rate>50</update_rate><always_on>true</always_on><imu/></sensor>'''
        body+='''<sensor name="camera" type="camera"><pose>.41 0 .17 0 0 0</pose><gz_frame_id>camera_optical_frame</gz_frame_id><topic>/robot/camera/image</topic><update_rate>5</update_rate><always_on>true</always_on><camera><horizontal_fov>1.4</horizontal_fov><image><width>320</width><height>240</height><format>R8G8B8</format></image><clip><near>.05</near><far>100</far></clip></camera></sensor>'''
    s=f'<model name="{name}"><pose>0 0 .02 0 0 0</pose><link name="base_link"><pose>0 0 .24 0 0 0</pose>{inertial(12,.5)}{body}</link>'
    for side,y in [('left',.32),('right',-.32)]:
        s+=f'<link name="{side}_wheel"><pose>.15 {y} .16 -1.57079632679 0 0</pose>{inertial(1,.013)}{visual("tire",cylinder(.16,.1),".07 .08 .09 1")}{collision("tire",cylinder(.16,.1))}</link>'
        s+=f'<joint name="{side}_joint" type="revolute"><parent>base_link</parent><child>{side}_wheel</child><axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>'
    s+=f'<link name="caster"><pose>-.29 0 .1 0 0 0</pose>{inertial(.2,.001)}{visual("caster",sphere(.1),".1 .1 .1 1")}{collision("caster",sphere(.1),mu=0)}</link><joint name="caster_joint" type="fixed"><parent>base_link</parent><child>caster</child></joint>'
    s+=f'''<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"><left_joint>left_joint</left_joint><right_joint>right_joint</right_joint><wheel_separation>.64</wheel_separation><wheel_radius>.16</wheel_radius><topic>/{name}/cmd_vel</topic><odom_topic>/{name}/{'wheel_odom' if person else 'odom'}</odom_topic><tf_topic>/{name}/tf</tf_topic><frame_id>{'human/odom' if person else 'odom'}</frame_id><child_frame_id>{'human/base_link' if person else 'base_footprint'}</child_frame_id><odom_publish_frequency>30</odom_publish_frequency><max_linear_velocity>1.2</max_linear_velocity><min_linear_velocity>-1.2</min_linear_velocity><max_angular_velocity>1.5</max_angular_velocity><min_angular_velocity>-1.5</min_angular_velocity><max_linear_acceleration>.8</max_linear_acceleration><min_linear_acceleration>-.8</min_linear_acceleration></plugin>'''
    if person: s+=odometry(name)
    else: s+=odometry(name).replace('/robot/odom', '/evaluation/robot_pose').replace('/robot/pose_tf','/evaluation/robot_pose_tf').replace('robot/base_link', 'robot/base_footprint')
    return s+'</model>'

def uav():
    body=visual('body',box('.6 .4 .16'),'.85 .87 .9 1')+collision('body',box('.6 .4 .16'))
    body+=visual('cross_a',box('1.4 .08 .06'),'.14 .18 .2 1')+visual('cross_b',box('.08 1.4 .06'),'.14 .18 .2 1')
    for i,(x,y) in enumerate([(.62,0),(-.62,0),(0,.62),(0,-.62)]):
        body+=visual(f'rotor_{i}',cylinder(.23,.015),'.2 .2 .25 .7',f'{x} {y} .1 0 0 0')
    body+=logical_sensor('uav',25,'0 0 -.12 0 1.57079632679 0',fov=2.3)
    body+='''<sensor name="camera" type="camera"><pose>0 0 -.15 0 1.57079632679 0</pose><topic>/uav/camera/image</topic><update_rate>3</update_rate><always_on>true</always_on><camera><horizontal_fov>1.9</horizontal_fov><image><width>320</width><height>240</height><format>R8G8B8</format></image><clip><near>.1</near><far>60</far></clip></camera></sensor>'''
    return f'<model name="uav"><static>true</static><pose>6 0 8 0 0 0</pose><link name="base_link">{body}</link>{odometry("uav")}</model>'

def write_xml(path,xml):
    root=ET.fromstring(xml)
    ET.indent(root, space='  ')
    path.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)

def model_asset(name,content):
    directory=ROOT/'models'/name
    write_xml(directory/'model.sdf',f'<sdf version="1.9">{content}</sdf>')
    write_xml(directory/'model.config',f'<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>HRI project contributors</name></author><description>Native offline Gazebo asset for belief-aware warnings.</description></model>')

NODES={'start':[0,-2],'fork':[4,-2],'trail_a':[10,-2],'trail_b':[4,6],'merge':[10,6],'safety':[16,6]}
EDGES=[('approach','start','fork'),('trail_a','fork','trail_a'),('exit_a','trail_a','safety'),('trail_b','fork','trail_b'),('bypass','trail_b','merge'),('exit_b','merge','safety')]
# Forest only: four independent escape routes to the safety zone. A and B are short;
# C runs along the upwind (south) side and D along the north ridge from the start.
# The prevailing wind blows north, so C is the long but safest detour.
FOREST_NODES={**NODES,'trail_c':[4,-8],'c_end':[18,-8],'trail_d':[-1,8.5],'d_end':[16,8.5]}
FOREST_EDGES=EDGES+[('trail_c','fork','trail_c'),('trail_c_east','trail_c','c_end'),('exit_c','c_end','safety'),
                    ('trail_d','start','trail_d'),('trail_d_east','trail_d','d_end'),('exit_d','d_end','safety')]
def route_map(name):return (FOREST_NODES,FOREST_EDGES) if name=='forest' else (NODES,EDGES)

def terrain_mesh():
    # Closed slopes as a genuine triangulated 3D collision mesh, surrounding flat trails.
    p=ROOT/'models'/'terrain'/'meshes'/'hills.obj'; p.parent.mkdir(parents=True,exist_ok=True)
    verts=[]; faces=[]
    # The southern hill sits below the Trail D corridor (y = -8).
    for cx,cy,rx,ry,height in [(-7,6,5,9,2.5),(13,-15,10,4,3.8),(9,15,12,5,2.5),(25,3,5,12,3.2)]:
        base=len(verts)
        verts.extend([(cx-rx,cy-ry,-.05),(cx+rx,cy-ry,-.05),(cx+rx,cy+ry,-.05),(cx-rx,cy+ry,-.05),(cx,cy,height)])
        faces.extend([(base+1,base+2,base+5),(base+2,base+3,base+5),(base+3,base+4,base+5),(base+4,base+1,base+5),(base+1,base+4,base+3),(base+1,base+3,base+2)])
    # Gazebo's DART/ODE mesh importer requires a normal for every expanded
    # vertex. Explicit per-face vertices also preserve sharp pyramid ridges.
    expanded=[]; normals=[]
    for face in faces:
        a,b,c=(verts[index-1] for index in face)
        u=[b[i]-a[i] for i in range(3)]
        v=[c[i]-a[i] for i in range(3)]
        cross=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
        length=math.sqrt(sum(value*value for value in cross))
        normal=tuple(value/length for value in cross)
        expanded.extend((a,b,c)); normals.extend((normal,normal,normal))
    content='# Authored HRI terrain; meters, Z up. Apache-2.0\n'
    content+=''.join('v %.9g %.9g %.9g\n'%v for v in expanded)
    content+=''.join('vn %.9g %.9g %.9g\n'%n for n in normals)
    content+=''.join(f'f {i}//{i} {i+1}//{i+1} {i+2}//{i+2}\n' for i in range(1,len(expanded)+1,3))
    p.write_text(content)
    mesh='<mesh><uri>model://terrain/meshes/hills.obj</uri></mesh>'
    model_asset('terrain',static_shape('terrain','0 0 0 0 0 0',mesh,'.27 .35 .17 1'))

def world(name):
    plugins=''.join(f'<plugin filename="gz-sim-{file}-system" name="gz::sim::systems::{cls}"/>' for file,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('imu','Imu'),('logical-camera','LogicalCamera')])
    plugins+='<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'
    s=f'<sdf version="1.9"><world name="{name}"><physics name="deterministic_1ms" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.81</gravity>{plugins}<scene><ambient>.5 .5 .5 1</ambient><background>.62 .75 .83 1</background><shadows>true</shadows></scene><light name="sun" type="directional"><pose>0 0 20 0 0 0</pose><diffuse>.9 .9 .85 1</diffuse><specular>.2 .2 .2 1</specular><direction>-.4 .2 -.9</direction><cast_shadows>true</cast_shadows></light>'
    s+=static_shape('ground','8 0 -.1 0 0 0',box('80 80 .2'),'.35 .4 .26 1' if name=='forest' else '.58 .52 .4 1')
    s+='<include><uri>model://robot</uri><name>robot</name><pose>0 0 .02 0 0 0</pose></include>'
    if name=='minimal':
        s+=static_shape('sensor_target','5 0 1 0 0 0',box('1 2 2'),'.82 .25 .08 1')
        s+=static_shape('side_wall','0 5 1 0 0 0',box('8 .3 2'),'.6 .65 .7 1')
        return s+'</world></sdf>'
    s+='<include><uri>model://human</uri><name>human</name><pose>0 -2 .02 0 0 0</pose></include><include><uri>model://uav</uri><name>uav</name><pose>6 0 8 0 0 0</pose></include>'
    nodes,edges=route_map(name)
    for edge,a,b in edges:
        x1,y1=nodes[a]; x2,y2=nodes[b]; length=math.hypot(x2-x1,y2-y1); yaw=math.atan2(y2-y1,x2-x1)
        s+=static_shape('route_'+edge,f'{(x1+x2)/2} {(y1+y2)/2} .006 0 0 {yaw}',box(f'{length} 2.2 .01'),'.52 .44 .31 1' if name=='forest' else '.24 .26 .28 1',solid=False)
    s+=static_shape('safety_zone','16 6 .018 0 0 0',box('3 3 .02'),'.15 .62 .4 1',solid=False)
    s+=static_shape('safety_marker','17.5 7.5 1 0 0 0',box('.15 .15 2'),'.1 .8 .35 1')
    if name=='forest':
        s+='<include><uri>model://terrain</uri></include>'
        # Moved off the Trail C/D corridors: (2,-6), (14.2,-3.5), (-2.6,4), (1.8,4.5), (-0.5,11.2), (1.6,2.2).
        trees=[(-3,-4),(-3,0),(-2.6,4),(1.8,4.5),(-0.5,11.2),(5,10),(8,10),(12,10),(16,10),(19,8),(19,4),(14.2,-3.5),(12,-5),(8,-5),(2,-6),(0,-5),(7,1),(7,3),(1.6,2.2),(12,2),(21,-3)]
        for i,(x,y) in enumerate(trees):
            h=3.5+(i%4)*.35
            trunk=collision('trunk',cylinder(.22,h),f'0 0 {h/2} 0 0 0')+visual('trunk',cylinder(.22,h),'.28 .13 .06 1',f'0 0 {h/2} 0 0 0')
            crown=visual('crown',sphere(1.15),'.1 .3 .1 1',f'0 0 {h} 0 0 0')+collision('crown',sphere(1.15),f'0 0 {h} 0 0 0')
            s+=f'<model name="tree_{i}"><static>true</static><pose>{x} {y} 0 0 0 0</pose><link name="tree">{trunk}{crown}</link></model>'
    else:
        for i,(x,y) in enumerate([(-5,3),(-5,9),(5,-8),(12,-8)]):
            s+=static_shape(f'oil_tank_{i}',f'{x} {y} 2.5 0 0 0',cylinder(2,5),'.7 .73 .74 1')
            s+=static_shape(f'tank_roof_{i}',f'{x} {y} 5.05 0 0 0',cylinder(2.05,.1),'.85 .85 .81 1')
        for i,(x,y,sx,sy) in enumerate([(9,11,5,4),(20,-2,5,6),(-5,-6,5,3)]):
            s+=static_shape(f'building_{i}',f'{x} {y} 2 0 0 0',box(f'{sx} {sy} 4'),'.73 .63 .45 1')
        for i,(x,y,length,yaw) in enumerate([(8,-5.5,13,0),(-2,7,7,math.pi/2)]):
            s+=static_shape(f'pipeline_{i}',f'{x} {y} .7 0 {math.pi/2} {yaw}',cylinder(.22,length),'.35 .4 .44 1')
        for i,y in enumerate([4.3,7.7]): s+=static_shape(f'gate_post_{i}',f'14 {y} 1.5 0 0 0',box('.25 .25 3'),'.8 .55 .08 1')
        s+=static_shape('gate_crossbar','14 6 3 0 0 0',box('.2 3.7 .2'),'.8 .55 .08 1')
        for i,x in enumerate([0,4,8,12,16]): s+=static_shape(f'perimeter_{i}',f'{x} -12 1 0 0 0',box('3.8 .2 2'),'.5 .52 .5 1')
    return s+'</world></sdf>'

# Dense smoke: a visual-only cloud. Perception treats it as an occluder of what lies
# beneath it (see hri_reasoning.tom.SMOKE_RADIUS), not as a hazard with a route state.
SMOKE_PUFFS=[(0,0,1.4,2.3),(1.7,.6,2.0,1.9),(-1.6,-.5,1.8,2.0),(.4,-1.5,2.4,1.8),(-.6,1.6,2.8,1.7),(0,0,3.4,1.9)]
def smoke_asset():
    smoke=''.join(visual(f'puff_{i}',sphere(r),'.55 .56 .57 .55',f'{x} {y} {z} 0 0 0') for i,(x,y,z,r) in enumerate(SMOKE_PUFFS))
    model_asset('hazard_smoke',f'<model name="hazard_smoke"><static>true</static><link name="link">{smoke}</link></model>')

# A fully developed fire with its smoke column (drifting north with the wind): an
# impassable incident, unlike the passable hazard_fire patches of the fire model.
def fire_smoke_asset():
    parts=visual('flame_core',cylinder(.9,2.0),'1 .3 .02 .9','0 0 1.0 0 0 0')+visual('flame_side',cylinder(.55,1.4),'1 .45 .05 .9','.7 .35 .7 0 0 0')
    parts+=visual('flame_tip',sphere(.75),'1 .68 .05 .85','0 0 2.1 0 0 0')
    for i,(x,y,z,r) in enumerate([(0,.3,3.2,1.1),(.3,.8,4.2,1.4),(.6,1.4,5.3,1.7),(1.0,2.2,6.4,2.0)]):
        parts+=visual(f'smoke_{i}',sphere(r),'.18 .18 .2 .55',f'{x} {y} {z} 0 0 0')
    model_asset('hazard_fire_smoke',f'<model name="hazard_fire_smoke"><static>true</static><link name="link">{parts}</link></model>')

def main():
    model_asset('robot',wheeled('robot'))
    model_asset('human',wheeled('human',True))
    model_asset('uav',uav())
    terrain_mesh()
    model_asset('hazard_tree',static_shape('hazard_tree','0 0 .45 0 0 0',cylinder(.45,3.4),'.32 .16 .05 1').replace('<link name="link">','<link name="link"><pose>0 0 0 1.57079632679 0 0</pose>'))
    model_asset('hazard_debris',static_shape('hazard_debris','0 0 .6 0 0 0',box('1.2 3.2 1.2'),'.3 .32 .34 1'))
    fire=visual('flame_core',cylinder(.7,1.8),'1 .25 .02 .85','0 0 .9 0 0 0')+visual('flame_tip',sphere(.5),'1 .65 .02 .8','0 0 1.8 0 0 0')
    for i in range(3): fire+=visual(f'smoke_{i}',sphere(.7+i*.25),'.22 .23 .25 .25',f'{i*.15} 0 {2.7+i*.8} 0 0 0')
    # Hazard volumes are intentionally non-solid: exposure is measured, not collision avoidance.
    model_asset('hazard_fire',f'<model name="hazard_fire"><static>true</static><link name="link">{fire}</link></model>')
    gas=visual('gas_volume',sphere(1.8),'.5 .65 .16 .3','0 0 1 0 0 0')
    model_asset('hazard_gas',f'<model name="hazard_gas"><static>true</static><link name="link">{gas}</link></model>')
    smoke_asset()
    fire_smoke_asset()
    for name in ['minimal','forest','industrial']:
        write_xml(ROOT/'worlds'/f'{name}.sdf',world(name))
    for name in ['forest','industrial']:
        NODES_,EDGES_=route_map(name)
        config={'scenario':name,'world':name,'frame':'world','nodes':NODES_,'edges':[{'id':eid,'source':a,'target':b,'length':math.dist(NODES_[a],NODES_[b]),'initial_state':'unknown'} for eid,a,b in EDGES_],'start':'start','goal':'safety','robot_start':[0,0,0.02],'human_start':[0,-2,0.02],'uav_start':[6,0,8],'hazards':[{'id':'hazard_tree' if name=='forest' else 'hazard_debris','model':'hazard_tree' if name=='forest' else 'hazard_debris','edge':'trail_a','position':[7,-2,.45 if name=='forest' else .6],'at':3.0,'kind':'tree' if name=='forest' else 'debris','radius':2.0},{'id':'hazard_fire','model':'hazard_fire','edge':'exit_a','position':[12,0.7,0],'at':24.0,'kind':'fire','radius':1.6},{'id':'hazard_fire_spread' if name=='forest' else 'hazard_gas','model':'hazard_fire' if name=='forest' else 'hazard_gas','edge':'exit_a','position':[13.2,2.2,0],'at':38.0,'kind':'fire' if name=='forest' else 'gas','radius':1.8}]}
        (ROOT/'config'/f'{name}.json').write_text(json.dumps(config,indent=2)+'\n')

if __name__=='__main__': main()
