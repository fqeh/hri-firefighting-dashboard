"""Prepare a native SDF instance from an explicit research scenario."""
import json,math,tempfile
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_world(share,case,seed=7,config_override=None,ground_robot=None):
    config_path=Path(config_override) if config_override else share/'config/forest_scenarios'/f'{case}.json'
    config=json.loads(config_path.read_text())
    config['seed']=int(seed)
    # Forest cases are UAV-only; tests that need the ground robot (e.g. the Phase 2
    # log-collision fixture) can put it back with ground_robot=True.
    if ground_robot is not None:config['ground_robot']=bool(ground_robot)
    directory=Path(tempfile.mkdtemp(prefix='hri_forest_'))
    config_path=directory/'scenario.json'
    config_path.write_text(json.dumps(config,indent=2)+'\n')
    tree=ET.parse(share/'worlds/forest.sdf');world=tree.getroot().find('world')
    for include in world.findall('include'):
        name=include.findtext('name')
        if name=='robot' and config.get('ground_robot',True) is False:
            world.remove(include);continue  # UAV-only observation scenarios
        if name in ('robot','human','uav'):
            include.find('pose').text=' '.join(map(str,config[name+'_start']))+' 0 0 0'
        if name=='uav':include.find('uri').text='model://uav_scout'
    model=ET.parse(share/'models/uav/model.sdf')
    for fov in model.getroot().findall('.//horizontal_fov'):fov.text=str(config['uav']['camera_hfov'])
    model_path=directory/'uav_scout';model_path.mkdir()
    model.write(model_path/'model.sdf')
    (model_path/'model.config').write_text('<model><name>uav_scout</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    goals=config['uav']['waypoints']
    for index,(a,b) in enumerate(zip(goals,goals[1:])):
        length=math.dist(a,b)
        yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        pitch=-math.atan2(b[2]-a[2],math.hypot(b[0]-a[0],b[1]-a[1]))
        xyz=[(x+y)/2 for x,y in zip(a,b)]
        xml=f'<model name="uav_planned_leg_{index}"><static>true</static><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 {pitch} {yaw}</pose><link name="link"><visual name="line"><geometry><box><size>{length} .035 .035</size></box></geometry><material><ambient>.1 .8 1 .7</ambient><diffuse>.1 .8 1 .7</diffuse></material></visual></link></model>'
        world.append(ET.fromstring(xml))
    # Understory deliberately stays outside route corridors.
    for index,(x,y) in enumerate([(-3,2),(2,11.2),(8,12),(14,-10.5),(20,5),(-4,-4)]):  # clear of Trails C/D
        world.append(ET.fromstring(f'<model name="understory_{index}"><static>true</static><pose>{x} {y} .25 0 0 0</pose><link name="link"><visual name="shrub"><geometry><ellipsoid><radii>.65 .4 .4</radii></ellipsoid></geometry><material><ambient>.18 .35 .12 1</ambient><diffuse>.18 .35 .12 1</diffuse></material></visual></link></model>'))
    tree.write(directory/'forest.sdf')
    return directory/'forest.sdf',config_path,directory
