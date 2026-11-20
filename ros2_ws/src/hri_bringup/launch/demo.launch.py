"""Physical simulation; no alternative simulator or fabricated telemetry."""
import os
import json
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    values = context.launch_configurations
    scenario = values['scenario']
    if scenario not in ('minimal', 'forest', 'industrial', 'oil_facility'):
        raise ValueError('scenario must be minimal, forest, industrial, or oil_facility')
    sim = Path(get_package_share_directory('hri_sim'))
    case=values['forest_case']
    world_file=sim/'worlds'/f'{scenario}.sdf'
    config_file=sim/'config'/f'{scenario}.json'
    extra_resources=''
    oil=scenario=='oil_facility'
    nav2=values['nav2'].lower()=='true' or (values['nav2']=='auto' and oil)
    if oil:
        from hri_industrial.world import prepare_world
        industrial=Path(get_package_share_directory('hri_industrial'))
        world_file,config_file,resources=prepare_world(sim,industrial,values['oil_case'],int(values['seed']),values['scenario_config'])
        extra_resources=str(resources)+':'
    if case:
        if scenario!='forest' or case not in 'ABCDEF' or len(case)!=1:
            raise ValueError('forest_case must be A–F and scenario must be forest')
        from hri_bringup.forest_world import prepare_world
        world_file,config_file,resources=prepare_world(sim,case,int(values['seed']),values['scenario_config'] or None)
        extra_resources=str(resources)+':'
    gui = values['gui'].lower() == 'true'
    mission = values['mission'].lower() == 'true'
    if mission and nav2:
        raise ValueError('mission and nav2 cannot both command the robot')
    command = ['gz', 'sim', '--seed', values['seed']]
    if values['start_paused'].lower() != 'true':
        command.append('-r')
    if not gui:
        command += ['-s', '--headless-rendering']
    command += [str(world_file)]
    supervisor=str(Path(get_package_prefix('hri_bringup'))/'lib/hri_bringup/gazebo_process')
    gazebo = ExecuteProcess(cmd=[supervisor,*command], output='screen', additional_env={
        'GZ_SIM_RESOURCE_PATH': extra_resources + str(sim / 'models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')})
    actions = [gazebo, RegisterEventHandler(OnProcessExit(target_action=gazebo,
        on_exit=[EmitEvent(event=Shutdown(reason='Gazebo process exited'))])),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge',
             parameters=[{'config_file': str(sim / 'config' / 'bridge.yaml'), 'use_sim_time': True}], output='screen'),
        Node(package='hri_bringup', executable='safety_controller',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_transform',
             arguments=['--z', '0.24', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='lidar_transform',
             arguments=['--x', '0.18', '--y', '0', '--z', '0.25',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_transform',
             arguments=['--x', '0.41', '--y', '0', '--z', '0.17',
                        '--roll', '-1.57079632679', '--pitch', '0', '--yaw', '-1.57079632679',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame'],
             parameters=[{'use_sim_time': True}])]
    if mission:
        actions.append(Node(package='hri_bringup', executable='waypoint_mission',
                            parameters=[{'use_sim_time': True,
                                         'waypoints_json': ParameterValue(values['waypoints_json'], value_type=str),
                                         'trajectory_path': values['trajectory_path']}], output='screen'))
    if values['dashboard'].lower() == 'true':
        actions.append(Node(package='hri_dashboard', executable='dashboard_server',
                            parameters=[{'use_sim_time': True}], output='screen'))
    if scenario != 'minimal' and not case and not oil:
        actions.append(Node(package='hri_bringup', executable='mission_monitor',
                            parameters=[{'use_sim_time': True}], output='screen'))
    if oil and values['reasoning'].lower()=='true':
        from hri_industrial.launching import reasoning_actions
        actions+=reasoning_actions(values,config_file,resources/'layout.json')
    if scenario != 'minimal' and not oil and values['reasoning'].lower() == 'true':
        params = {'use_sim_time': True, 'scenario': scenario, 'config': str(config_file),
                  'seed': int(values['seed']), 'policy': values['policy'], 'log_path': values['log_path'],
                  'human_beta': float(values['human_beta']), 'assumed_beta': float(values['assumed_beta']),
                  'trust': float(values['trust']), 'observation_radius': float(values['observation_radius']),
                  'passive_map_rate': float(values['passive_map_rate']),
                  'tom': values['tom'].lower() == 'true',
                  'human_lapse': float(values['human_lapse']), 'assumed_lapse': float(values['assumed_lapse']),
                  'observation_sources': values['observation_sources'],
                  'assumed_observation_radius': float(values['assumed_observation_radius']),
                  'assumed_compliance': float(values['assumed_compliance'])}
        external=params['tom'] and values['warning_engine']!='legacy'
        params.update(external_routing=external,response_config=values['response_config'])
        if params['tom']:
            from hri_reasoning.tom import public_map, DEFAULT_PRIOR, DEFAULT_TRANSITION
            public_path = Path(tempfile.mkdtemp(prefix='hri_public_map_'))/'map.json'
            public_path.write_text(json.dumps(public_map(json.loads(config_file.read_text()))))
            # Separate ROS processes get ONLY the public allowlist, never params/config above.
            inference_params = {'use_sim_time':True, 'public_map':str(public_path),
                'assumed_beta':params['assumed_beta'], 'assumed_lapse':params['assumed_lapse'],
                'assumed_observation_radius':params['assumed_observation_radius'],
                'assumed_compliance':params['assumed_compliance'], 'policy':values['policy'],
                'external_policy':external,
                'observation_sources':values['observation_sources'],
                'tom_prior':ParameterValue(values['tom_prior'] or json.dumps(DEFAULT_PRIOR),value_type=str),
                'tom_transition':ParameterValue(values['tom_transition'] or json.dumps(DEFAULT_TRANSITION),value_type=str)}
            for executable in ('public_perception','bayesian_tom'):
                actions.append(Node(package='hri_reasoning', executable=executable,
                                    parameters=[inference_params], output='screen'))
            if external:
                if values['warning_engine'] not in ('expected_loss','value_iteration','q_learning'):
                    raise ValueError('Unknown warning_engine')
                robot_start=json.loads(config_file.read_text()).get('robot_start',[0,0,0])
                policy_params={'use_sim_time':True,'public_map':str(public_path),
                    'policy':values['policy'],'warning_engine':values['warning_engine'],
                    'warning_config':values['warning_config'],'q_policy':values['q_policy'],
                    'observe_enabled':values['observe_enabled'].lower()=='true',
                    'robot_origin_x':float(robot_start[0]),'robot_origin_y':float(robot_start[1])}
                for executable in ('rescue_routing','oracle_warning' if values['policy']=='oracle' else 'warning_decision'):
                    actions.append(Node(package='hri_reasoning',executable=executable,parameters=[policy_params],output='screen'))
        if values['uav_controller'] not in ('scripted_pose','external'):
            raise ValueError('uav_controller must be scripted_pose or external')
        for executable in (('forest_manager','scripted_uav','human_behavior','belief_warning','event_logger') if case else ('scenario_manager', 'human_behavior', 'belief_warning', 'event_logger')):
            if executable=='belief_warning' and params['tom']:
                continue
            if executable=='scripted_uav' and values['uav_controller']=='external':
                continue
            actions.append(Node(package='hri_reasoning', executable=executable,
                                parameters=[params], output='screen'))
    if nav2:
        from hri_bringup.navigation import navigation_actions
        actions += navigation_actions(scenario)
        robot_start=json.loads(config_file.read_text()).get('robot_start',[0,0,0]) if scenario!='minimal' else [0,0,0]
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name='world_odom_transform',
            arguments=['--x',str(robot_start[0]),'--y',str(robot_start[1]),'--frame-id','world','--child-frame-id','odom'],
            parameters=[{'use_sim_time':True}]))
    return actions


def generate_launch_description():
    args = {'scenario': 'forest', 'seed': '7', 'policy': 'B4', 'gui': 'false', 'start_paused': 'false', 'forest_case': '', 'scenario_config':'', 'uav_controller': 'scripted_pose',
            'dashboard': 'true', 'reasoning': 'true', 'nav2': 'auto', 'oil_case':'blockage', 'simulated_operator':'false',
            'mission': 'false', 'waypoints_json': '[[1,0],[1,1]]',
            'trajectory_path': 'results/waypoint_trajectory.csv',
            'log_path': '/work/results/demo.jsonl', 'human_beta': '1.2', 'assumed_beta': '1.2',
            'trust': '0.8', 'observation_radius': '3.0', 'passive_map_rate': '0.05',
            'tom':'true', 'human_lapse':'.03', 'assumed_lapse':'.03',
            'observation_sources':'robot,uav', 'assumed_observation_radius':'3.0',
            'assumed_compliance':'.8', 'tom_prior':'', 'tom_transition':''}
    args.update(warning_engine='expected_loss',warning_config='',response_config='',q_policy='',observe_enabled='true')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
                             + [OpaqueFunction(function=launch_setup)])
