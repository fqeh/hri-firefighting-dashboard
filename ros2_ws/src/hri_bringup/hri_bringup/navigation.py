"""Nav2 local/global obstacle costmaps from real lidar; no invented free-space map."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def navigation_actions(scenario='industrial'):
    # Start from the installed Jazzy distribution's parameters, retaining upstream
    # controller/BT defaults while configuring this robot's sensors and frames.
    path = Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml'
    config = yaml.safe_load(path.read_text())
    common = {'use_sim_time': True, 'enable_stamped_cmd_vel': False}
    for key, value in config.items():
        if 'ros__parameters' in value:
            value['ros__parameters'].update(common)
    for key in ('bt_navigator', 'behavior_server'):
        config[key]['ros__parameters'].update(global_frame='odom', robot_base_frame='base_link',
                                              odom_topic='/robot/odom', transform_tolerance=0.5)
    # Default 20 ms action handshakes are too short for software rendering in a
    # shared ARM VM. Keep failures explicit but permit normal scheduling delay.
    config['bt_navigator']['ros__parameters'].update(
        default_server_timeout=2000, default_cancel_timeout=2000, wait_for_service_timeout=5000)
    config['controller_server']['ros__parameters'].update(odom_topic='/robot/odom', controller_frequency=10.0)
    follow = config['controller_server']['ros__parameters']['FollowPath']
    if 'MPPI' in follow.get('plugin', ''):
        follow.update(batch_size=400, time_steps=40, model_dt=0.1, visualize=False,
                      vx_max=0.45, vx_min=-0.2, vy_max=0.0, wz_max=1.0,
                      transform_tolerance=0.5)
    for name, size in [('local_costmap', 8), ('global_costmap', 48)]:
        params = config[name][name]['ros__parameters']
        params.update(common, global_frame='odom', robot_base_frame='base_link',
                      rolling_window=True, width=size, height=size, resolution=0.1,
                      robot_radius=0.5, track_unknown_space=True,
                      plugins=['obstacle_layer', 'inflation_layer'], transform_tolerance=0.5)
        params['obstacle_layer'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
            'observation_sources': 'scan', 'scan': {
                'topic': '/robot/scan', 'data_type': 'LaserScan', 'clearing': True,
                'marking': True, 'max_obstacle_height': 3.0,
                'raytrace_max_range': 18.0, 'obstacle_max_range': 15.0,
                'inf_is_valid': True}}
        params['inflation_layer'] = {'plugin': 'nav2_costmap_2d::InflationLayer',
                                     'inflation_radius': 0.8, 'cost_scaling_factor': 3.0}
        if scenario=='oil_facility':
            params['filters']=['keepout_filter','keepout_inflation']
            params['keepout_filter']={'plugin':'nav2_costmap_2d::KeepoutFilter','enabled':True,
                'filter_info_topic':'/oil/filter_info','transform_tolerance':.5,'override_lethal_cost':False}
            params['keepout_inflation']={'plugin':'nav2_costmap_2d::InflationLayer',
                'inflation_radius':.6,'cost_scaling_factor':4.0}
    config['velocity_smoother']['ros__parameters'].update(
        odom_topic='/robot/odom', feedback='OPEN_LOOP', max_velocity=[0.45, 0.0, 1.0],
        min_velocity=[-0.2, 0.0, -1.0])
    # Explicitly launch only the required maintained Nav2 servers; avoid unrelated
    # docking/route-server defaults changing this experiment between apt releases.
    components = [('nav2_controller', 'controller_server'), ('nav2_smoother', 'smoother_server'),
                  ('nav2_planner', 'planner_server'), ('nav2_behaviors', 'behavior_server'),
                  ('nav2_bt_navigator', 'bt_navigator'), ('nav2_velocity_smoother', 'velocity_smoother')]
    # A ROS parameter file is needed for nested costmap node namespaces.
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='hri_nav2_', delete=False) as f:
        yaml.safe_dump(config, f)
        params_path = f.name
    actions = []
    for package, executable in components:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', '/nav2/cmd_vel')]
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', '/nav2/cmd_vel'), ('cmd_vel_smoothed', '/robot/cmd_vel')]
        actions.append(Node(package=package, executable=executable, name=executable,
                            parameters=[params_path], remappings=remaps, output='screen'))
    actions += [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_navigation', parameters=[{
                         'use_sim_time': True, 'autostart': True,
                         'node_names': [executable for _, executable in components]}], output='screen'),
                Node(package='hri_industrial' if scenario=='oil_facility' else 'hri_bringup',
                     executable='shared_navigation' if scenario=='oil_facility' else 'operator_navigation',
                     parameters=[common], output='screen')]
    if scenario in ('forest', 'industrial'):
        actions.append(Node(package='hri_bringup', executable='operator_warning',
                            parameters=[common, {'scenario': scenario}], output='screen'))
    return actions
