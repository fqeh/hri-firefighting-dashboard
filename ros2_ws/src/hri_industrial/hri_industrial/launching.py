"""Give public nodes only allowlisted data; reserve private settings for trials."""
import json,tempfile
from pathlib import Path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from hri_reasoning.tom import public_map,DEFAULT_PRIOR,DEFAULT_TRANSITION


def reasoning_actions(values,config_file,layout_file):
    if values['tom'].lower()!='true' or values['warning_engine']=='legacy':
        raise ValueError('oil_facility reuses Phase 3/4 ToM and expected_loss/value_iteration/q_learning')
    path=Path(tempfile.mkdtemp(prefix='oil_public_'))/'map.json'
    path.write_text(json.dumps(public_map(json.loads(config_file.read_text()))))
    public={'use_sim_time':True,'public_map':str(path),'facility_layout':str(layout_file),
        'assumed_beta':float(values['assumed_beta']),'assumed_lapse':float(values['assumed_lapse']),
        'assumed_compliance':float(values['assumed_compliance']),'observation_sources':'robot',
        'policy':values['policy'],'warning_engine':values['warning_engine'],
        'warning_config':values['warning_config'],'q_policy':values['q_policy'],'observe_enabled':False,
        'tom_prior':ParameterValue(values['tom_prior'] or json.dumps(DEFAULT_PRIOR),value_type=str),
        'tom_transition':ParameterValue(values['tom_transition'] or json.dumps(DEFAULT_TRANSITION),value_type=str)}
    nodes=[Node(package='hri_industrial',executable=e,parameters=[public],output='screen') for e in
        ('industrial_perception','industrial_routing','operator_input','operator_tom',
         'operator_oracle' if values['policy']=='oracle' else 'operator_warning_policy')]
    private={'use_sim_time':True,'scenario':'oil_facility','config':str(config_file),
        'seed':int(values['seed']),'log_path':values['log_path'],'policy':values['policy'],'response_config':values['response_config'],
        'equipment_registry':str(Path(__file__).resolve().parents[1]/'config/equipment_registry.json'),
        'world_file':str(layout_file.parent/'oil_facility.sdf')}
    nodes.append(Node(package='hri_industrial',executable='industrial_manager',parameters=[private],output='screen'))
    nodes.append(Node(package='hri_reasoning',executable='event_logger',parameters=[private],output='screen'))
    if values['simulated_operator'].lower()=='true':
        nodes.append(Node(package='hri_industrial',executable='simulated_operator',parameters=[private],output='screen'))
    return nodes
