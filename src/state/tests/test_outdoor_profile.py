import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1]
loader=SourceFileLoader('mission_launch',str(root/'launch/state.launch'))
spec=importlib.util.spec_from_loader(loader.name,loader)
m=importlib.util.module_from_spec(spec);loader.exec_module(m)
config=yaml.safe_load((root/'config/mission.yaml').read_text())['mission_state']['ros__parameters']
base={'controller_server':{'ros__parameters':{'goal_checker_plugins':['general_goal_checker']}}}
profile=m.outdoor_goal_profile(base,config)['controller_server']['ros__parameters']
assert base['controller_server']['ros__parameters']['goal_checker_plugins']==['general_goal_checker']
for phase in ('entry','exit'):
 checker=profile[phase+'_goal_checker']
 if phase=='exit':
  assert checker['xy_goal_tolerance']==config['unload_alignment.goal_tolerance']
  assert checker['yaw_goal_tolerance']>3.14
 else:
  assert checker['xy_goal_tolerance']==config[phase+'_navigation.position_tolerance']
  assert checker['yaw_goal_tolerance']==config[phase+'_navigation.yaw_tolerance']
 assert checker['plugin']=='nav2_controller::SimpleGoalChecker'
 assert profile['goal_checker_plugins'].count(phase+'_goal_checker')==1
bad=dict(config);bad['exit_navigation.yaw_tolerance']=float('nan')
try:m.outdoor_goal_profile(base,bad)
except ValueError:pass
else:raise AssertionError('invalid destination tolerance accepted')
print('PASS: both coordinate goals use configured tolerances; invalid values rejected')

nav=yaml.safe_load((root.parent/'bringup/config/nav2_params.yaml').read_text())
nav=m.outdoor_speed_profile(nav,config)
c=nav['controller_server']['ros__parameters']
assert c['UnloadAlignPath']['vx_max']==config['unload_alignment.speed']
assert not c['UnloadAlignPath']['GoalAngleCritic']['enabled']
assert c['OutdoorPath']['vx_max']==config['navigation_speed.outdoor_max_linear']
assert c['FollowPath']['vx_max']==config['cave_navigation.max_linear_speed']
assert c['FollowPath']['wz_max']==config['cave_navigation.max_angular_speed']
assert nav['velocity_smoother']['ros__parameters']['max_velocity'][0]>=config['cave_navigation.max_linear_speed']
# A field change must reach the controller and smoother independently of outdoor speed.
for speed in (.12, .45):
 changed=dict(config);changed['cave_navigation.max_linear_speed']=speed
 result=m.outdoor_speed_profile(nav,changed)
 assert result['controller_server']['ros__parameters']['FollowPath']['vx_max']==speed
 assert result['controller_server']['ros__parameters']['OutdoorPath']['vx_max']==config['navigation_speed.outdoor_max_linear']
 assert result['velocity_smoother']['ros__parameters']['max_velocity'][0]>=speed
for invalid in (0., -.1, float('nan'), float('inf')):
 bad=dict(config);bad['cave_navigation.max_linear_speed']=invalid
 try:m.outdoor_speed_profile(nav,bad)
 except ValueError:pass
 else:raise AssertionError('invalid cave speed accepted')
print('PASS: cave speed follows YAML through controller/smoother without changing outdoor speed')
for angular in (.2, .6):
 changed=dict(config);changed['cave_navigation.max_angular_speed']=angular
 result=m.outdoor_speed_profile(nav,changed)
 assert result['controller_server']['ros__parameters']['FollowPath']['wz_max']==angular
 assert result['velocity_smoother']['ros__parameters']['max_velocity'][2]>=angular
 assert result['velocity_smoother']['ros__parameters']['min_velocity'][2]<=-angular
for invalid in (0., -.1, float('nan'), float('inf'), .81):
 bad=dict(config);bad['cave_navigation.max_angular_speed']=invalid
 try:m.outdoor_speed_profile(nav,bad)
 except ValueError:pass
 else:raise AssertionError('invalid cave angular speed accepted')
print('PASS: cave angular speed reaches controller/smoother; invalid values rejected')
