import importlib.util, threading
from pathlib import Path
from types import SimpleNamespace
from geometry_msgs.msg import PoseStamped
spec=importlib.util.spec_from_file_location('bridge',str(Path(__file__).resolve().parents[1] / 'scripts/standalone_grasp_executor.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
bridge=m.StandaloneGraspExecutor.__new__(m.StandaloneGraspExecutor)
bridge.guard=threading.RLock();bridge.busy=True;bridge.grasp_proc=object();bridge.grasp_started_stamp=100.;bridge.reposition_target=None
pose=PoseStamped();pose.header.frame_id='base_link';pose.header.stamp.sec=99;pose.pose.position.x=.63
bridge.on_reposition(pose);assert bridge.reposition_target is None
pose.header.stamp.sec=101;pose.header.frame_id='camera_link';bridge.on_reposition(pose);assert bridge.reposition_target is None
pose.header.frame_id='base_link';bridge.on_reposition(pose);assert bridge.reposition_target['xyz'][0]==.63
bridge.busy=False;pose.pose.position.x=.8;bridge.on_reposition(pose);assert bridge.reposition_target['xyz'][0]==.63
# A reposition result must be unsuccessful, but drive-safe ONLY after child cleanup
# and measured folding. The target must survive cleanup for the coordinator.
bridge.closing=False;bridge.reserved=True;bridge.detection_proc=object();bridge.timeout=1.;bridge.precise_params='/tmp/params.yaml'
bridge.phase='idle';bridge.session='';bridge.task_id=0;bridge.result_code=None
bridge.folded=lambda: True
bridge.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000))
bridge.get_logger=lambda:SimpleNamespace(error=lambda x:None,info=lambda x:None)
clean=[]
bridge.stop_process=lambda process,timeout: clean.append(process)
bridge.start_detection=lambda: setattr(bridge,'detection_proc',object())
heartbeats=[];bridge.heartbeat=lambda:heartbeats.append((bridge.busy,bridge.grasp_proc,bridge.reposition_target))
child=SimpleNamespace(poll=lambda:None)
def spawn(command,label):
 assert 'planner_params_file:=/tmp/params.yaml' in command
 bridge.result_code=-2;bridge.reposition_target={'frame':'base_link','xyz':[.63,0.,0.],'stamp':101.}
 return child
bridge.spawn=spawn
class Handle:
 request=SimpleNamespace(session='trial',task_id=42,command='grasp')
 is_cancel_requested=False
 def abort(self):assert child in clean and bridge.grasp_proc is None
 def canceled(self):raise AssertionError('unexpected cancellation')
 def succeed(self):raise AssertionError('reposition must not report grasp success')
result=bridge.execute(Handle())
assert not result.success and result.safe_to_drive and result.phase=='READY'
assert bridge.reposition_target['xyz'][0]==.63
assert heartbeats[-1][0] is False and heartbeats[-1][1] is None
print('PASS: stale/wrong-frame/inactive observations rejected; reposition survives cleanup; arm cleanup precedes drive-safe result')
bridge.prepare_params='/tmp/prepare.yaml'
folded=[False]
bridge.folded=lambda:folded[0]
bridge.detection_proc=object()
def prepare_spawn(command,label):
 assert 'planner_params_file:=/tmp/prepare.yaml' in command
 assert label.startswith('prepare_') and not folded[0]
 # Simulate successful physical staged folding, not a pre-existing ready pose.
 folded[0]=True
 bridge.result_code=1
 return child
bridge.spawn=prepare_spawn
class PrepareHandle(Handle):
 request=SimpleNamespace(session='trial',task_id=43,command='prepare')
 def succeed(self):assert folded[0] and child in clean and bridge.grasp_proc is None
 def abort(self):raise AssertionError('prepare must actively fold instead of rejecting unfolded arm')
result=bridge.execute(PrepareHandle())
assert result.success and result.safe_to_drive
print('PASS: prepare starts from unfolded feedback, runs dedicated arm command, and confirms folded state after cleanup')
