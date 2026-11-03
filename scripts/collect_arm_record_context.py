"""Read-only snapshots of arm parameters, process arguments and ROS logs."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time
import yaml

workspace, destination = map(Path, sys.argv[1:3])
destination.mkdir(parents=True, exist_ok=True)
static = destination / 'configuration'
static.mkdir(exist_ok=True)
paths = [
    'install/lite_motion_planner/share/lite_motion_planner/config/tzb_planner.yaml',
    'install/lite_motion_planner/share/lite_motion_planner/config/tzb_description.srdf',
    'install/lite_motion_planner/share/lite_motion_planner/launch/bring_launch.launch.py',
    'install/lite_motion_planner/share/lite_motion_planner/launch/tzb_catch.launch.py',
    'install/pose_estimator/share/pose_estimator/config/params.yaml',
    'install/pose_estimator/share/pose_estimator/config/camera_switch.yaml',
    'install/pose_estimator/share/pose_estimator/config/half_block.pcd',
    'install/rm_serial_driver/share/rm_serial_driver/config/serial_params.yaml',
    'install/tzb_description/share/tzb_description/urdf/tzb_description.urdf',
]
for relative in paths:
    source = workspace / relative
    if source.is_file():
        target = static / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

manifest = destination / 'processes.json'
processes = json.loads(manifest.read_text()) if manifest.exists() else {}


def parameter_paths(value):
    if isinstance(value, dict):
        for item in value.values():
            yield from parameter_paths(item)
    elif isinstance(value, list):
        for item in value:
            yield from parameter_paths(item)
    elif isinstance(value, str) and value.startswith('/'):
        yield Path(value)


def collect():
    for entry in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            if entry.parent.stat().st_uid != os.getuid():
                continue
            args = [item.decode(errors='replace') for item in entry.read_bytes().split(b'\0') if item]
            if not args:
                continue
            relevant = any(str(workspace / 'install') + '/' in arg for arg in args[:2])
            relevant |= any(arg in args for arg in ['lite_motion_planner', 'pose_estimator', 'yolo_node']) and 'launch' in args
            relevant |= 'component_container' in args[0] and any('camera' in arg for arg in args)
            if not relevant:
                continue
            pid = entry.parent.name
            if pid in processes:
                continue
            processes[pid] = dict(args=args, captured=time.time(), runtime_parameters=[])
            for index, arg in enumerate(args[:-1]):
                if arg == '--params-file' and Path(args[index+1]).is_file():
                    source = Path(args[index+1])
                    target = destination / 'runtime_parameters' / f'{pid}_{index}_{source.name}'
                    target.parent.mkdir(exist_ok=True)
                    shutil.copy2(source, target)
                    processes[pid]['runtime_parameters'].append(str(target.relative_to(destination)))
                    # Preserve the actually selected template and robot model,
                    # including overrides pointing into src rather than install.
                    try:
                        for model in parameter_paths(yaml.safe_load(source.read_text())):
                            if model.is_file() and model.suffix.lower() in ['.pcd', '.urdf', '.srdf']:
                                saved = destination / 'selected_models' / f'{pid}_{model.name}'
                                saved.parent.mkdir(exist_ok=True)
                                shutil.copy2(model, saved)
                            if model.is_dir() and list(model.glob('*.xml')):
                                model_info = {}
                                for item in model.iterdir():
                                    if item.is_file() and item.suffix in ['.yaml', '.txt', '.xml', '.bin']:
                                        model_info[item.name] = hashlib.sha256(item.read_bytes()).hexdigest()
                                processes[pid]['detector_model'] = dict(path=str(model), sha256=model_info)
                    except (OSError, yaml.YAMLError):
                        pass
            executable = Path(args[0])
            if executable.is_file() and str(workspace / 'install') in str(executable):
                processes[pid]['executable_sha256'] = hashlib.sha256(executable.read_bytes()).hexdigest()
        except (OSError, ValueError):
            continue
    temporary = manifest.with_suffix('.next')
    temporary.write_text(json.dumps(processes, indent=2))
    temporary.replace(manifest)


if '--watch' in sys.argv:
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        collect()
        time.sleep(2)
else:
    collect()
    log_root = Path(os.environ.get('ROS_LOG_DIR', str(Path.home() / '.ros/log')))
    logs = destination / 'node_logs'
    logs.mkdir(exist_ok=True)
    if log_root.is_dir():
        for source in log_root.iterdir():
            try:
                if source.is_file() and any(f'_{pid}_' in source.name for pid in processes):
                    shutil.copy2(source, logs / source.name)
                if source.is_dir() and source.name.rsplit('-', 1)[-1] in processes:
                    for item in source.glob('*.log'):
                        shutil.copy2(item, logs / f'{source.name}_{item.name}')
            except OSError:
                continue
