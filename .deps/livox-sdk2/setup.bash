# Source this file before building or running with the workspace-local Livox SDK2.
_rover_livox_sdk_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export CMAKE_PREFIX_PATH="${_rover_livox_sdk_dir}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${_rover_livox_sdk_dir}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset _rover_livox_sdk_dir
