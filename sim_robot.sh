#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROBOT_DIR="${SCRIPT_DIR}/ROS/src/sim_robot"

export GZ_SIM_RESOURCE_PATH="${SIM_ROBOT_DIR}/models:${GZ_SIM_RESOURCE_PATH}"

gnome-terminal -- bash -c "gz sim -v 4 ${SIM_ROBOT_DIR}/worlds/friction_world.sdf" &

sleep 3

gz service -s /world/friction_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req "sdf_filename: \"${SIM_ROBOT_DIR}/models/THex_Quadruped/model.sdf\", name: \"THex_Quadruped\", pose: { position: { x: 0, y: 0, z: 0.5 } }"

gz service -s /world/friction_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req "sdf_filename: \"${SIM_ROBOT_DIR}/models/Cube/model.sdf\", name: \"Cube\", pose: { position: { x: 0, y: 0, z: 0.1 } }"
