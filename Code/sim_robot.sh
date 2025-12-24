#!/bin/bash

export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models;

gnome-terminal -- bash -c "gz sim ~/.gz/worlds/friction_world.sdf" &

# gnome-terminal -- bash -c "nautilus /home/raahimhash/.gz/models/THex_Quadruped" &

sleep 3

gz service -s /world/friction_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/home/raahimhash/.gz/models/THex_Quadruped/model.sdf", name: "THex_Quadruped", pose: { position: { x: 0, y: 0, z: 0.5 } }'

gz service -s /world/friction_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/home/raahimhash/.gz/models/Cube/model.sdf", name: "Cube", pose: { position: { x: 0, y: 0, z: 0.1 } }'
