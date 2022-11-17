#!/bin/bash
docker run --privileged -it --rm \
    --name pi_comm \
    --network host \
    --hostname pi_comm \
    -v $PWD/rttfcar_commander:/home/user/rttfcar_commander/ \
    --env "DISABLE_CONTRACTS=1" \
    --env "ROS_MASTER_URI=http://localhost:11311" \
    --env "HOSTNAME=master" \
    pi_comm:latest \
    bash