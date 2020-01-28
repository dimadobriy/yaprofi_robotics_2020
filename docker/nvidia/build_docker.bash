#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

# docker build -t yaprofi-task-base -f $ROOT_DIR/docker/nvidia/Dockerfile.nvidia $ROOT_DIR

docker build -t yaprofi-task-base -f .
