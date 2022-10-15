#!/bin/bash
docker build -t marrtino:robotics-3d -f Dockerfile.robotics-3d .

docker container prune -f
docker image prune -f