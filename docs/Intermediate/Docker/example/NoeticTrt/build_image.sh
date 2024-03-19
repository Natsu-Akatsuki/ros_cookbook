#!/bin/bash
docker build -t helios:noetic-trt1.0 --network=host -f Dockerfile .