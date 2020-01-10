#!/bin/bash

num_vehicles=4
num_obstacles=10
granularity=8192
time_steps=10
time_step_size=0.1
sensing_range=40
seed=1
repetitions=1

mkdir traces
( /usr/bin/time ./main 1 $repetitions $seed `expr 65536 \* $sensing_range` $granularity $granularity $time_step_size $num_vehicles $num_obstacles $time_steps 2>&1 ) > exploration.txt
