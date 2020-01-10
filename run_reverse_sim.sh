#!/bin/bash

num_vehicles=10
num_obstacles=10
granularity=8192
time_steps=100
time_step_size=0.1
sensing_range=40
seed=1
repetitions=1

( ./main 0 1 $seed `expr 65536 \* $sensing_range` $granularity $granularity $time_step_size $num_agents $num_vehicles $num_obstacles $time_steps 2>&1 ) > reverse_sim.txt
