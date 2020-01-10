#!/bin/bash

make -j4

echo executing epoch length experiment
./scripts/epoch_length/run_epoch_length_eval.sh

echo executing garbage bits experiment
./scripts/garbage_bits/run_gb_eval.sh

echo executing input transition experiment
./scripts/velocity_ins/run_velocity_ins_eval.sh

echo executing running time experiment
./scripts/running_time/run_running_time_eval.pl

echo executing reverse exploration experiment
./scripts/exploration/run_exploration_eval.sh

echo executing case study experiment
./scripts/case_study/run_case_study_eval.sh

echo executing table size experiment
./scripts/table_size/run_table_size_eval.pl

ls -lh *.eps
