#!/bin/bash

g=4096
for n in 1 2 4 8; do 
  next_seed=1
  for seed in `seq 1 10`; do
    echo "seed: $next_seed"
    ( /usr/bin/time ./main 1 1 $next_seed `expr 65536 \* 40` $g $g 0.1 $n 0 10 2>&1 ) > results_exp_${g}_${n}_${seed}.dat
    next_seed=`expr $? + 1`
  done
done

rm exploration_vehicles.csv
for num_agents in 1 2 4 8; do
  echo -n "$num_agents, " >> exploration_vehicles.csv
  ./scripts/exploration/collect_exp_results.pl results_exp_4096_${num_agents}_*.dat | head -n1 | awk '{print $2" "$3}' >> exploration_vehicles.csv
done

./scripts/exploration/collect_exp_results.pl results_exp_4096_8_*.dat > exploration_8_veh.csv

gnuplot scripts/exploration/exploration_vehicles.plot
gnuplot scripts/exploration/exploration_steps.plot
