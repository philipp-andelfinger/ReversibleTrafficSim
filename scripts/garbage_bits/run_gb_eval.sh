#!/bin/bash

mkdir results

for step in 16384 8192 4096; do
  for num_agents in 1 2 4 8 16 32; do
    ( ./main 0 1000 -1 `expr 65536 \* 40` $step $step 0.1 $num_agents 100 1000 2>&1 ) > results/garbage_bits_${step}_${num_agents}.dat
  done
done

./scripts/garbage_bits/collect_gb_results.sh
