#!/bin/bash

for step in 4096 8192 16384; do
  rm garbage_bits_${step}.csv
  for num_agents in 1 2 4 8 16 32; do
    echo -n "${num_agents}, " >> garbage_bits_${step}.csv
    grep 'per time step per vehicle: ' results/garbage_bits_${step}_${num_agents}.dat | awk '{ total += $6 } END { print total/NR }' >> garbage_bits_${step}.csv
  done
done

gnuplot scripts/garbage_bits/garbage_bits.plot
