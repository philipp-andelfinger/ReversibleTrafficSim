#!/bin/bash

for granularity in 16384 8192 4096; do
  ./main 2 1 -1 `expr 65536 \* 40` $granularity $granularity 0.1 1 100 1000 2>&1 | grep 'input transitions:' | awk '{print $3" "$4}' > velocity_ins_$granularity.csv
done

gnuplot scripts/velocity_ins/velocity_ins.plot
