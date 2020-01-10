#!/bin/bash

./main 3 1 0 `expr 40 \* 65536` 8192 8192 0.1 2 2 10 > case_study.dat 2>&1 
./scripts/case_study/collect_distances.pl < case_study.dat
gnuplot ./scripts/case_study/case_study.plot
