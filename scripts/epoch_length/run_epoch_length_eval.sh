#!/bin/bash

for el in 4 16 64 optimal; do
  ./scripts/epoch_length/epoch_length.py $el > epoch_length_$el.csv
done

gnuplot scripts/epoch_length/epoch_length.plot
