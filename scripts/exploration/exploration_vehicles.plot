set terminal postscript eps enhanced size 3.3in,1.3in font 'Helvetica,13'
set output 'exploration_vehicles.eps'

set xlabel "Number of vehicles"
set ylabel "Number of states"

set xtics nomirror
set ytics nomirror

set key top left

set xrange [1:8]
set log x 2

set grid
set log y

plot "exploration_vehicles.csv" using 1:2 with lines lw 3 lt 2 title "Visited",\
     "" using 1:3 with lines lw 3 lt 1 title "Valid"
