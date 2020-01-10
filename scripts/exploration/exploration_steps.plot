set terminal postscript eps enhanced size 3.3in,1.3in font 'Helvetica,13'
set output 'exploration_steps.eps'

set xlabel "Time step"
set ylabel "Number of states"

set xtics nomirror
set ytics nomirror

set key top left

#ylower=140
#yupper=240
set xrange [9:0]

set grid
set log y

plot "exploration_8_veh.csv" using 1:2 with lines lw 3 lt 2 title "Visited",\
     "" using 1:3 with lines lw 3 lt 1 title "Valid"
