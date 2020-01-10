set terminal postscript eps enhanced size 3.3in,1.3in font 'Helvetica,13'
set output 'velocity_ins.eps'

set xlabel "Velocity [m/s]"
set ylabel "Number of input transitions"

set xtics nomirror
set ytics nomirror

set key bottom left

set grid
set log y

set xrange [0:20]
set yrange [1:1e6]

plot "velocity_ins_4096.csv" using ($1 / 65536):2 with lines lw 3 title "Granularity: 0.25",\
     "velocity_ins_8192.csv" using ($1 / 65536):2 with lines lw 3 title "0.125",\
     "velocity_ins_16384.csv" using ($1 / 65536):2 with lines lw 3 title "0.0625"
