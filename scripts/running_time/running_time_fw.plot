set terminal postscript eps enhanced size 3.3in,1.3in font 'Helvetica,13'
set output 'running_time_fw.eps'

set xlabel "Number of vehicles"
set ylabel "Time per veh. update [{/Symbol m}s]"

set xtics nomirror
set ytics nomirror

set key top right

set grid
set xrange [1:32]
set yrange [0:6]

set log x 2

plot  "running_time_fw_4096.csv" using 1:($2 / 1000) with linespoints lw 3 title "Granularity: 0.0625",\
      "running_time_fw_8192.csv" using 1:($2 / 1000) with linespoints lw 3 title "0.125",\
      "running_time_fw_16384.csv" using 1:($2 / 1000) with linespoints lw 3 title "0.25"
