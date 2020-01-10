set terminal postscript eps enhanced size 3.3in,1.3in font 'Helvetica,13'
set output 'garbage_bits.eps'

set xlabel "Number of vehicles"
set ylabel "Garbage bits per time step"

set xtics nomirror
set ytics nomirror

set key top left

set grid
set xrange [1:32]
set yrange [0.7:1.45]

set log x 2

naive_bits_per_ts = 2.0

plot "garbage_bits_16384.csv" using 1:2 with linespoints lw 3 title "Granularity: 0.25",\
      "garbage_bits_8192.csv" using 1:2 with linespoints lw 3 title "0.125",\
      "garbage_bits_4096.csv" using 1:2 with linespoints lw 3 title "0.0625"
