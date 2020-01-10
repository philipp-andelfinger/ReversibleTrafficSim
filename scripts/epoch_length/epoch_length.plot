set terminal postscript eps enhanced size 3.3in,1.3in font 'Helvetica,13'
set output 'epoch_length.eps'

set xlabel "Lane-change probability"
set ylabel "Exp. bits per time step"

set xtics nomirror
set ytics nomirror

#set key bottom right
set key top left width -2

#ylower=140
#yupper=240

set grid
set xrange [0.0:0.6]
set yrange [0:5]

naive_bits_per_ts = 2.0

plot 2 lw 1.75 title "Naive",\
"epoch_length_64.csv" using 1:4 with lines title "Steps/Epoch: 64",\
"epoch_length_16.csv" using 1:4 with lines title "16",\
"epoch_length_4.csv" using 1:4 with lines title "4",\
"epoch_length_optimal.csv" using 1:4 with lines lw 4 title "Optimal"
