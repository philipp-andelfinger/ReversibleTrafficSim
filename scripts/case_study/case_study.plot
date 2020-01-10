set terminal postscript eps enhanced size 3.3in,1.3in font 'Helvetica,13'
set output "case_study.eps"
set xlabel "Distance from obstacles [m]"
set ylabel "Frequency"
set boxwidth 1
set xrange [40:0]
set style line 2 lc rgb 'gray30' lt 1 lw 1
set key off
set log y
set style fill solid 0.2
plot "distance_hist.csv" u 1:2 with boxes ls 2
