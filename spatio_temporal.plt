set term pngcairo
set output "spatio_temporal.png"
set xlabel "Time"
set ylabel "Position"
unset key
set yrange [0:26]
p "spatio_temporal.dat" pt 0
