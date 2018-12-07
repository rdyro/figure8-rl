###############################################################################
set terminal pngcairo dashed size 900,650 font "Arial,18"
set output "mdp_it.png"

set logscale y 10
set xlabel "Iteration # (1)"
set ylabel "Number of Revised Actions (1)"
set format y "%5.1e"

plot "../data/vlit_it.txt" u 1:2 w l not lw 2 lc rgb 'blue'
###############################################################################

###############################################################################
set terminal pngcairo dashed size 900,650 font "Arial,18"
set output "mdp_v.png"

unset logscale y
set xlabel "Depth (1)"
set xrange [1:]
set ylabel "Average Velocity (m/s)"
set format y "%5.1e"
unset mxtics
set xtics 1

set key right bottom

plot "../data/vlit_avgv.txt" u ($0 + 1):($1) w l not lw 2 dt 2 lc rgb 'red', \
"../data/vlit_avgv.txt" u ($0 + 1):($1) w p title 'Value Iteration' lw 2 pt 4 ps 2 lc rgb 'red', \
"../data/fwds_avgv.txt" u ($0 + 1):($1) w l not lw 2 dt 2 lc rgb 'blue', \
"../data/fwds_avgv.txt" u ($0 + 1):($1) w p title 'Forward Search' lw 2 pt 4 ps 2 lc rgb 'blue', \
"../data/mcts_avgv.txt" u ($0 + 1):($1) w l not lw 2 dt 2 lc rgb 'black', \
"../data/mcts_avgv.txt" u ($0 + 1):($1) w p title 'MCTS' lw 2 pt 4 ps 2 lc rgb 'black'
###############################################################################

###############################################################################
set terminal pngcairo dashed size 900,650 font "Arial,18"
set output "mdp_c.png"

#set logscale y 10
set xlabel "Depth (1)"
set xrange [1:]
set ylabel "Computation Time (s)"
set format y "%5.1e"
unset mxtics
set xtics 1

set key left top

plot "../data/fwds_avgc.txt" u ($0 + 1):($1) w l not lw 2 dt 2 lc rgb 'blue', \
"../data/fwds_avgc.txt" u ($0 + 1):($1) w p title 'Forward Search' lw 2 pt 4 ps 2 lc rgb 'blue', \
"../data/mcts_avgc.txt" u ($0 + 1):($1) w l not lw 2 dt 2 lc rgb 'black', \
"../data/mcts_avgc.txt" u ($0 + 1):($1) w p title 'MCTS' lw 2 pt 4 ps 2 lc rgb 'black'
###############################################################################
