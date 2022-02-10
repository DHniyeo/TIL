set xlabel "x"
set ylabel "y"

plot "out.txt" using 1:2 title "this is title" with lines lw 3

pause mouse any "Any key or button will terminate window"
