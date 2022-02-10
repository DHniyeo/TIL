set xlabel "x"
set ylabel "y"

plot "out.txt" using 1:2 title "this is title" with line lw 2

pause mouse any "Any key or button will terminate window"
