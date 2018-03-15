# comandos de gnuplot
#set key top left samplen 1 #right#outside #set key es para los titulos de cada grafica;  set key default
set key outside samplen 1#right#outside #set key es para los titulos de cada grafica;  set key default
#set key box
set mouse doubleclick 0 zoomjump #convendría hacer un ZOOM dinámico...
#bind "ctrl-a" "set  key box"  # no funciona????
set pointsize 1.0
set bmargin 5
#set tmargin 5
#set lmargin 7
#set rmargin 7
#show margin
set grid x y
set title "$1"
#set data style lines
set xlabel "$2"
set xrange [$3 : $4]
set yrange [$5 : $6]
if (($9 == 1)|| ($9==2))set xtics $3, $7 ,$4
if (($9 == 2)|| ($9==3))set ytics $5, $8 ,$6
set time


