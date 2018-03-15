# comandos de gnuplot
#set terminal x11 En multiplot no puede ponerse parece ser???????
if ($0==1) clear
if ($0==1) unset time
if ($0==1) unset bmargin
if ($0==1) set size $1,$2   # (x,y)
if ($0==1) set origin $3,$4
if ($0==2) unset title
if ($0==2) set size $1,$2
if ($0==2) set origin $5,$6
if ($0==2) set time

