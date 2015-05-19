#!/bin/bash

./svpwm.py
gnuplot -e 'set term canvas; set title "SVPWM-Popokurven"; set yrange [0:255]; plot "popokurven-+0.txt" with lines title "0", "popokurven--120.txt" with lines title "-120", "popokurven-+120.txt" with lines title "+120"' > svpwm.html
