#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Generate SVPWM-Popokurven
#
from math import *

amplitude = 127 * 1.155
counts = 360

def phasesv(angle, amplitude):
	pwm_abc = [amplitude * sin((angle + phase)/180.0 * pi) for phase in [0, -120, +120]]
	Voff = (min(pwm_abc) + max(pwm_abc))/2
	return [x - Voff for x in pwm_abc]

svtable = [phasesv(angle, amplitude) for angle in xrange(counts)]

# We only really need one phase
f = open("popokurve.h", "w")
f.write(("const uint8_t popokurve[%d] = {" % counts) + 
	", ".join([str(128 + int(round(x[0]))) for x in svtable]) + "};\n")
f.close()

# for gnuplot
for phase in xrange(3):
	f = open("popokurven-%+d.txt" % [0,-120,+120][phase], "w")
	f.write("\n".join(["%d %d" % (i,(128 + int(round(x[phase])))) for i,x in enumerate(svtable)]) + "};" )
	f.close()

