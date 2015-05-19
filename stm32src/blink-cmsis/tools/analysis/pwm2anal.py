#!/usr/bin/env python

state=[0] * 6

samp = 0
samp_up = [0] * 6
samp_down = [0] * 6
period = [0] * 6

f = open('untitled.csv', 'r')
d = [open('d1.txt', 'w'), None, open('d2.txt', 'w'), None, open('d3.txt', 'w'), None]

for line in f.readlines():
	if line[0].isalpha(): continue
	bits=[int(x.strip()) for x in line.split(',')[1:]]

	for chan in [0, 2, 4]:
		if state[chan] == 0 and bits[chan] == 1:
			period[chan] = samp - samp_up[chan]
			if period[chan] > 0:
				duty = 1.0 * (samp_down[chan] - samp_up[chan]) / period[chan]
			else:
				duty = 0
			samp_up[chan] = samp
			d[chan].write("%d %f\n" % (samp, duty))

		if state[chan] == 1 and bits[chan] == 0:
			samp_down[chan] = samp

		state[chan] = bits[chan]
		# print bits[0], samp, samp_up, samp_down, period, duty
		# print samp, duty

	samp = samp + 1

	#print bits, butts