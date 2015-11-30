import pygame

BITS = 10

def encode(x):
	phases = []
	for i in xrange(BITS):
		bit = x & 0x01
		x >>= 1
		phases = phases + [[0,1],[1,0]][bit]
	return phases

def main():
    pygame.init()
    surf = pygame.Surface((2**BITS,BITS * 2))
    for v in xrange(2**BITS):
    	phases = encode(v)
    	print phases
    	for n in xrange(len(phases)):
    		surf.set_at((v,n), [0xff00000000, 0xffffffff][phases[n]])
    pygame.image.save(surf, 'manchester-%s.png' % BITS)


    surf = pygame.Surface((2**BITS, BITS))


main()