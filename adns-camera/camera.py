import pygame
import serial

def SerialInit():
    return serial.Serial('/dev/tty.SLAB_USBtoUART', 230400, timeout=1)

def CaptureImage(cer):
    cer.write("f")
    while cer.readline().strip() != 'DUMP':
        #print "waiting..."
        pass
    #cock = cer.readline().split()
    #return [int(x, 16) for x in cock]
    cock = [None] * 900
    for i in xrange(900):
         cock[i] = ord(cer.read()) - 32
    return cock


def main():
    pygame.init()
    pygame.display.set_caption("ADNS-9800 camera test")
    screen = pygame.display.set_mode((120,120))
    clock = pygame.time.Clock() 
    running = True
    testimage = pygame.image.load('bldc_thomas.png')

    cereal = SerialInit()

    #print pygame.image.tostring(testimage, 'RGB')
    lasttime = 0
    framecount = 0
    time_accu = 0
    while running:
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            running = False
        else:
            img = CaptureImage(cereal)
            surf = pygame.Surface((30, 30))
            brightness = 1
            for i in xrange(len(img)):
                eff = img[i] * brightness
                if eff > 255:
                    eff = 255
                surf.set_at((i % 30, i / 30), pygame.Color(eff, eff, eff, 255))
            screen.blit(pygame.transform.scale(surf, screen.get_size()), (0, 0))
            pygame.display.flip()
            framecount = framecount + 1
            current_time = pygame.time.get_ticks()
            frametime = current_time - lasttime 
            lasttime = current_time
            time_accu = time_accu + frametime
            if (time_accu > 1000):
                print "FPS=", framecount
                framecount = 0
                time_accu = time_accu - 1000

if __name__=="__main__":
    main()
