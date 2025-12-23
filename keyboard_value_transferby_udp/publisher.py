import pygame
import sys
import time
import socket

udp_ip="127.0.0.1"
udp_port=5005


def main():
    pygame.init()
    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pygame.display.set_mode((1000, 1000))
    mf=1500
    mb=1500
    data= ""
    while True:
        f=0
        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                pygame.quit()
                
            elif event.type == pygame.KEYDOWN:
                if event.key ==pygame.K_w:
                    print("Up")
                    mf=2000
                    mb=2000
                    f=1
                elif event.key == pygame.K_s:
                    print("Down")
                    mf=1000
                    mb=1000
                    f=1
                elif event.key == pygame.K_ESCAPE:
                    mf=1500
                    mb=1500
                    print(data)
                    pygame.quit()
                    sys.exit()
        # if(f==0):
        #     mf=1500
        #     mb=1500
                

        data="["+str(mf)+","+str(mb)+"]"
        sock.sendto(data.encode(),(udp_ip, udp_port))
        print("Sent:", data)
        print(data)
    
if __name__ == "__main__":
    main()