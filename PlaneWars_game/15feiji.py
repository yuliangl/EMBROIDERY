import pygame 
from time import *
from pygame.locals import *

def main():
	screen = pygame.display.set_mode((480,852),0,32)#cj chuankou
	
	background = pygame.image.load("./feiji/background.png")#cj beijing
	
	hero  = pygame.image.load("./feiji/hero1.png")#cj feiji
	
	x = 200
	y = 600
	
	while True:
		screen.blit(background,(0,0))
		screen.blit(hero,(x,y))
		pygame.display.update()
	
		for event in pygame.event.get(): #for:get keyboard
			if event.type == QUIT:
				print("exit")
				exit()
			elif event.type == KEYDOWN:
				if event.key == K_a or event.key == K_LEFT:
					print("left")	
					x-=10
				elif event.key == K_d or event.key == K_RIGHT:
					print("right")
					x+=10	
				elif event.key == K_SPACE:
					print("space")	
		sleep(0.02)
if __name__=="__main__":
	main()





