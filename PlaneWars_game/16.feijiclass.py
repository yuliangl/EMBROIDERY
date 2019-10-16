import pygame 
from time import *
from pygame.locals import *
import random 
class HeroPlane():
    def __init__(self,screen_temp):
	self.x = 200
	self.y = 600
	self.screen = screen_temp
	self.image =  pygame.image.load("./feiji/hero1.png")#cj feiji
	self.bullet_list = []#cun chu zidan

    def display(self):
	
	self.screen.blit(self.image,(self.x,self.y))

	for bullet in self.bullet_list:
	    bullet.display()
 	    if bullet.judge():
	        self.bullet_list.remove(bullet)
    def left(self):
	self.x -= 10

    def right(self):
	self.x += 10

    def fire(self):
	self.bullet_list.append(Bullet(self.screen,self.x,self.y))

class Bullet():
    def __init__(self,screen_temp,x,y):
	self.x = x+40
	self.y = y-20
	self.screen = screen_temp
	self.image =  pygame.image.load("./feiji/bullet.png")#cj zidan

    def display(self):
	self.screen.blit(self.image,(self.x,self.y))
	self.y -=10
    def judge(self):
	if self.y < 100:
	    return True
	else:
	    return False	

class EnemyPlane():
    def __init__(self,screen_temp):
	self.x = 0
	self.y = 0
	self.screen = screen_temp
	self.image =  pygame.image.load("./feiji/enemy0.png")#cj feiji
	self.bullet_list = []#cun chu dijizidan
	self.direction = "right"
    def display(self):
	self.screen.blit(self.image,(self.x,self.y))
	for bullet in self.bullet_list:
	    bullet.display()
	    bullet.move()
 	    if bullet.judge():
	        self.bullet_list.remove(bullet)

    def fire(self): 
	if random.randint(1,30) == 5:
	    self.bullet_list.append(EnemyBullet(self.screen,self.x,self.y))
	

    def move(self):
	if self.direction == "right":
	    self.x += 5
	elif self.direction == "left":
	    self.x -= 5
	
	if self.x <= 0:
	    self.direction = "right"
	elif self.x >= 480-50:
	    self.direction = "left"

class EnemyBullet():
    def __init__(self,screen_temp,x,y):
	self.x = x+25
	self.y = y+25
	self.screen = screen_temp
	self.image =  pygame.image.load("./feiji/bullet1.png")#cj zidan

    def display(self):
	self.screen.blit(self.image,(self.x,self.y))
    def move(self):
	self.y += 20
    
    def judge(self):

	if self.y > 600:
	    return True
	else:
	    return False	
def Keyboard_control(hero_temp):
    for event in pygame.event.get(): #for:get keyboard
	if event.type == QUIT:
	    print("exit")
	    exit()
	elif event.type == KEYDOWN:
	    if event.key == K_a or event.key == K_LEFT:
		print("left")	
		hero_temp.left()
	    elif event.key == K_d or event.key == K_RIGHT:
		print("right")
		hero_temp.right()	
	    elif event.key == K_SPACE:
		print("space")	
		hero_temp.fire()

def main():
    screen = pygame.display.set_mode((480,852),0,32)#cj chuankou

    background = pygame.image.load("./feiji/background.png")#cj beijing

    hero = HeroPlane(screen)
   
    enemy = EnemyPlane(screen)

    while True:
	screen.blit(background,(0,0))
	hero.display()
	enemy.display()
	enemy.move()
	enemy.fire()
	pygame.display.update()
	Keyboard_control(hero)

	
  	sleep(0.02)
if __name__=="__main__":
	main()




