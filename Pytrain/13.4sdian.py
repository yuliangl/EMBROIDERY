class CarStore():
    def order(self,money):
        if money>=50000:
	    return Car()
class Car():
    def move(self):
	print("car is moving")
    def music(self):
	print("car is playing music")
    def stop(self):
	print("car is syop")

car_store = CarStore()
car = car_store.order(100000)
#car = Car
car.move()
car.music()
car.stop()

