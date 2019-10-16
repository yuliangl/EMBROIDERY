class Animal():
	def __init__(self,new_age,new_weight):
		self.age = new_age
		self.weight = new_weight
	def __str__(self):
		return "wangcai age is %d, weight is %dkg"%(self.age,self.weight)
	def eat(self):
		print("------chi------")
	def drink(self):
		print("------he------")
	def sleep(self):
		print("------shui------")
	def run(self):
		print("------pao------")

class Dog(Animal):
	def bark(self):
		print("------jiao------")	
		
		
wangcai = Dog(4,21)	
print(wangcai)


	
		
		
		
		
		




