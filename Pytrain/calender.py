#-*-coding=utf-8-*-

def leap_year(year):
	if year % 4 == 0 and year % 100 != 0 or year % 400 == 0:
		return True
	else:
		return False

class Months():
	def __init__(self ,year ,month):
		self.year = year
		self.month = month
		self.big = [1,3,5,7,8,10,12]
		self.small = [4,6,9,11]
		self.leap = [2]
	def days(self):
		if self.month in self.big:
			return 31
		elif self.month in self.small:
			return 30
		elif self.month in self.leap:
			if leap_year(year):
				return 28
			else:
				return 29
	def week(self):
		if (self.month == 1 or self.month == 2):
			self.month += 12
			self.year -= 1
		c = self.year/100
		y = self.year%100
		m = self.month
		d = 1
		w = c / 4 - 2 *c + y +y / 4 + 26*(m+1) / 10 + d - 1	   
		if(w < 0):
			return (w + (-w / 7 + 1) * 7) % 7
		else:
			return w % 7
	def month_body(self):
		print("年月: %d %d"%(year,month))
		print("日  月  火  水  木  金  土")
		j = self.week() 
		if j != 0:
			print "    "*j,
		for i in range(1,self.days()+1):
			print "%-3d"%i,
			j += 1
			if j % 7 == 0:
				print ""
		
year ,month = input("please input dates:")
one_month = Months(year ,month)
one_month.month_body() 

