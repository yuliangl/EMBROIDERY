"""
def test(a,b=22):
    result = a+b
    print(result)

test(11)
test(22,33)#ru guo  
test(33)
print("--*50")
"""
def test1(a,d,b=22,c=1,*args,**kwargs):
	
    print(a)
    print(b)
    print(c)
    print(d)
    print(args)
    print(kwargs)
A=(11,22,33)
B={"name":"laoshi","age":"60"}
test1(1,1,*A,**B)
#test1(d=2,a=1)






