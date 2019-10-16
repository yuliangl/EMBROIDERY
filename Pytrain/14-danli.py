class Dog(object):
    __inst = None

    def __new__(cls):
	if cls.__inst==None:
	    cls.__inst = object.__new__(cls)
	    return cls.__inst

	else:
	    return cls.__inst


a = Dog()
print(id(a))
b = Dog()
print(id(b))
