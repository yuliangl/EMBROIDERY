import os

ret = os.fork()
print(ret)
if ret == 0:
	for i in range(3):
		print("zi_jin_cheng")
else:
	for i in range(3):
		print("fu_jin_cheng")





