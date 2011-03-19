import os

files = os.listdir('.');
for f in files:
	print f[-4:]
	if f[-4:] == 'tiff':
		f2 = f[:-1]
		print f2
		os.system("cp %s %s" % (f,f2))		

		
