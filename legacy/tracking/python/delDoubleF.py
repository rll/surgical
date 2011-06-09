import os

files = os.listdir('.');
for f in files:
    if f[-4:] == 'tiff':
        print f
        os.remove(f)

            
