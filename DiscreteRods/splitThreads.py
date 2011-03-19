


files = ['suturenylon_processed_projected',
         'sutureblack_processed_projected',
         'suturepurple_processed_projected']

for fname in files:
  with open('LearnParams/'+fname+'.txt', 'r') as fin:
    numpts = int(fin.readline().strip())
    for i, line in enumerate(fin):
      with open('threaddata/'+fname[:fname.find('_')]+'_'+str(i)+'.txt', 'w') as fout:
        fout.write("%d\n"%numpts)
        fout.write(line)

