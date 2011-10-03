import sys

prob = sys.argv[1]

f = open('environmentFiles/' + prob + '/' + prob + '_data.txt')
lines = f.readlines()
f.close()

scores = { } 

for line in lines:
	x = line.split('_')
	x[-1] = x[-1].rstrip()
	horizon = int(x[3])
	noise = float(x[4])
	time = float(x[-1].split(' ')[0])
	value = int(x[-1].split(' ')[1])
	
	if horizon not in scores:
		scores[horizon] = { }
		
	if noise not in scores[horizon]:
		scores[horizon][noise] = { } 
		
	if value not in scores[horizon][noise]:
		scores[horizon][noise][value] = 0 
		
	scores[horizon][noise][value] = scores[horizon][noise][value] + 1

d = open('environmentFiles/' + prob + '/' + prob + '_data_plot.txt', 'w')

for horizon in sorted(scores.iterkeys()):
	for noise in sorted(scores[horizon].iterkeys()):
		sucess = 0
		experiments = 0
		for value in scores[horizon][noise]:
			if value == 1:
				sucess += scores[horizon][noise][value]
			experiments += scores[horizon][noise][value]
		d.write("%d %.2f %.2f\n" % (horizon, noise, float(sucess)/float(experiments)))
		print "%d %.2f %.2f %d %d %.1f" % (horizon, noise, float(sucess)/float(experiments), sucess, experiments, time)
d.close();
