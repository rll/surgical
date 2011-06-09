# -*- coding: utf-8 -*-
import os

if os.environ.get('OSTYPE') == 'linux':
    directory = '/windows/D/Research/modelbasedtracking'
else: 
    directory = 'D:\Research\modelbasedtracking'

os.chdir(directory + '/models')
modelname = 'inner_shaft.lwo'
basename = os.path.splitext(modelname)[0]

filePolys = open(directory + '/models/' + basename + '.pols')
fileVerts = open(directory + '/models/' + basename + '.verts')

verts = [(-1, -1, -1)] + [tuple([float(val) for val in line.rstrip().split(' ')]) for line in fileVerts] # vertex indexing starts at 1
polys = [tuple([int(val) for val in line.rstrip().split(' ')]) for line in filePolys]

filePolys.close()
fileVerts.close()

print 'verts and polys read'

edges = {}
for i in range(1,len(verts)):
    edges[i] = list() 
    
pCounter = 0
for poly in polys:
    if pCounter % 1000 == 0:
        print "poly", pCounter
    v0 = poly[0]
    for v1 in poly:
        if v0 == v1:
            continue
        va = min(v0, v1)                # keep from recording duplicates
        vb = max(v0, v1)
        if vb not in edges[va]:
            edges[va].append(vb)        # if not registered in first, will not be registered in second
        if va not in edges[vb]:
            edges[vb].append(va)
        v0 = v1
    v1 = poly[0]			# last edge
    va = min(v0, v1)                # keep from recording duplicates
    vb = max(v0, v1)
    if vb not in edges[va]:
        edges[va].append(vb)
    if va not in edges[vb]:
        edges[vb].append(va)
    pCounter = pCounter + 1


####################	write to file	###########################
fileEdges = open(directory + '/models/' + basename + '.edges_double', 'w')
maxLengthAdjacencyList = max([len(edges[i]) for i in edges])

for vert in edges:
    s1 = (str(vert) + " %i"*len(edges[vert]) % tuple(edges[vert]))
    padding = " 0"*(maxLengthAdjacencyList - len(edges[vert]))
    s = s1 + padding
#    print s
    fileEdges.write(s + '\n')
fileEdges.close()

fileEdgesSingle = open(directory + '/models/' + basename + '.edges', 'w')
for vert in edges:
    edgesNoDups = [v for v in edges[vert] if v > vert]
    s1 = str(vert) + " %i"*len(edgesNoDups) % tuple(edgesNoDups)
    padding = " 0"*(maxLengthAdjacencyList - len(edgesNoDups))
    s = s1 + padding
#    print s
    fileEdgesSingle.write(s + '\n')
fileEdgesSingle.close()


###################### print list of edges #############################
fileEdgeList = open(directory + '/models/' + basename + '.edgeList', 'w')
for vert in edges:
    edgesNoDups = [v for v in edges[vert] if v > vert]
    for v in edgesNoDups:
        s = str(vert) + " " + str(v)
        fileEdgeList.write(s + '\n')
        
fileEdgeList.close()
