# -*- coding: utf-8 -*-
import os
import numpy

if os.environ.get('OSTYPE') == 'linux':
    directory = '/windows/D/Research/modelbasedtracking'
else: 
    directory = 'D:\Research\modelbasedtracking'

os.chdir(directory + '/models')
modelname = 'LND_3.lwo'
basename = os.path.splitext(modelname)[0]

filePolys = open(directory + '/models/' + basename + '.pols')
fileVerts = open(directory + '/models/' + basename + '.verts')

verts = [(-1,-1,-1)] + [tuple([float(val) for val in line.rstrip().split(' ')]) for line in fileVerts] 	# add a dummy vert for vertex 0; vertex indexing starts at 1
polys = [tuple([int(val) for val in line.rstrip().split(' ')]) for line in filePolys]

filePolys.close()
fileVerts.close()

print 'verts and polys read'

norms = []
for poly in polys:
    vert0 = verts[poly[0]]                # polys must have at least 3 verts
    vert1 = verts[poly[1]]
    vert2 = verts[poly[2]]
    vect1 = (vert1[0] - vert0[0], vert1[1] - vert0[1],vert1[2] - vert0[2])
    vect2 = (vert2[0] - vert1[0], vert2[1] - vert1[1],vert2[2] - vert1[2])
    norm = numpy.cross(vect2, vect1)      # should be pointing to exterior of face
    norm = norm/numpy.linalg.norm(norm)
    norms.append(norm)

print "some norms", norms[0:10]

fileNorms = open(directory + '/models/' + basename + '.poly_norms', 'w')
for norm in norms:
    s = " ".join([str(val) for val in norm])
    fileNorms.write(s + '\n')
    
fileNorms.close()
print ('norms written')


################### compute barycenters ########################
centers = []
for poly in polys:
    polyVerts = [verts[vertId] for vertId in poly]
    sumVerts = numpy.sum(polyVerts, axis = 0)
    centroid = sumVerts/len(polyVerts)
    centers.append(centroid)
    
fileBarycenters = open(directory + '/models/' + basename + '.barycenters', 'w')
for datum in centers:
    s = " ".join([str(val) for val in datum])
    fileBarycenters.write(s + '\n')
    
fileBarycenters.close()
print('barycenters written')
        