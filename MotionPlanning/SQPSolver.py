from cvxopt import matrix, spmatrix, spdiag, sparse, normal
from cvxopt.solvers import qp
#from qp import qp
from time import clock
from sys import argv
import pymosek

from cvxopt import printing
from cvxopt import solvers


printing.options['dformat'] = '%.5f'
printing.options['width'] = -1
solvers.options['show_progress'] = False
#solvers.options['MOSEK'] = {pymosek.iparam.log: 0}

def sparseIdentity(n):
  return spmatrix(1.0, range(n), range(n))

def sparseZero(m,n):
  return spmatrix([], [], [], (m,n))

def generateP(num_traj, num_states, size_each_state, size_each_control, lmbda):
  num_controls = num_states - 1
  all_matrices = []

  for i in range(num_traj*(num_states - 2)): #unknown states
    all_matrices.append(sparseZero(size_each_state,size_each_state))

  for i in range(num_controls): #controls
    all_matrices.append(2*lmbda*sparseIdentity(size_each_control))

  for i in range(num_traj*(num_states - 1)): #transformation variables
    all_matrices.append(2*sparseIdentity(size_each_state))

  P = spdiag(all_matrices)

  return P
def generateA(num_traj, num_states, size_each_state, size_each_control, all_trans):
  num_controls = num_states - 1;
  left_block = all_trans;
  right_block = sparseIdentity(num_traj * (num_states - 1) * size_each_state)
  A = sparse([[left_block], [right_block]])

  return A
   
def generateB(num_traj, num_states, size_each_state, b_data):
  B = [ ]
  for i in range(num_traj):
    bi_data = b_data[i*(num_states*size_each_state):(i+1)*(num_states*size_each_state)]
    B.append(generateBi(num_states, size_each_state, bi_data));

  B = matrix(B);
  return B; 

def generateBi(num_states, size_each_state, bi_data):

  x_0 = -1 * bi_data[0:size_each_state]
  x_N = bi_data[-size_each_state:]

  b = [ ]
  b.append(x_0)
  for i in range(num_states-3):
    b.append(spmatrix([], [], [], (size_each_state, 1)))
  b.append(x_N)

  b = matrix(b);
  return b;


def generateQ(num_traj, num_states, size_each_state, size_each_control):
  q_size =  num_traj*(num_states-2)*size_each_state + (num_states - 1) * size_each_control + num_traj*(num_states-1)*size_each_state
  q = matrix(0.0, (q_size, 1))
  return q

def generateG(num_traj, num_states, size_each_state, size_each_control):
  num_controls = num_states - 1
  num_inequality_constraints = 2 * (size_each_control * num_controls)

  left_block = sparseZero(num_inequality_constraints, num_traj*(num_states-2)*size_each_state)
  upper_middle_block = sparseIdentity(num_controls*size_each_control)
  lower_middle_block = -1*sparseIdentity(num_controls*size_each_control)
  middle_block = sparse([upper_middle_block, lower_middle_block])
  right_block = sparseZero(num_inequality_constraints, num_traj*(num_states-1)*size_each_state)
  G = sparse([[left_block], [middle_block], [right_block]])
  return G

def generateH(num_traj, num_states, control_const_vec):
  num_controls = num_states - 1
  H = [ ]
  for i in range(2*num_controls):
    H.append(control_const_vec)
  
  H = matrix(H)
  return H;

def solveSQP(A_m, A_n, A_file, b_m, b_n, b_file, x_file, num_traj, num_states, size_each_state, size_each_control):
  #read A file to get J

  num_controls = num_states - 1
  max_trans = 2e-1
  max_rot = 5e-2
  lmbda = 0.001

  control_const_vec = matrix([max_trans, max_trans, max_trans, max_rot, max_rot, max_rot, max_trans, max_trans, max_trans, max_rot, max_rot, max_rot], (12,1))

  t0 = clock();
  f_A = open(A_file, 'r')
  rows = [ ]
  cols = [ ]
  vals = [ ]
  for line in f_A:
    x = line.split('\t')
    rows.append(int(x[0])-1)
    cols.append(int(x[1])-1)
    vals.append(float(x[2].rstrip()))

  f_A.close()

  all_trans = spmatrix(vals, rows, cols, (A_m, A_n))

  print "setting up all_trans took %f" % (clock()-t0)
  t0 = clock()

  #read b file to get x_1, x_N
  f_b = open(b_file, 'r')
  b_data = matrix(0.0, (num_traj*num_states*size_each_state, 1))
  i = 0
  for line in f_b:
    b_data[i] = float(line.rstrip())
    i = i + 1
  
  f_b.close()
  
  print "setting up loading b_data took %f" % (clock() - t0)
  t0 = clock()

  #setup matrices
  P = generateP(num_traj, num_states, size_each_state, size_each_control, lmbda)
  q = generateQ(num_traj, num_states, size_each_state, size_each_control)
  A = generateA(num_traj, num_states, size_each_state, size_each_control, all_trans)
  b = generateB(num_traj, num_states, size_each_state, b_data)
  G = generateG(num_traj, num_states, size_each_state, size_each_control)
  h = generateH(num_traj, num_states, control_const_vec)

  print "generating took %f" % (clock() - t0)
  t0 = clock()
  #solve

  x = qp(P,q,G,h,A,b,solver='mosek')['x']


  print "solving took %f" % (clock() - t0)
  t0 = clock()
  x = x[:num_traj*(num_states-2)*size_each_state+(num_controls)*size_each_control];

  U = matrix(x[num_traj*(num_states-2)*size_each_state:], (12, num_controls))
  print U

  f = open(x_file, 'w')
  for i in range(x.size[0]):
    f.write('%.8f\n' % x[i])
  f.close()

  print "writing took %f" % (clock()-t0)
  t0 = clock()

def debug():
  t0 = clock()
  size_each_state = 50
  size_each_control = 12
  num_states = 5
  num_controls = num_states - 1
  lmbda = 0.01
  max_trans = 2e-1
  max_rot = 5e-2
  control_const_vec = matrix([max_trans, max_trans, max_trans, max_rot, max_rot, max_rot, max_trans, max_trans, max_trans, max_rot, max_rot, max_rot], (12,1))

  solveSQP(3880,3612,'SQP_DATA/_alltrans.txt',4268,1,'SQP_DATA/_goalvec.txt','SQP_DATA/_newstate1.txt',1,11,388,12)


  print "Setup took: %f. Starting qp" % (clock() - t0)
  t0 = clock();
  #qp(P,q,G,h,A,b,solver='mosek')['x']
  print clock() - t0


def main():

  if argv[1] == 'solver':
    A_m =                 int(argv[2])
    A_n =                 int(argv[3])
    A_file =                  argv[4]
    b_m =                 int(argv[5])
    b_n =                 int(argv[6])
    b_file =                  argv[7]
    x_file =                  argv[8]
    num_traj =            int(argv[9])
    num_states =          int(argv[10])
    size_each_state =     int(argv[11])
    size_each_control =   int(argv[12])
    solveSQP(A_m, A_n, A_file, b_m, b_n, b_file, x_file, num_traj, num_states, size_each_state, size_each_control)

  elif argv[1] == 'debug':
    debug()
  

if __name__ == "__main__":
  main()

  
