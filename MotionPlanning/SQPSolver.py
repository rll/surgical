MOSEK_VERSION = 5.0

from cvxopt import matrix, spmatrix, spdiag, sparse, normal
from cvxopt.solvers import qp
#from qp import qp
from time import clock
from sys import argv
try:
  import pymosek
except ImportError:
  import mosek
  MOSEK_VERSION = 6.0


from cvxopt import printing
from cvxopt import solvers

printing.options['dformat'] = '%.5f'
printing.options['width'] = -1
solvers.options['show_progress'] = True

if MOSEK_VERSION == 5.0 and not solvers.options['show_progress']:
  solvers.options['MOSEK'] = {pymosek.iparam.log: 0, 
      pymosek.iparam.intpnt_num_threads: 0}

if MOSEK_VERSION == 6.0 and not solvers.options['show_progress']:
  solvers.options['MOSEK'] = {mosek.iparam.log: 0,
      mosek.iparam.intpnt_num_threads: 0}

def sparseIdentity(n):
  return spmatrix(1.0, range(n), range(n))

def sparseZero(m,n):
  return spmatrix([], [], [], (m,n))

def finiteDifferenceMatrix(num_elements, size_each_element):
  D = sparseZero((num_elements-1)*size_each_element, num_elements*size_each_element)
  for i in range(num_elements-1):
    D[i*size_each_element:(i+1)*size_each_element, i*size_each_element:(i+1)*size_each_element] = sparseIdentity(size_each_element);
    D[i*size_each_element:(i+1)*size_each_element, (i+1)*size_each_element:(i+2)*size_each_element] = sparseIdentity(size_each_element);

  return D

def calculate_state_size(num_traj, num_states, size_each_state, size_each_control):
  num_controls = num_states - 1
  size = num_traj*(num_states-2)*size_each_state # X
  size = size + num_controls*size_each_control # U
  size = size + num_traj*(num_states-1)*size_each_state # L
  size = size + (num_controls - 1)*size_each_control # E
  size = size + num_traj*(num_states-2)*size_each_state # D
  size = size + num_traj*(num_states-2)*size_each_state # XI
  size = size + num_controls*(size_each_control) # UI
  size = size + num_traj*(num_states-1)*size_each_state # B
  return size

def generateP(num_traj, num_states, size_each_state, size_each_control, 
    lambda_1, lambda_2, lambda_3, lambda_4):
  num_controls = num_states - 1
  all_matrices = []
  
  # Unknown states X = (X^1, X^2, ... , X^{num_traj})
  for i in range(num_traj*(num_states - 2)): 
    all_matrices.append(sparseZero(size_each_state,size_each_state))

  # Unknown controls U = (u_0, u_1, ... , u_{num_controls}) 
  for i in range(num_controls): 
    all_matrices.append(2*lambda_1*sparseIdentity(size_each_control))

  # all_trans*[X; U] - B transformation variables L
  for i in range(num_traj*(num_states - 1)): 
    all_matrices.append(2*sparseIdentity(size_each_state))

  # u_{i+1} - u_{i} transformation variables E
  for i in range(num_controls-1): 
    all_matrices.append(2*lambda_2*sparseIdentity(size_each_control));

  # \sum_{i=1}^{n} X_i - X_g transformation variables D
  for i in range(num_traj*(num_states-2)):
    all_matrices.append(2*lambda_3*sparseIdentity(size_each_state));

  # x_i - x_i^init constraints XI
  for i in range(num_traj*(num_states-2)): 
    all_matrices.append(sparseZero(size_each_state,size_each_state))

  # u_i - u_i^init constraints UI
  for i in range(num_controls):
    all_matrices.append(sparseZero(size_each_control,size_each_control))

  # min ||x_i - x_i-1|| = ||Bu|| transformation variables B
  for i in range(num_traj*(num_states-1)):
    all_matrices.append(2*lambda_4*sparseIdentity(size_each_state));

  P = spdiag(all_matrices)
  return P

def generateA(num_traj, num_states, size_each_state, size_each_control, all_trans):
  num_controls = num_states - 1;
  
  XU_block = all_trans;
  L_block = sparseIdentity(num_traj * (num_states - 1) * size_each_state)
  E_block = sparseZero(num_traj*(num_states-1)*size_each_state, 
      (num_controls-1)*size_each_control)
  D_block = sparseZero(num_traj*(num_states-1)*size_each_state, 
      num_traj*(num_states-2)*size_each_state)
  XI_block = sparseZero(num_traj*(num_states-1)*size_each_state,
      num_traj*(num_states-2)*size_each_state)
  UI_block = sparseZero(num_traj*(num_states-1)*size_each_state,
      num_controls*size_each_control)
  B_block = sparseZero(num_traj*(num_states-1)*size_each_state,
      num_traj*(num_states-1)*size_each_state)

  trans_A = sparse([[XU_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])

  X_block = sparseZero(size_each_control*(num_controls-1), 
      num_traj*(num_states-2)*size_each_state)
  U_block = finiteDifferenceMatrix(num_controls, size_each_control)
  L_block = sparseZero(size_each_control*(num_controls-1),
      num_traj*(num_states-1)*size_each_state)
  E_block = sparseIdentity(size_each_control*(num_controls-1))
  D_block = sparseZero(size_each_control*(num_controls-1), 
      num_traj*(num_states-2)*size_each_state)
  XI_block = sparseZero(size_each_control*(num_controls-1),
      num_traj*(num_states-2)*size_each_state)
  UI_block = sparseZero(size_each_control*(num_controls-1),
      num_controls*size_each_control)
  B_block = sparseZero(size_each_control*(num_controls-1),
      num_traj*(num_states-1)*size_each_state)

  udot_A = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])

  X_block = sparseIdentity(num_traj*(num_states-2)*size_each_state);
  U_block = sparseZero(num_traj*(num_states-2)*size_each_state, 
      num_controls*size_each_control);
  L_block = sparseZero(num_traj*(num_states-2)*size_each_state,
      num_traj*(num_states-1)*size_each_state)
  E_block = sparseZero(num_traj*(num_states-2)*size_each_state,
      (num_controls-1)*size_each_control)
  D_block = sparseIdentity(num_traj*(num_states-2)*size_each_state)
  XI_block = sparseZero(num_traj*(num_states-2)*size_each_state,
      num_traj*(num_states-2)*size_each_state)
  UI_block = sparseZero(num_traj*(num_states-2)*size_each_state,
      num_controls*size_each_control)
  B_block = sparseZero(num_traj*(num_states-2)*size_each_state,
      num_traj*(num_states-1)*size_each_state)

  dist_to_goal_A = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])

  X_block = sparseZero(num_traj*(num_states-2)*size_each_state, 
      num_traj*(num_states-2)*size_each_state);
  U_block = sparseZero(num_traj*(num_states-2)*size_each_state, 
      num_controls*size_each_control);
  L_block = sparseZero(num_traj*(num_states-2)*size_each_state, 
      num_traj*(num_states-1)*size_each_state);
  E_block = sparseZero(num_traj*(num_states-2)*size_each_state, 
      size_each_control*(num_controls-1));
  D_block = sparseZero(num_traj*(num_states-2)*size_each_state, 
      num_traj*(num_states-2)*size_each_state);
  XI_block = sparseIdentity(num_traj*(num_states-2)*size_each_state)
  UI_block = sparseZero(num_traj*(num_states-2)*size_each_state,
      num_controls*size_each_control)
  B_block = sparseZero(num_traj*(num_states-2)*size_each_state,
      num_traj*(num_states-1)*size_each_state)

  dist_from_Xinit = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])

  X_block = sparseZero(num_controls*size_each_control, 
      num_traj*(num_states-2)*size_each_state);
  U_block = sparseZero(num_controls*size_each_control, 
      num_controls*size_each_control);
  L_block = sparseZero(num_controls*size_each_control, 
      num_traj*(num_states-1)*size_each_state);
  E_block = sparseZero(num_controls*size_each_control, 
      size_each_control*(num_controls-1));
  D_block = sparseZero(num_controls*size_each_control, 
      num_traj*(num_states-2)*size_each_state);
  XI_block = sparseZero(num_controls*size_each_control,
      num_traj*(num_states-2)*size_each_state);
  UI_block = sparseIdentity(num_controls*size_each_control)
  B_block = sparseZero(num_controls*size_each_control, 
      num_traj*(num_states-1)*size_each_state)

  dist_from_Uinit = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])

  X_block = sparseZero(num_traj*(num_states-1)*size_each_state, 
      num_traj*(num_states-2)*size_each_state);
  U_block = all_trans[:,num_traj*(num_states-2)*size_each_state:]
  L_block = sparseZero(num_traj*(num_states-1)*size_each_state, 
      num_traj*(num_states-1)*size_each_state);
  E_block = sparseZero(num_traj*(num_states-1)*size_each_state, 
      size_each_control*(num_controls-1));
  D_block = sparseZero(num_traj*(num_states-1)*size_each_state, 
      num_traj*(num_states-2)*size_each_state);
  XI_block = sparseZero(num_traj*(num_states-1)*size_each_state,
      num_traj*(num_states-2)*size_each_state);
  UI_block = sparseZero(num_traj*(num_states-1)*size_each_state,
      num_controls*size_each_control)
  B_block = sparseIdentity(num_traj*(num_states-1)*size_each_state)


  dist_from_pairs = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])
  

  A = sparse([trans_A, udot_A, dist_to_goal_A, dist_from_Xinit, dist_from_Uinit, dist_from_pairs])

  return A
   
def generateB(num_traj, num_states, size_each_state, size_each_control, b_data, u_data):
  num_controls = num_states - 1
  B = [ ]
  X_N = [ ]
  init = [ ]
  for i in range(num_traj):
    bi_data = b_data[i*(num_states*size_each_state):(i+1)*(num_states*size_each_state)]
    init.append(bi_data[size_each_state:-size_each_state])
    X_N.append( bi_data[-size_each_state:] )
    B.append(generateBi(num_states, size_each_state, bi_data));

  B.append(sparseZero(size_each_control*(num_controls-1),1));

  for i in range(num_traj):
    for j in range(num_states-2):
      B.append(X_N[i])

  for i in range(num_traj):
    B.append(init[i])

  for i in range(num_controls):
    B.append(u_data[i*size_each_control:(i+1)*size_each_control])

  for i in range(num_traj):
    B.append(sparseZero(size_each_state*(num_controls),1))

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
  q_size =  calculate_state_size(num_traj, num_states, size_each_state, size_each_control)
  q = matrix(0.0, (q_size, 1))
  return q

def generateG(num_traj, num_states, size_each_state, size_each_control):
  num_controls = num_states - 1
  
  num_ctrl_ineq = 2 * (size_each_control * num_controls)
  X_block = sparseZero(num_ctrl_ineq, num_traj*(num_states-2)*size_each_state)
  U_upper_block = sparseIdentity(num_controls*size_each_control)
  U_lower_block = -1*sparseIdentity(num_controls*size_each_control)
  U_block = sparse([U_upper_block, U_lower_block])
  L_block = sparseZero(num_ctrl_ineq, num_traj*(num_states-1)*size_each_state)
  E_block = sparseZero(num_ctrl_ineq, (num_controls-1)*size_each_control)
  D_block = sparseZero(num_ctrl_ineq, num_traj*(num_states-2)*size_each_state)
  XI_block = sparseZero(num_ctrl_ineq, num_traj*(num_states-2)*size_each_state)
  UI_block = sparseZero(num_ctrl_ineq, num_controls*size_each_control)
  B_block = sparseZero(num_ctrl_ineq, num_traj*(num_states-1)*size_each_state)

  ctrl_ineq = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])
  
  num_state_ineq = 2*(num_traj*(num_states-2)*size_each_state)
  X_upper_block = sparseIdentity(num_traj*(num_states-2)*size_each_state)
  X_lower_block = -1 * sparseIdentity(num_traj*(num_states-2)*size_each_state)
  X_block = sparse([X_upper_block, X_lower_block])
  L_block = sparseZero(num_state_ineq, num_traj*(num_states-1)*size_each_state)
  U_block = sparseZero(num_state_ineq, num_controls*size_each_control)
  E_block = sparseZero(num_state_ineq, (num_controls-1)*size_each_control)
  D_block = sparseZero(num_state_ineq, num_traj*(num_states-2)*size_each_state)
  XI_upper_block = -1 * sparseIdentity(num_traj*(num_states-2)*size_each_state)
  XI_lower_block = sparseIdentity(num_traj*(num_states-2)*size_each_state)
  XI_block = sparse([XI_upper_block, XI_lower_block])
  UI_block = sparseZero(num_state_ineq, num_controls*size_each_control)
  B_block = sparseZero(num_state_ineq, num_traj*(num_states-1)*size_each_state)

  state_ineq = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])

  num_ctrl_init_ineq = 2 * (size_each_control * num_controls)

  X_block = sparseZero(num_ctrl_init_ineq, num_traj*(num_states-2)*size_each_state)
  U_upper_block = sparseIdentity(num_controls*size_each_control)
  U_lower_block = -1*sparseIdentity(num_controls*size_each_control)
  U_block = sparse([U_upper_block, U_lower_block])
  L_block = sparseZero(num_ctrl_init_ineq, num_traj*(num_states-1)*size_each_state)
  E_block = sparseZero(num_ctrl_init_ineq, (num_controls-1)*size_each_control)
  D_block = sparseZero(num_ctrl_init_ineq, num_traj*(num_states-2)*size_each_state)
  XI_block = sparseZero(num_ctrl_init_ineq, num_traj*(num_states-2)*size_each_state)
  UI_upper_block = -1 * sparseIdentity(num_controls*size_each_control)
  UI_lower_block = sparseIdentity(num_controls*size_each_control)
  UI_block = sparse([UI_upper_block, UI_lower_block])
  B_block = sparseZero(num_ctrl_init_ineq, num_traj*(num_states-1)*size_each_state)

  ctrl_init_ineq = sparse([[X_block], [U_block], [L_block], [E_block], [D_block], [XI_block], [UI_block], [B_block]])

  G = sparse([ctrl_ineq, state_ineq, ctrl_init_ineq])

  return G

def generateH(num_traj, num_states, control_const_vec, state_const_vec, control_diff_const_vec):
  num_controls = num_states - 1
  H = [ ]
  for i in range(2*num_controls):
    H.append(control_const_vec)
  for i in range(2*num_traj*(num_states-2)):
    H.append(state_const_vec)
  for i in range(2*num_controls):
    H.append(control_diff_const_vec)

  H = matrix(H)

  return H;

def solveSQP(A_m, A_n, A_file, b_m, b_n, b_file, u_file, x_file, num_traj, num_states, size_each_state, size_each_control, lambda_u, lambda_u_dot, lambda_dist_from_goal, lambda_dist_from_pairs, state_init_const, transl_init_const, rot_init_const):
  #read A file to get J

  num_controls = num_states - 1
  max_trans = 2e-1
  max_rot = 5e-2
  lambda_1 = lambda_u
  lambda_2 = lambda_u_dot
  lambda_3 = lambda_dist_from_goal
  lambda_4 = lambda_dist_from_pairs

  control_const_vec = matrix([max_trans, max_trans, max_trans, max_rot, max_rot, max_rot, max_trans, max_trans, max_trans, max_rot, max_rot, max_rot], (12,1))
  
  state_init_const_vec = matrix([state_init_const for i in range(size_each_state)])
  control_diff_init_const_vec = matrix([transl_init_const, transl_init_const, transl_init_const, rot_init_const, rot_init_const, rot_init_const, transl_init_const, transl_init_const, transl_init_const, rot_init_const, rot_init_const, rot_init_const])


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

  f_u = open(u_file, 'r')
  u_data = matrix(0.0, (num_controls*size_each_control, 1))
  i = 0
  for line in f_u:
    u_data[i] = float(line.rstrip())
    i = i + 1
  f_u.close()

  print "setting up loading u_data took %f" % (clock() - t0)
  t0 = clock()

  #setup matrices
  P = generateP(num_traj, num_states, size_each_state, size_each_control, 
      lambda_1, lambda_2, lambda_3, lambda_4)
  q = generateQ(num_traj, num_states, size_each_state, size_each_control)
  A = generateA(num_traj, num_states, size_each_state, size_each_control, all_trans)
  b = generateB(num_traj, num_states, size_each_state, size_each_control, b_data, u_data)
  G = generateG(num_traj, num_states, size_each_state, size_each_control)
  h = generateH(num_traj, num_states, control_const_vec, state_init_const_vec, control_diff_init_const_vec)

  print "generating took %f" % (clock() - t0)
  t0 = clock()
  #solve

  x = qp(P,q,G,h,A,b,solver='mosek')['x']


  print "solving took %f" % (clock() - t0)
  t0 = clock()
  x = x[:num_traj*(num_states-2)*size_each_state+(num_controls)*size_each_control];

  U = matrix(x[num_traj*(num_states-2)*size_each_state:], (12, num_controls))
  #print U

  f = open(x_file, 'w')
  for i in range(x.size[0]):
    f.write('%.8f\n' % x[i])
  f.close()

  print "writing took %f" % (clock()-t0)
  t0 = clock()

def main():

  if argv[1] == 'solver':
    A_m =                      int(argv[2])
    A_n =                      int(argv[3])
    A_file =                       argv[4]
    b_m =                      int(argv[5])
    b_n =                      int(argv[6])
    b_file =                       argv[7]
    u_file =                       argv[8]
    x_file =                       argv[9]
    num_traj =                int(argv[10])
    num_states =              int(argv[11])
    size_each_state =         int(argv[12])
    size_each_control =       int(argv[13])
    lambda_u =              float(argv[14])
    lambda_u_dot =          float(argv[15])
    lambda_dist_from_goal = float(argv[16])
    lambda_dist_from_pair = float(argv[17])
    state_init_const =      float(argv[18])
    transl_init_const =     float(argv[19])
    rot_init_const =        float(argv[20])

    solveSQP(A_m, A_n, A_file, b_m, b_n, b_file, u_file, x_file, num_traj, num_states, size_each_state, size_each_control, lambda_u, lambda_u_dot, lambda_dist_from_goal, lambda_dist_from_pair, state_init_const, transl_init_const, rot_init_const)

if __name__ == "__main__":
  main()

  
