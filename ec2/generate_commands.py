import os, sys, time
from MessageHandler import MessageHandler

HOST = "128.32.37.215"
PORT = 10000

ARG_PARAM_FILE  = 1
ARG_EVAL_MODE   = 2
ARG_MIN_TWIST   = 3
ARG_ITER_TWIST  = 4
ARG_MAX_TWIST   = 5
ARG_MIN_GRAV    = 6
ARG_ITER_GRAV   = 7
ARG_MAX_GRAV    = 8

USER_HOST = "wan@slave444.cs.berkeley.edu"

COMPILE_CMD = "cd /home/ubuntu/code/trunk/surgical/DiscreteRods; make clean; make clean; cd LearnParams; make clean; make clean; cd ..; make clean; make; cd LearnParams; make clean; make;"

BINARY = "./try_energy_params"

SCP_CMD = "scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -i /home/ubuntu/.ssh/sameep-surgical.pem /home/ubuntu/code/trunk/surgical/DiscreteRods/LearnParams/results/* %s:~/rll/code/trunk/surgical/DiscreteRods/LearnParams/results/;" % (USER_HOST)
def frange(start,end,step):
  return map(lambda x: x*step, range(int(start*1./step),int(end*1./step)))

if __name__ == "__main__":
  param_file = sys.argv[ARG_PARAM_FILE]
  eval_mode = sys.argv[ARG_EVAL_MODE]
  min_twist = float(sys.argv[ARG_MIN_TWIST])
  iter_twist = float(sys.argv[ARG_ITER_TWIST])
  max_twist = float(sys.argv[ARG_MAX_TWIST])
  min_grav = float(sys.argv[ARG_MIN_GRAV])
  iter_grav = float(sys.argv[ARG_ITER_GRAV])
  max_grav = float(sys.argv[ARG_MAX_GRAV])

  print param_file, min_twist, iter_twist, max_twist, min_grav, iter_grav, max_grav

  
  messenger = MessageHandler(HOST, PORT)
  i=0
  for twist in frange(min_twist, max_twist+20, 20):
    BINARY_CMD = ("%s %s %s %s %s %s %s %s %s;" % 
    (BINARY, param_file, eval_mode, twist, iter_twist, min(twist+20-iter_twist/2, max_twist), min_grav, iter_grav, max_grav))
    job = "%s %s %s" % (COMPILE_CMD, BINARY_CMD, SCP_CMD)
    print job
    data = "USER:JOB:" + job
    messenger.sendMessage(data)
    time.sleep(0.1)
    i += 1
  
  messenger.close()
  print i
  

  """
  BINARY_CMD = ("%s %s %s %s %s %s %s %s %s;" % 
  (BINARY, param_file, eval_mode, min_twist, iter_twist, max_twist, min_grav, iter_grav, max_grav))
  job = "%s %s %s" % (COMPILE_CMD, BINARY_CMD, SCP_CMD)
  print job

  data = "USER:JOB:" + job
  messenger = MessageHandler(HOST, PORT)
  messenger.sendMessage(data)
  messenger.close()

  """
