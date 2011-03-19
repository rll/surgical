import SocketServer, threading, socket, commands, sys
from logger import EC2Logger
from MessageHandler import MessageHandler, get_my_ip_address
from Queue import Queue
from EC2Tools import EC2Tools
from threading import Lock
from subprocess import Popen, PIPE, STDOUT

"""
threaded_server.py <port> 
This is a server instance. It interacts with the user and the instances on EC2. 
"""
MASTER_IP = "128.32.37.215"
MASTER_PORT = 10000
MAX_NUM_INSTANCES = 200

SSHCMD = "ssh -i sameep-surgical.pem -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ubuntu@"
SVNCMD = " 'cd /home/ubuntu/code/trunk/surgical; svn up --config-option config:tunnels:ssh=\"ssh -a -i /home/ubuntu/.ssh/id_rsa_st\"'"

logger = EC2Logger("MASTER")
toolkit = EC2Tools()
workers = { } 
job_list = [ ]
job_lock = Lock()

class Master(SocketServer.StreamRequestHandler):
  def run_command(self, cmd, instance):
    id, hostname = instance.split(',')
    logger.info("[ID: %s, ubuntu@%s] Running command: %s" % 
        (id, hostname, cmd))
    logger.info("[ID: %s, ubuntu@%s] %s" %
        (id, hostname, commands.getoutput(cmd)))

  def run_command_untested(self, cmd, instance):
    id, hostname = instance.split(',')
    logger.info("[ID: %s, ubuntu@%s] Running command: %s" % 
        (id, hostname, cmd))
    process = Popen(cmd, stdout=PIPE, stdin=PIPE, stderr=STDOUT, shell=True)
    data = ""
    while process.poll() is None:
      data += process.stdout.read()
      msgs = data.split('\n')
      for m in msgs[0:-1]:
        logger.info("[ID: %s, ubuntu@%s] %s" % (id, hostname, m))
      data = msgs[-1]
    if data != '':
      logger.info("[ID: %s, ubuntu@%s] %s" % (id, hostname, data))
    logger.info("[ID: %s, ubuntu@%s] Command Complete" % (id, hostname, cmd))
    
  def handle_instance(self, instance, msg_type, msg, first_call=True):
    #uninitialized instances communicating for orders
    global job_list
    global job_lock
    id, hostname = instance.split(',')
    if first_call:
      logger.info("Handling instance. Updating codebase")
      logger.info(SSHCMD + hostname + SVNCMD)
      self.run_command(SSHCMD + hostname + SVNCMD, instance)
    jobSet = False
    try:
      job_lock.acquire()
      if len(job_list) == 0:
        logger.info("Instance not needed; terminating")
        toolkit.terminateInstance(id)
      else:
        job = job_list[0]
        job_list = job_list[1:]
        jobSet = True
    finally:
        job_lock.release()
        if jobSet:
          job = job.replace('\'', '"')
          self.run_command((SSHCMD + hostname + " '%s'") % (job), instance)
          self.handle_instance(instance, msg_type, msg, first_call=False)

  def handle_user(self, instance, msg_type, msg):
    global job_list
    global job_lock
    if msg_type == "JOB":
      try:
        job_lock.acquire()
        job_list.append(msg)
      finally:
        job_lock.release()
    if msg_type == "PRINT_JOBS":
      try:
        job_lock.acquire()
        logger.info("Job list = %s, len(job_list) = %s" % (job_list, len(job_list)))
      finally:
        job_lock.release()
    if msg_type == "RUN_INSTANCE":
      msg = msg.split(' ')
      inst_run = int(msg[0])
      if len(msg) > 1:
        inst_tier = msg[1]
      else:
        inst_tier = toolkit.tier
      while toolkit.num_instances < MAX_NUM_INSTANCES and inst_run > 0:
        toolkit.runInstance(tier=inst_tier)
        inst_run = inst_run - 1
  
  def handle(self):
    while 1:
      self.data = self.rfile.readline().strip()
      if not self.data:
        return
      tokens = self.data.split(':')
      instance_id = tokens[0]
      msg_type = tokens[1]
      msg = ":".join(tokens[2:])
      logger.info("%s %s %s" % (instance_id, msg_type, msg))
      if instance_id == "USER":
        self.handle_user(instance_id, msg_type, msg)
      else:
        self.handle_instance(instance_id, msg_type, msg)
#server host is a tuple ('host', port)
if __name__ == "__main__":
  SocketServer.ThreadingTCPServer.allow_reuse_address = True
  server = SocketServer.ThreadingTCPServer((MASTER_IP, MASTER_PORT),
                                          Master)
  logger.info("Server started at %s:%s" % (MASTER_IP, MASTER_PORT))
  server.serve_forever()
