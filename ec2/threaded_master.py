import SocketServer, threading, socket, commands, sys
from logger import EC2Logger
from MessageHandler import MessageHandler
from Queue import Queue
from EC2Tools import EC2Tools
from threading import Lock


"""
threaded_master.py <port> 
This is an master instance. It interacts with the user and the instances on EC2. 
"""

MASTER_IP = "128.32.37.215" 
MASTER_PORT = 10000
MAX_NUM_INSTANCES = 50
WORKERS_TO_AGGREGATORS = 10
AGGREGATOR_PORT = 10000
MAX_JOBS_QUEUE_PER_INST = 1


SSHCMD = "ssh -i sameep-surgical.pem -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ubuntu@"
SVNCMD = " 'cd /home/ubuntu/code/trunk/surgical; svn up --config-option config:tunnels:ssh=\"ssh -a -i /home/ubuntu/.ssh/id_rsa_st\"'"

logger = EC2Logger("MASTER")
toolkit = EC2Tools()
jobs = Queue()
workers = { } 
aggregators = [ ]
job_list = [ ]
create_aggregator = True
job_lock = Lock()
class Master(SocketServer.StreamRequestHandler):
  def handle_instance(self, instance, msg_type, msg):
    #uninitialized instances communicating for orders
    global job_list
    global create_aggregator
    global job_lock
    id, hostname = instance.split(',')
    if msg_type == "READY":
      logger.info("Handling instance. Updating codebase")
      logger.info(SSHCMD + hostname + SVNCMD)
      logger.info(commands.getoutput(SSHCMD + hostname + SVNCMD))
      try:
        job_lock.acquire()
        if len(job_list) == 0 and not create_aggregator:
          logger.info("Unneccessary instance was spawned; terminating")
          toolkit.terminateInstance(id)
          pass
        else:
          #see which aggregator is open
          job_assigned = False
          for ag in aggregators:
            if not job_assigned and not create_aggregator and ag['num_workers']<WORKERS_TO_AGGREGATORS:
              for i in range(MAX_JOBS_QUEUE_PER_INST):
                if len(job_list) > 0:
                  job = job_list[0]
                  messenger = MessageHandler(ag['hostname'], ag['port'])
                  msg = messenger.exchangeMessage("MASTER:JOB:" + job)
                  messenger.close()
                  # now the aggregator has the job
                  if msg!= "Job Ack":
                    logger.error("No Job Ack recv. CRITICAL ERROR!")
                  self.wfile.write("WORKER:%s:%s\n" 
                    %(ag['hostname'], ag['port']))
                  logger.info(job + " assigned to Aggregator %s ubuntu@%s" 
                    % (ag['id'], ag['hostname']))
                  logger.info("Worker %s created ubuntu@%s" % (id, hostname))
                  ag['num_workers'] += 1
                  workers[id] = ag['id']
                  job_list = job_list[1:]
                  job_assigned = True
          if not job_assigned or create_aggregator:
            self.wfile.write("AGGREGATOR:%s\n" % (AGGREGATOR_PORT))
            logger.info("Aggregator created: %s ubuntu@%s" % (id, hostname))
            ag = {  }
            create_aggregator = False
            ag['hostname'] = hostname
            ag['id'] = id
            ag['port'] = AGGREGATOR_PORT
            ag['num_workers'] = 0
            aggregators.append(ag)
            #launch an ec2 instance
            toolkit.runInstance()
      finally:
        job_lock.release()
    if msg_type == "JOB_COMPLETE":
      worker_id, worker_job = msg.split(',')
      logger.info("Job Complete: [AGR %s ubuntu@%s] [WORKER  %s] [Job %s]" % (id, hostname, worker_id, worker_job)) 
    if msg_type == "JOB_ASSIGN":
      worker_id, worker_job = msg.split(',')
      logger.info("Job Assign: [AGR %s ubuntu@%s] [WORKER  %s] [Job %s]" % (id, hostname, worker_id, worker_job)) 
    if msg_type == "DATA":
      logger.info(msg)
    if msg_type == "FINISH":
      for ag in aggregators:
        if ag['id'] == workers[id]:
          ag['num_workers'] -= 1
          workers[id] = None
          logger.info("Command finished, terminating instance " + instance)
          toolkit.terminateInstance(id)
      try:
        job_lock.acquire()
        if toolkit.num_instances < MAX_NUM_INSTANCES and len(job_list) > 0:
          toolkit.runInstance()  
      finally:
        job_lock.release()

  def handle_user(self, instance, msg_type, msg):
    global job_list
    global job_lock
    try:
      job_lock.acquire()
      if msg_type == "JOB":
        job_list.append(msg)
        if toolkit.num_instances < MAX_NUM_INSTANCES:
          if len(job_list) >= MAX_JOBS_QUEUE_PER_INST:
            toolkit.runInstance()
          else:
            noWorkers = True
            for ag in aggregators:
              if ag['num_workers'] != 0:
                noWorkers = False
            if noWorkers:
              toolkit.runInstance()
    finally:
      job_lock.release()
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
  toolkit.runInstance()
  server.serve_forever()
