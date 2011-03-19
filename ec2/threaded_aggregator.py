import SocketServer, threading, socket, commands, sys
from logger import EC2Logger
from MessageHandler import MessageHandler
from threading import Lock


"""
threaded_aggregator.py <port> 
This is an aggregator instance. It gets jobs from the master server and relays them to worker clients
"""

INSTANCE_ID = commands.getoutput("wget -q -O - http://169.254.169.254/latest/meta-data/instance-id")
PUBLIC_HOSTNAME = commands.getoutput("wget -q -O - http://169.254.169.254/latest/meta-data/public-hostname")

AGGREGATOR_IP = PUBLIC_HOSTNAME
AGGREGATOR_PORT = int(sys.argv[1])

MASTER_IP = "128.32.37.215"
MASTER_PORT = 10000

logger = EC2Logger("AGR-" + INSTANCE_ID)
jobs = [ ] 
job_lock = Lock()

class Aggregator(SocketServer.StreamRequestHandler):
  def handle_master(self, instance, msg_type, msg):
    global jobs
    global job_lock
    if msg_type == "JOB":
      try:
        job_lock.acquire()
        jobs.append(msg)
        self.wfile.write("Job Ack\n")
      finally:
        job_lock.release()
    if msg_type == "STATUS":
      self.wfile.write("%s:STATUS:%s" % (INSTANCE_ID, len(jobs)))
  def handle_instance(self, instance, msg_type, msg):
    global jobs
    global job_lock
    if msg_type == "READY":
      try:
        job_lock.acquire()
        if len(jobs) == 0:
          logger.info("No jobs to assign! Terminating %s" % (INSTANCE_ID))
          self.job = "TERMINATE"
        else:
          self.job = jobs[0] + "\n"
          jobs = jobs[1:]
        self.wfile.write(self.job)
        messenger = MessageHandler(MASTER_IP, MASTER_PORT)
        messenger.sendMessage("%s,%s:JOB_ASSIGN:%s,%s" % 
            (INSTANCE_ID,PUBLIC_HOSTNAME,instance,self.job))
        messenger.close()
        logger.info(self.job)
      finally:
        job_lock.release()
    if msg_type == "DATA":
      logger.info(msg)
    if msg_type == "FINISH":
      logger.info("Job complete: " + self.job)
      messenger = MessageHandler(MASTER_IP, MASTER_PORT)
      messenger.sendMessage("%s,%s:JOB_COMPLETE:%s,%s" % 
          (INSTANCE_ID,PUBLIC_HOSTNAME,instance,self.job))
      messenger.close()
  def handle(self):
    while 1:
      self.data = self.rfile.readline().strip()
      if not self.data:
        return
      tokens = self.data.split(':')
      instance_id = tokens[0]
      msg_type = tokens[1]
      msg = ":".join(tokens[2:])
      logger.debug("%s %s %s" % (instance_id, msg_type, msg))
      if instance_id == "MASTER":
        self.handle_master(instance_id, msg_type, msg)
      else:
        self.handle_instance(instance_id, msg_type, msg)
#server host is a tuple ('host', port)
if __name__ == "__main__":
  SocketServer.ThreadingTCPServer.allow_reuse_address = True
  server = SocketServer.ThreadingTCPServer((AGGREGATOR_IP, AGGREGATOR_PORT),
                                          Aggregator)
  logger.info("Server started at %s:%s" % (AGGREGATOR_IP, AGGREGATOR_PORT))
  server.serve_forever()
