import SocketServer, commands, socket, time
from subprocess import Popen, PIPE, STDOUT
from logger import EC2Logger
from MessageHandler import MessageHandler


HOST = "128.32.37.215"
PORT = 10000
INSTANCE_ID = commands.getoutput("wget -q -O - http://169.254.169.254/latest/meta-data/instance-id")
PUBLIC_HOSTNAME = commands.getoutput("wget -q -O - http://169.254.169.254/latest/meta-data/public-hostname")

AGGREGATOR_CMD = "python /home/ubuntu/code/trunk/surgical/ec2/threaded_aggregator.py "
WORKER_CMD = "python /home/ubuntu/code/trunk/surgical/ec2/worker.py "

logger = EC2Logger("CFG-" + INSTANCE_ID)
messenger = MessageHandler(HOST, PORT)
data = INSTANCE_ID + "," + PUBLIC_HOSTNAME
logger.info(data+":READY")
msg = messenger.exchangeMessage(data+":READY")
messenger.close()
msg = msg.split(":")

if msg[0] == "AGGREGATOR":
  logger.info("Starting Aggregator Service")
  process = Popen("%s %s" % (AGGREGATOR_CMD, msg[1]), shell=True)
elif msg[0] == "WORKER":
  logger.info("Starting Worker Service")
  process = Popen("%s %s %s" % (WORKER_CMD, msg[1], msg[2]), shell=True)

while process.poll() is None:
  time.sleep(15)

messenger = MessageHandler(HOST, PORT)
messenger.sendMessage(data+":FINISH")
messenger.close()
