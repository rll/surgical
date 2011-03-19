import SocketServer, commands, socket, time, sys
from subprocess import Popen, PIPE, STDOUT
from MessageHandler import MessageHandler

HOST = sys.argv[1]
PORT = int(sys.argv[2])

INSTANCE_ID = commands.getoutput("wget -q -O - http://169.254.169.254/latest/meta-data/instance-id")

messenger = MessageHandler(HOST, PORT)

keepRunning = True
while keepRunning:  
  data = INSTANCE_ID + ":READY"
  msg = messenger.exchangeMessage(data)
  print msg
  if (msg == "TERMINATE"):
    keepRunning = False
    msg = "echo 'I am terminating'"

  process = Popen(msg, stdout=PIPE, stdin=PIPE, stderr=STDOUT, shell=True)
  data = ""
  while process.poll() is None:
    data += process.stdout.read()
    msgs = data.split('\n')
    for m in msgs[0:-1]:
      print m
      messenger.sendMessage(INSTANCE_ID+":DATA:"+m)
    data = msgs[-1]
    msg = messenger.sendMessage(INSTANCE_ID+":DATA:"+data)
  messenger.sendMessage(INSTANCE_ID+":FINISH")

messenger.close()
