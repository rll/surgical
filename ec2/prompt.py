import SocketServer, commands, socket, time
from subprocess import Popen, PIPE, STDOUT
from logger import EC2Logger
from MessageHandler import MessageHandler


HOST = "128.32.37.215"
PORT = 10000

while 1: 
  command = raw_input("$ ")
  if len(command) == 0 or command[0] == '$':
    tokens = command[1:].split(' ') 
    command = "%s:%s" % (tokens[0], " ".join(tokens[1:]))
  else:
    command = "JOB:" + command
  data = "USER:" + command
  print data
  messenger = MessageHandler(HOST, PORT)
  messenger.sendMessage(data)
  messenger.close()
  time.sleep(0.1)

