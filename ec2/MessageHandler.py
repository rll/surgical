import socket
import urllib


def get_my_ip_address():
  whatismyip = 'http://www.whatismyip.com/automation/n09230945.asp'
  return urllib.urlopen(whatismyip).readlines()[0]

class MessageHandler():
  def __init__(self, HOST, PORT):
    self.data = ""
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.connect((HOST,PORT))
  def sendMessage(self, msg):
    self.sock.send(msg+"\n")
  def exchangeMessage(self, msg):
    self.sendMessage(msg)
    received = ""
    while "\n" not in received:
      received = self.sock.recv(1024)
    return received.strip()
  def close(self):
    self.sock.close()

  
