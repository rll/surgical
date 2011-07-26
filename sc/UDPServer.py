import SocketServer
from logger import *
class MyUDPHandler(SocketServer.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        print "%s wrote:" % self.client_address[0]
        print data
        logThingy.info(data)

if __name__ == "__main__":
    HOST, PORT = "128.114.59.150", 36010 
    server = SocketServer.UDPServer((HOST, PORT), MyUDPHandler)
    logThingy = EC2Logger('sc')
    server.serve_forever()

