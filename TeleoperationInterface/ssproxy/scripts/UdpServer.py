import select
import socket
import time
from UdpClient import UdpClient

class UdpServer:
    BUFFER_SIZE = 0x0000FF00
    MAX_RECV_BUFFER_SIZE = 8

    def __init__( self, listenPort = 9560, replyPort = 0, name = None ):
        self._name = name if name is not None else "UdpServer"
        self._clients = dict( )
        self._isRunning = True
        self._communicationWaitTime = 10
        self._maxReceiveTaskCount = 3
        self._msgRecved = list( )
        self._listenPort = max( 1, min( 65535, listenPort ) )
        self._replyPort = replyPort if replyPort > 0 else self._listenPort
        self._sckServer = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
        self._sckServer.bind( ("0.0.0.0", self._listenPort) )
        self._sckList = [ self._sckServer ]
    #END __init__( )

    def __del__( self ):
        self._isRunning = False
        for sock in self._sckList:
            sock.close( )
        self._srvSocket = None
        self._clients.clear( )
        del self._msgRecved[:]
        del self._sckList[:]
    #END __del__( )

    def __len__( self ):
        return len( self._clients )
    #END __len__( )

    def isRunning( self ):
        return self._isRunning
    #END isRunning( )

    def broadcast( self, packet ):
        for key in self._clients:
            self._clients[key].send( packet )
    #END broadcast( )

    def getCommunicationWaitTime( self ):
        return self._communicationWaitTime
    #END getCommunicationWaitTime( )

    def setCommunicationWaitTime( self, seconds ):
        self._communicationWaitTime = max( 1, seconds )
    #END setCommunicationWaitTime( )

    def getMaxReceiveTaskCount( self ):
        return self._maxReceiveTaskCount
    #END getMaxReceiveTaskCount( )

    def setMaxReceiveTaskCount( self, count ):
        self._maxReceiveTaskCount = count
    #END setMaxReceiveTaskCount( )

    def receive( self ):
        if len( self._msgRecved ) > 0:
            pair = self._msgRecved.pop( 0 )
            return pair[0], pair[1]
        return None, None
    #END receive( )

    def update( self ):
        if self._isRunning:
            readable, writable, errors = select.select( self._sckList, self._sckList, self._sckList, 0.01 )
            self._updateReadable( readable )
            self._updateWritable( writable )
            self._updateErrors( errors )

        clientsToRemove = list( )
        for key in self._clients:
            if not self._clients[key].update( ) or time.time( ) - self._clients[key].getCommunicationTimestamp( ) >= self._communicationWaitTime:
                clientsToRemove.append( key )
        for key in clientsToRemove:
            del self._clients[key]
    #END update( )

    def _updateReadable( self, readable ):
        for sck in readable:
            data, address = sck.recvfrom( UdpServer.BUFFER_SIZE )
            address = str(address[0]) # ip address only
            if address in self._clients:
                client = self._clients[address]
            else:
                client = UdpClient( address, self._replyPort, self._name )
                client.setMaxReceiveTaskCount( self._maxReceiveTaskCount )
                self._clients[address] = client
                print( "[%s] connection from %s" % (self._name, client.address( )) )
            client.setCommunicationTimestamp( time.time( ) )
            if client.onReceiveData( data ):
                packet = client.receive( )
                while packet is not None:
                    self._msgRecved.append( (packet, client) )
                    if len( self._msgRecved ) > UdpServer.MAX_RECV_BUFFER_SIZE:
                        self._msgRecved.pop( 0 )
                    packet = client.receive( )
    #END _updateReadable( )

    def _updateWritable( self, writable ):
        pass
    #END _updateWritable( )

    def _updateErrors( self, errors ):
        for sck in errors:
            print( "[%s] error with the server socket" % self._name )
            self._isRunning = False
    #END _updateErrors( )
#END UdpServer
