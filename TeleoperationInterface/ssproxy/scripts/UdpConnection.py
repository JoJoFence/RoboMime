import select
import socket
from PacketManager import PacketManager

class UdpConnection:
    BUFFER_SIZE = 0x0000FF00
    MAX_RECV_BUFFER_SIZE = 8
    MAX_SEND_BUFFER_SIZE = 128

    def __init__( self, name = None ):
        self._name = name if name is not None else "UdpConnection"
        self._address = "127.0.0.1:9560"
        self._hostAddress = "127.0.0.1"
        self._hostPort = 9560
        self._listenPort = 9560
        self._msgRecved = list( )
        self._msgToSend = list( )
        self._pckmgr = PacketManager( )
        self._socket = None
        self._sckList = [ ]
        self._isValid = False
    #END __init__( )

    def __del__( self ):
        self.disconnect( )
    #END __del__( )

    def address( self ):
        return self._address
    #END address( )

    def getMaxReceiveTaskCount( self ):
        return self._pckmgr.getMaxReceiveTaskCount( )
    #END getMaxReceiveTaskCount( )

    def setMaxReceiveTaskCount( self, count ):
        self._pckmgr.setMaxReceiveTaskCount( count )
    #END setMaxReceiveTaskCount( )

    def connect( self, hostAddress, hostPort, listenPort = 0 ):
        if self._socket is None:
            self._hostAddress = str(hostAddress)
            self._hostPort = max( 1, min( 65535, hostPort ) )
            self._listenPort = listenPort if listenPort > 0 else self._hostPort
            self._address = self._hostAddress + ":" + str(self._hostPort)
            self._socket = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
            self._socket.bind( ("0.0.0.0", self._listenPort) )
            self._sckList = [ self._socket ]
            self._isValid = True
    #END connect( )

    def disconnect( self ):
        self._isValid = False
        self._sckList = [ ]
        if self._socket is not None:
            self._socket.close( )
            self._socket = None
        del self._msgRecved[:]
        del self._msgToSend[:]
    #END disconnect( )

    def isValid( self ):
        return self._isValid
    #END isValid( )

    def receive( self ):
        if len( self._msgRecved ) > 0:
            return self._msgRecved.pop( 0 )
        return None
    #END receive( )

    def send( self, data ):
        data = self._pckmgr.split( data )
        if len( self._msgToSend ) + len( data ) <= UdpConnection.MAX_SEND_BUFFER_SIZE:
            for piece in data:
                self._msgToSend.append( piece )
    #END send( )

    def update( self ):
        if self._isValid:
            readable, writable, exceptions = select.select( self._sckList, self._sckList, self._sckList, 0.001 )
            self._updateReadable( readable )
            self._updateWritable( writable )
            self._updateExceptions( exceptions )
        return self._isValid
    #END update( )

    def _updateReadable( self, readable ):
        for sck in readable:
            data, address = sck.recvfrom( UdpConnection.BUFFER_SIZE )
            if self._pckmgr.merge( data ):
                while self._pckmgr.hasReceivedPacket( ):
                    packet, complete = self._pckmgr.getReceivedPacket( )
                    self._msgRecved.append( packet )
                    if len( self._msgRecved ) > UdpConnection.MAX_RECV_BUFFER_SIZE:
                        self._msgRecved.pop( 0 )
    #END _updateReadable( )

    def _updateWritable( self, writable ):
        if len( writable ) > 0 and len( self._msgToSend ) > 0:
            try:
                self._socket.sendto( self._msgToSend.pop( 0 ), (self._hostAddress, self._hostPort) )
            except:
                print( "[%s] failed to send data to %s" % (self._name, self._address) )
                self.disconnect( )
    #END _updateWritable( )

    def _updateExceptions( self, exceptions ):
        if len( exceptions ) > 0:
            self.disconnect( )
    #END _updateExceptions( )
#END UdpConnection
