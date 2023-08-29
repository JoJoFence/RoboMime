import select
import socket
from PacketManager import PacketManager

class UdpClient:
    MAX_SEND_BUFFER_SIZE = 128

    def __init__( self, address, port, name = None ):
        self._name = name if name is not None else "UdpClient"
        self._communicationTimestamp = 0
        self._hostAddress = str(address)
        self._hostPort = max( 1, min( 65535, port ) )
        self._address = self._hostAddress + ":" + str(self._hostPort)
        self._msgRecved = list( )
        self._msgToSend = list( )
        self._pckmgr = PacketManager( )
        self._socket = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
        self._sckList = [ self._socket ]
        self._isValid = True
    #END __init__( )

    def __del__( self ):
        self.disconnect( )
    #END __del__( )

    def address( self ):
        return self._address
    #END address( )

    def getCommunicationTimestamp( self ):
        return self._communicationTimestamp
    #END getCommunicationTimestamp( )

    def setCommunicationTimestamp( self, time ):
        self._communicationTimestamp = time
    #END setCommunicationTimestamp( )

    def getMaxReceiveTaskCount( self ):
        return self._pckmgr.getMaxReceiveTaskCount( )
    #END getMaxReceiveTaskCount( )

    def setMaxReceiveTaskCount( self, count ):
        self._pckmgr.setMaxReceiveTaskCount( count )
    #END setMaxReceiveTaskCount( )

    def disconnect( self ):
        self._isValid = False
        self._sckList = [ ]
        if self._socket is not None:
            self._socket.close( )
            self._socket = None
        del self._msgRecved[:]
        del self._msgToSend[:]
        print( "[%s] disconnected from %s" % (self._name, self._address) )
    #END disconnect( )

    def isValid( self ):
        return self._isValid
    #END isValid( )

    def onReceiveData( self, data ):
        newrecv = False
        if self._pckmgr.merge( data ):
            while self._pckmgr.hasReceivedPacket( ):
                packet, complete = self._pckmgr.getReceivedPacket( )
                self._msgRecved.append( packet )
                newrecv = True
        return newrecv
    #END onReceiveData( )

    def receive( self ):
        if len( self._msgRecved ) > 0:
            return self._msgRecved.pop( 0 )
        return None
    #END receive( )

    def send( self, data ):
        data = self._pckmgr.split( data )
        if len( self._msgToSend ) + len( data ) <= UdpClient.MAX_SEND_BUFFER_SIZE:
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
        return False
    #END update( )

    def _updateReadable( self, readable ):
        pass
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
#END UdpClient
