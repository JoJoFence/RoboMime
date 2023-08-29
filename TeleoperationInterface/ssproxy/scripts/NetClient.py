import select
import struct
import socket

class NetClient:
    PACKET_MAXIMUM_SIZE     = 0x3FFFFFFF
    PACKET_MINIMUM_SIZE     = 4
    PACKET_NORMAL           = 0X00
    PACKET_CHANGECONNECTION = 0X02
    PACKET_DISCONNECTION    = 0X03

    def __init__( self, sock = None, addr = "" ):
        self._address = addr
        self._socket = sock
        self._isValid = self._socket is not None
        self._msgRecved = list( )
        self._msgToSend = list( )
        self._recvBuffer = bytearray( )
        if self._isValid:
            self._sckList = [self._socket]
        self._sendBytes = 0
    #END __init__( )

    def __del__( self ):
        self.disconnect( False )
    #END __del__( )

    def address( self ):
        return self._address
    #END address( )

    def connect( self, hostAddress, hostPort, moduleName ):
        if self._socket is None:
            self._address = ( hostAddress, hostPort )
            self._socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
            self._socket.connect( self._address )
            self._isValid = True
            self._sckList = [self._socket]
            header = NetClient.pckCreateHeader( len( moduleName ), NetClient.PACKET_CHANGECONNECTION )
            self._msgToSend.append( header + moduleName.encode( ) )
    #END connect( )

    def disconnect( self, bSendClosingMessage ):
        if self._socket is not None:
            if bSendClosingMessage:
                try: # send disconnect message
                    data = NetClient.pckCreateHeader( 0 )
                    self._socket.sendall( data )
                except:
                    pass
            self._socket.close( )
            self._socket = None
        self._isValid = False
        del self._msgRecved[:]
        del self._msgToSend[:]
        self._recvBuffer = bytearray( )
        self._sckList = []
        self._sendBytes = 0
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
        header = NetClient.pckCreateHeader( len( data ), NetClient.PACKET_NORMAL )
        self._msgToSend.append( header + data )
    #END send( )

    def update( self ):
        if self._isValid:
            readable, writable, exceptions = select.select( self._sckList, self._sckList, self._sckList, 0.001 )
            self._updateReadable( readable )
            self._updateWritable( writable )
            self._updateExceptions( exceptions )
        return self._isValid
    #END update( )

    def _processReceivedData( self ):
        while self._isValid and len( self._recvBuffer ) >= NetClient.PACKET_MINIMUM_SIZE:
            type, length = NetClient.pckParseHeader( self._recvBuffer )
            if length < NetClient.PACKET_MINIMUM_SIZE or length > NetClient.PACKET_MAXIMUM_SIZE:
                self._recvBuffer = bytearray( ) # incorrect length information => discard all
            elif length > len( self._recvBuffer ):
                return # we need more data
            elif type == NetClient.PACKET_NORMAL:
                self._msgRecved.append( self._recvBuffer[NetClient.PACKET_MINIMUM_SIZE:length] )
                self._recvBuffer = self._recvBuffer[length:]
            elif type == NetClient.PACKET_CHANGECONNECTION:
                self._isValid = False # not expected
            elif type == NetClient.PACKET_DISCONNECTION:
                self._isValid = False
    #END _processReceivedData( )

    def _updateReadable( self, readable ):
        if len( readable ) > 0:
            try:
                data = self._socket.recv( 1024 * 1024 )
            except:
                self._isValid = False
                data = None
            if data is not None:
                self._recvBuffer += data
                self._processReceivedData( )
    #END _updateReadable( )

    def _updateWritable( self, writable ):
        if len( writable ) > 0 and len( self._msgToSend ) > 0:
            try:
                nBytes = self._socket.send( self._msgToSend[0][self._sendBytes:] )
            except:
                nBytes = 0
            if nBytes > 0:
                self._sendBytes += nBytes
                if self._sendBytes >= len( self._msgToSend[0] ):
                    self._msgToSend.pop( 0 )
                    self._sendBytes = 0
            else:
                self._isValid = False
                print( "[NetClient] failed to send data to %s" % str(self._address) )
    #END _updateWritable( )

    def _updateExceptions( self, exceptions ):
        if len( exceptions ) > 0:
            self._isValid = False
    #END _updateExceptions( )

    @staticmethod
    def pckCreateHeader( length, mode ):
        length = length + NetClient.PACKET_MINIMUM_SIZE
        return struct.pack( "<I", (mode << 30) | (length & NetClient.PACKET_MAXIMUM_SIZE) )
    #END pckCreateHeader( )

    @staticmethod
    def pckParseHeader( data ):
        if len( data ) >= NetClient.PACKET_MINIMUM_SIZE:
            value = struct.unpack( "<I", data[:NetClient.PACKET_MINIMUM_SIZE] )[0]
            type = (value & (~NetClient.PACKET_MAXIMUM_SIZE)) >> 30
            length = value & NetClient.PACKET_MAXIMUM_SIZE
            return ( type, length )
        return ( NetClient.PACKET_DISCONNECTION, 0 )
    #END pckParseHeader( )
#END NetClient
