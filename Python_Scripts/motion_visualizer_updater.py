import select
import struct
import socket

class MotionVisualizerUpdater:
    MODULE_NAME = "MotionVisualizerJointUpdate"
    PACKET_MAXIMUM_SIZE     = 0x3FFFFFFF
    PACKET_MINIMUM_SIZE     = 4
    PACKET_NORMAL           = 0X00
    PACKET_CHANGECONNECTION = 0X02
    PACKET_DISCONNECTION    = 0X03

    def __init__( self ):
        self._address = ""
        self._socket = None
        self._isValid = False
        self._msgRecved = list( )
        self._msgToSend = list( )
        self._recvBuffer = bytearray( )
        self._sckList = [ ]
        self._sendBytes = 0
    #END __init__( )

    def __del__( self ):
        self.disconnect( False )
    #END __del__( )

    def address( self ):
        return self._address
    #END address( )

    def connect( self, hostAddress, hostPort = 9407 ):
        if self._socket is None:
            self._address = ( hostAddress, hostPort )
            self._socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
            self._socket.connect( self._address )
            self._isValid = True
            self._sckList = [self._socket]
            data = MotionVisualizerUpdater.MODULE_NAME.encode( "utf-8" )
            header = MotionVisualizerUpdater.pckCreateHeader( len( data ), MotionVisualizerUpdater.PACKET_CHANGECONNECTION )
            self._msgToSend.append( header + data )
    #END connect( )

    def disconnect( self, bSendClosingMessage ):
        if self._socket is not None:
            if bSendClosingMessage:
                try: # send disconnect message
                    data = MotionVisualizerUpdater.pckCreateHeader( 0 )
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

    def send( self, shoulder_pitch_r = float('NaN'), shoulder_roll_r = float('NaN'), elbow_yaw_r = float('NaN'), elbow_pitch_r = float('NaN'), \
                          shoulder_pitch_l = float('NaN'), shoulder_roll_l = float('NaN'), elbow_yaw_l = float('NaN'), elbow_pitch_l = float('NaN'), \
                          neck_yaw = float('NaN'), head_roll = float('NaN'), head_pitch = float('NaN') ):
        data = struct.pack( "<Ifffffffffff", 0xFFFFFFFF, shoulder_pitch_r, shoulder_roll_r, elbow_yaw_r, elbow_pitch_r, shoulder_pitch_l, shoulder_roll_l, elbow_yaw_l, elbow_pitch_l, neck_yaw, head_roll, head_pitch )
        header = MotionVisualizerUpdater.pckCreateHeader( len( data ), MotionVisualizerUpdater.PACKET_NORMAL )
        self._msgToSend.append( header + data )
    #END send( )

    def update( self ):
        readable, writable, exceptions = select.select( self._sckList, self._sckList, self._sckList, 0.001 )
        self._updateReadable( readable )
        self._updateWritable( writable )
        self._updateExceptions( exceptions )
        return self._isValid
    #END update( )

    def _processReceivedData( self ):
        while self._isValid and len( self._recvBuffer ) >= MotionVisualizerUpdater.PACKET_MINIMUM_SIZE:
            type, length = MotionVisualizerUpdater.pckParseHeader( self._recvBuffer )
            if length < MotionVisualizerUpdater.PACKET_MINIMUM_SIZE or length > MotionVisualizerUpdater.PACKET_MAXIMUM_SIZE:
                self._recvBuffer = bytearray( ) # incorrect length information => discard all
            elif length > len( self._recvBuffer ):
                return # we need more data
            elif type == MotionVisualizerUpdater.PACKET_NORMAL:
                self._msgRecved.append( self._recvBuffer[MotionVisualizerUpdater.PACKET_MINIMUM_SIZE:length] )
                self._recvBuffer = self._recvBuffer[length:]
            elif type == MotionVisualizerUpdater.PACKET_CHANGECONNECTION:
                self._isValid = False # not expected
            elif type == MotionVisualizerUpdater.PACKET_DISCONNECTION:
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
        length = length + MotionVisualizerUpdater.PACKET_MINIMUM_SIZE
        return struct.pack( "<I", (mode << 30) | (length & MotionVisualizerUpdater.PACKET_MAXIMUM_SIZE) )
    #END pckCreateHeader( )

    @staticmethod
    def pckParseHeader( data ):
        if len( data ) >= MotionVisualizerUpdater.PACKET_MINIMUM_SIZE:
            value = struct.unpack( "<I", data[:MotionVisualizerUpdater.PACKET_MINIMUM_SIZE] )[0]
            type = (value & (~MotionVisualizerUpdater.PACKET_MAXIMUM_SIZE)) >> 30
            length = value & MotionVisualizerUpdater.PACKET_MAXIMUM_SIZE
            return ( type, length )
        return ( MotionVisualizerUpdater.PACKET_DISCONNECTION, 0 )
    #END pckParseHeader( )
#END MotionVisualizerUpdater



import time
if __name__ == '__main__':
    updateInterval = 100000 # 100 ms
    value = 0
    connection = MotionVisualizerUpdater( )
    connection.connect( "127.0.0.1" )
    lastUpdated = time.time( ) * 1000000.0
    while connection.isValid( ):
        now = time.time( ) * 1000000.0
        elapsed = now - lastUpdated
        diff = updateInterval - elapsed
        if diff <= 0.0:
            lastUpdated = now
            connection.send( shoulder_pitch_r=value, shoulder_pitch_l=value )
            value = value + 5
            if value >= 150:
                value = -150
            connection.update( )
            time.sleep( 0 )
        else:
            time.sleep( diff / 4000000 )
#END if-else

