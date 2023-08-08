import math
import json
import select
import struct
import socket

class RobotMotionUpdater:
    NUM_JOINTS = 11

    def __init__( self ):
        self._isValid = False
        self._address = ""
        self._socket = None
        self._sckList = [ ]
        self._recvBuffer = bytearray( )
        self._msgToSend = list( )
        self._sendBytes = 0
        self._jointAngles = None # robot's joint angles, this is updated when sendPoseRequest() is called and the result is back
        self._jointCvtGain = [ 180.0/5.0, 75.0/5.0, 90.0/5.0, 77.0/5.0, 180.0/5.0, 75.0/5.0, 90.0/5.0, 77.0/5.0, 90.0/5.0, 40.0/5.0, 80.0/5.0 ]
        self._jointCvtOffset = [ 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 ]
        try:
            f = open( "configuration.json" )
            js = json.load( f )
            jointArray = js["Joints"]
            for i, joint in enumerate( jointArray ):
                self._jointCvtGain[i] = joint["ConversionGainNumerator"] / joint["ConversionGainDenominator"]
                self._jointCvtOffset[i] = joint["ConversionOffset"]
            f.close( )
        except:
            pass
    #END __init__( )

    def __del__( self ):
        self.disconnect( )
    #END __del__( )

    def address( self ):
        return self._address
    #END address( )

    def connect( self, hostAddress, hostPort = 11000 ):
        if self._socket is None:
            self._address = ( hostAddress, hostPort )
            self._socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
            self._socket.connect( self._address )
            self._isValid = True
            self._sckList = [self._socket]
    #END connect( )

    def disconnect( self ):
        if self._socket is not None:
            self._socket.close( )
            self._socket = None
        self._isValid = False
        del self._msgToSend[:]
        self._recvBuffer = bytearray( )
        self._sckList = []
        self._sendBytes = 0
    #END disconnect( )

    def isValid( self ):
        return self._isValid
    #END isValid( )

    def getAngle( self ):
        angles = self._jointAngles
        self._jointAngles = None
        return angles
    #END getAngle( )

    def setAngle( self, shoulder_pitch_r = float('NaN'), shoulder_roll_r = float('NaN'), elbow_yaw_r = float('NaN'), elbow_pitch_r = float('NaN'), \
                        shoulder_pitch_l = float('NaN'), shoulder_roll_l = float('NaN'), elbow_yaw_l = float('NaN'), elbow_pitch_l = float('NaN'), \
                        neck_yaw = float('NaN'), head_roll = float('NaN'), head_pitch = float('NaN'), duration = 1.0 ):
        angles = [ shoulder_pitch_r, shoulder_roll_r, elbow_yaw_r, elbow_pitch_r, shoulder_pitch_l, shoulder_roll_l, elbow_yaw_l, elbow_pitch_l, neck_yaw, head_roll, head_pitch ]
        poses = self.convertToPoseValue( angles )
        self.setPose( poses, duration, True )
    #END setAngle( )

    def convertToAngle( self, poses ):
        angles = [ ]
        for i, pose in enumerate( poses ):
            if not math.isnan( pose ) and not math.isinf( pose ):
                angles.append( pose * self._jointCvtGain[i] + self._jointCvtOffset[i] )
            else:
                angles.append( float('NaN') )
        return angles
    #END convertToAngle( )

    def convertToPoseValue( self, angles ):
        poses = [ ]
        for i, angle in enumerate( angles ):
            if not math.isnan( angle ) and not math.isinf( angle ):
                poses.append( (angle - self._jointCvtOffset[i]) / self._jointCvtGain[i] )
            else:
                poses.append( float('NaN') )
        return poses
    #END convertToPoseValue( )

    def sendPoseRequest( self ):
        self._sendstring( "GET POSE" )
    #END sendPoseRequest( )

    def setPose( self, poses, duration = 1.0, priority = True ):
        enable = 0x00000000
        mask = 0x000000001
        message = "SET UPPER_BODY_AXIS " + "1" if priority else "0"
        for pose in poses:
            if not math.isnan( pose ) and not math.isinf( pose ):
                message += " " + str(int( pose * 1000 ))
                enable = enable | mask
            else:
                message += " 0"
            mask = mask << 1
        message += " " + str(int( duration * 1000 )) + " " + str(enable)
        self._sendstring( message )
        print( message )
    #END setPose( )

    def update( self ):
        readable, writable, exceptions = select.select( self._sckList, self._sckList, self._sckList, 0.001 )
        self._updateReadable( readable )
        self._updateWritable( writable )
        self._updateExceptions( exceptions )
        return self._isValid
    #END update( )

    def _updateReadable( self, readable ):
        if len( readable ) > 0:
            try:
                data = self._socket.recv( 1024 * 1024 )
            except:
                self._isValid = False
                data = None
            while data is not None:
                index = data.find( b'\n' )
                if index >= 0 and data[index - 1] == ord('\r'): # \r\n
                    msg = (self._recvBuffer + data[:index + 1]).decode( 'ascii' )
                    data = data[index + 1:]
                    self._recvBuffer = bytearray( )
                    if msg.startswith( "401" ):
                        self._sendstring( "AUTH ROS_COMMAND" )
                    elif msg.startswith( "201" ):
                        pass
                    elif msg.startswith( "294" ):
                        self._parsePose( msg )
                    else:
                        print( "Unhandled message: %s" % msg )
                else:
                    self._recvBuffer += data
                    data = None
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

    def _parsePose( self, message ):
        poses = [ 0.0 ] * RobotMotionUpdater.NUM_JOINTS
        tokens = message.split( ' ' )[1:]
        for i, value in enumerate( tokens ):
            if (i % 2) == 1:
                index = int(tokens[i - 1])
                poses[index] = float(value) / 1000.0
        self._jointAngles = tuple( self.convertToAngle( poses ) )
    #END _parsePose( )

    def _sendstring( self, message ):
        data = (message + "\r\n").encode( "ascii" )
        self._msgToSend.append( data )
    #END _sendstring( )
#END RobotMotionUpdater



import time
if __name__ == '__main__':
    updateInterval = 100000 # 100 ms
    connection = RobotMotionUpdater( )
    connection.connect( "192.168.128.143" )
    lastUpdated = time.time( ) * 1000000.0
    while connection.isValid( ):
        now = time.time( ) * 1000000.0
        elapsed = now - lastUpdated
        diff = updateInterval - elapsed
        if diff <= 0.0:
            lastUpdated = now
            angles = connection.getAngle( )
            if angles is not None:
                print( angles )
            #connection.sendPoseRequest( )
            #connection.setAngle( 100 )
            connection.update( )
            time.sleep( 0 )
        else:
            time.sleep( diff / 4000000 )
#END if-else

