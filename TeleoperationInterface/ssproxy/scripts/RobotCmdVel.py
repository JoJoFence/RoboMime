#!/usr/bin/python
import argparse
import rospy
import signal
import struct
import time
from geometry_msgs.msg import Twist, Vector3
from Configuration import ConfigurationSection
from Serialization import Serialization
from UdpServer import UdpServer

class RobotCmdVel:
    DEFAULT_UDP_IDENTIFIER  = 0x07629960
    DEFAULT_UDP_LISTEN_PORT = 9564
    DEFAULT_UDP_REPLY_PORT  = 9564
    STOP_TIME = 1.0 # seconds to force stop without new commands

    def __init__( self, params = None ):
        self._udpCommunicator = None
        self._udpIdentifier = RobotCmdVel.DEFAULT_UDP_IDENTIFIER
        self._udpListenPort = RobotCmdVel.DEFAULT_UDP_LISTEN_PORT
        self._udpReplyPort = RobotCmdVel.DEFAULT_UDP_REPLY_PORT
        self._control = None
        self._lastCmdVel = Twist( Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) )
        self._lastCmdVelTime = 0
        try:
            enable = 0
            rosCmdVelName = "/cmd_vel"
            if params is not None:
                enable = int( params.get( 'enable', enable ) )
                rosCmdVelName = str( params.get( 'rosCmdVelName', rosCmdVelName ) )
                self._udpIdentifier = int( params.get( 'udpIdentifier', self._udpIdentifier ) )
                self._udpListenPort = int( params.get( 'udpListenPort', self._udpListenPort ) )
                self._udpReplyPort = int( params.get( 'udpReplyPort', self._udpReplyPort ) )
            if enable:
                rospy.init_node( "SS_RobotCmdVel", anonymous = True )
                self._control = rospy.Publisher( rosCmdVelName, Twist, queue_size=1 )
                self._udpCommunicator = UdpServer( self._udpListenPort, self._udpReplyPort, "RobotCmdVel" )
                self._udpCommunicator.setCommunicationWaitTime( 11 )
        except Exception as ex:
            print( "[RobotCmdVel] Error: %s" % ex )
    #END __init__( )

    def __del__( self ):
        if self._control is not None:
            self._control.unregister( )
            self._control = None
        self._udpCommunicator = None
    #END __del__( )

    def threadUpdateInterval( self ):
        return 30000 # 30 miliseconds
    #END threadUpdateInterval( )

    def update( self ):
        if self._udpCommunicator is None:
            return

        twist = self._lastCmdVel

        self._udpCommunicator.update( )
        packet, client = self._udpCommunicator.receive( )
        while packet is not None:
            if len( packet ) < 4 or struct.unpack( "<I", packet[:4] )[0] != self._udpIdentifier:
                # unexpected packet data -> disconnect
                client.disconnect( )
            else:
                packet = packet[4:]
                if len( packet ) > 0:
                    twist = Serialization.unpackTwist( packet )
                    client.send( struct.pack( "<I", self._udpIdentifier ) )
            packet, client = self._udpCommunicator.receive( )

        if len( self._udpCommunicator ) <= 0 or time.time( ) - self._lastCmdVelTime >= RobotCmdVel.STOP_TIME:
            twist = Twist( Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) )

        if twist.linear.x != self._lastCmdVel.linear.x or twist.linear.y != self._lastCmdVel.linear.y or twist.linear.z != self._lastCmdVel.linear.z or \
           twist.angular.x != self._lastCmdVel.angular.x or twist.angular.y != self._lastCmdVel.angular.y or twist.angular.z != self._lastCmdVel.angular.z:
            print( "[RobotCmdVel] move=(%.4f, %.4f, %.4f) turn=(%.4f, %.4f, %.4f)" % (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z) )
            self._control.publish( twist )
            self._lastCmdVel = twist
            self._lastCmdVelTime = time.time( )
    #END update( )
#END RobotCmdVel


def workerSignalHandler( signal, frame ):
    global threadRunning
    threadRunning = False
#END workerSignalHandler( )

def workerMain( ):
    global threadRunning
    threadRunning = True

    signal.signal( signal.SIGABRT, workerSignalHandler )
    signal.signal( signal.SIGHUP, workerSignalHandler )
    signal.signal( signal.SIGINT, workerSignalHandler )
    signal.signal( signal.SIGTERM, workerSignalHandler )

    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser( formatter_class=arg_fmt )
    parser.add_argument( "-t", "--topic", dest="rosCmdVelName", default="/cmd_vel", type=str, help="ROS command velocity topic name" )
    parser.add_argument( "-ui", "--udp-identifier",  dest="udpIdentifier", default=RobotCmdVel.DEFAULT_UDP_IDENTIFIER,  type=int, help="Connection identifier for the UDP communication" )
    parser.add_argument( "-up", "--udp-listen-port", dest="udpListenPort", default=RobotCmdVel.DEFAULT_UDP_LISTEN_PORT, type=int, help="Listening port for the UDP communication" )
    parser.add_argument( "-ur", "--udp-reply-port",  dest="udpReplyPort",  default=RobotCmdVel.DEFAULT_UDP_REPLY_PORT,  type=int, help="Reply port for the UDP communication" )
    args = parser.parse_args( rospy.myargv( )[1:] )

    params = ConfigurationSection( )
    params.add( 'enable', 1 )
    params.add( 'rosCmdVelName', args.rosCmdVelName )
    params.add( 'udpIdentifier', args.udpIdentifier )
    params.add( 'udpListenPort', args.udpListenPort )
    params.add( 'udpReplyPort', args.udpReplyPort )

    handler = RobotCmdVel( params )
    lastUpdated = time.time( ) * 1000000.0
    while threadRunning:
        now = time.time( ) * 1000000.0
        elapsed = now - lastUpdated
        diff = handler.threadUpdateInterval( ) - elapsed
        if diff <= 0.0:
            lastUpdated = now
            handler.update( )
            time.sleep( 0 )
        else:
            time.sleep( diff / 4000000 )
#END workerMain( )

if __name__ == '__main__':
    workerMain( )
#END if-else
