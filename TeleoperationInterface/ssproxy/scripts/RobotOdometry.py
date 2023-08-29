#!/usr/bin/python
import argparse
import rospy
import signal
import struct
import time
from nav_msgs.msg import Odometry
from Configuration import ConfigurationSection
from Serialization import Serialization
from UdpServer import UdpServer

class RobotOdometry:
    DEFAULT_UDP_IDENTIFIER  = 0x762C4BC1
    DEFAULT_UDP_LISTEN_PORT = 9565
    DEFAULT_UDP_REPLY_PORT  = 9565

    def __init__( self, params = None ):
        self._udpCommunicator = None
        self._udpIdentifier = RobotOdometry.DEFAULT_UDP_IDENTIFIER
        self._udpListenPort = RobotOdometry.DEFAULT_UDP_LISTEN_PORT
        self._udpReplyPort = RobotOdometry.DEFAULT_UDP_REPLY_PORT
        self._odometry = None
        try:
            enable = 0
            rosOdomName = "/odom"
            if params is not None:
                enable = int( params.get( 'enable', enable ) )
                rosOdomName = str( params.get( 'rosOdomName', rosOdomName ) )
                self._udpIdentifier = int( params.get( 'udpIdentifier', self._udpIdentifier ) )
                self._udpListenPort = int( params.get( 'udpListenPort', self._udpListenPort ) )
                self._udpReplyPort = int( params.get( 'udpReplyPort', self._udpReplyPort ) )
            if enable:
                rospy.init_node( "SS_RobotOdometry", anonymous = True )
                self._odometry = rospy.Subscriber( rosOdomName, Odometry, self.onOdometryReceived )
                self._udpCommunicator = UdpServer( self._udpListenPort, self._udpReplyPort, "RobotOdometry" )
                self._udpCommunicator.setCommunicationWaitTime( 11 )
        except Exception as ex:
            print( "[RobotOdometry] Error: %s" % ex )
    #END __init__( )

    def __del__( self ):
        if self._odometry is not None:
            self._odometry.unregister( )
            self._odometry = None
        self._udpCommunicator = None
    #END __del__( )

    def threadUpdateInterval( self ):
        return 30000 # 30 miliseconds
    #END threadUpdateInterval( )

    def onOdometryReceived( self, data ):
        packet = struct.pack( "<I", self._udpIdentifier ) + Serialization.packOdometry( data )
        self._udpCommunicator.broadcast( packet )
    #END onOdometryReceived( )

    def update( self ):
        if self._udpCommunicator is None:
            return

        self._udpCommunicator.update( )
        packet, client = self._udpCommunicator.receive( )
        while packet is not None:
            if len( packet ) != 4 or struct.unpack( "<I", packet )[0] != self._udpIdentifier:
                # unexpected packet data -> disconnect
                client.disconnect( )
            packet, client = self._udpCommunicator.receive( )
    #END update( )
#END RobotOdometry


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
    parser.add_argument( "-t", "--topic", dest="rosOdomName", default="/odom", type=str, help="ROS odometry topic name" )
    parser.add_argument( "-ui", "--udp-identifier",  dest="udpIdentifier", default=RobotOdometry.DEFAULT_UDP_IDENTIFIER,  type=int, help="Connection identifier for the UDP communication" )
    parser.add_argument( "-up", "--udp-listen-port", dest="udpListenPort", default=RobotOdometry.DEFAULT_UDP_LISTEN_PORT, type=int, help="Listening port for the UDP communication" )
    parser.add_argument( "-ur", "--udp-reply-port",  dest="udpReplyPort",  default=RobotOdometry.DEFAULT_UDP_REPLY_PORT,  type=int, help="Reply port for the UDP communication" )
    args = parser.parse_args( rospy.myargv( )[1:] )

    params = ConfigurationSection( )
    params.add( 'enable', 1 )
    params.add( 'rosOdomName', args.rosOdomName )
    params.add( 'udpIdentifier', args.udpIdentifier )
    params.add( 'udpListenPort', args.udpListenPort )
    params.add( 'udpReplyPort', args.udpReplyPort )

    handler = RobotOdometry( params )
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
