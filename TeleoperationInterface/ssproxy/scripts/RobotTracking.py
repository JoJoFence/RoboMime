#!/usr/bin/python
import argparse
import rospy
import signal
import struct
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from Configuration import ConfigurationSection
from Serialization import Serialization
from UdpServer import UdpServer

class RobotTracking:
    DEFAULT_UDP_IDENTIFIER  = 0x762E2A6F
    DEFAULT_UDP_LISTEN_PORT = 9566
    DEFAULT_UDP_REPLY_PORT  = 9566

    def __init__( self, params = None ):
        self._udpCommunicator = None
        self._udpIdentifier = RobotTracking.DEFAULT_UDP_IDENTIFIER
        self._udpListenPort = RobotTracking.DEFAULT_UDP_LISTEN_PORT
        self._udpReplyPort = RobotTracking.DEFAULT_UDP_REPLY_PORT
        self._tracking = None
        try:
            enable = 0
            rosRobotTrack = "/pose_6d"
            if params is not None:
                enable = int( params.get( 'enable', enable ) )
                rosRobotTrack = str( params.get( 'rosRobotTrack', rosRobotTrack ) )
                self._udpIdentifier = int( params.get( 'udpIdentifier', self._udpIdentifier ) )
                self._udpListenPort = int( params.get( 'udpListenPort', self._udpListenPort ) )
                self._udpReplyPort = int( params.get( 'udpReplyPort', self._udpReplyPort ) )
            if enable != 0:
                rospy.init_node( "SS_RobotTracking", anonymous = True )
                self._tracking = rospy.Subscriber( rosRobotTrack, PoseWithCovarianceStamped, self.onRobotTracked )
                self._udpCommunicator = UdpServer( self._udpListenPort, self._udpReplyPort, "RobotTracking" )
                self._udpCommunicator.setCommunicationWaitTime( 11 )
        except Exception as ex:
            print( "[RobotTracking] Error: %s" % ex )
    #END __init__( )

    def __del__( self ):
        if self._tracking is not None:
            self._tracking.unregister( )
            self._tracking = None
        self._udpCommunicator = None
    #END __del__( )

    def threadUpdateInterval( self ):
        return 15000 # 15 miliseconds
    #END threadUpdateInterval( )

    def onRobotTracked( self, data ):
        packet = struct.pack( "<I", self._udpIdentifier ) + Serialization.packPoseWithCovarianceStamped( data )
        self._udpCommunicator.broadcast( packet )
    #END onRobotTracked( )

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
#END RobotTracking


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
    parser.add_argument( "-t", "--topic", dest="rosRobotTrack", default="/pose_6d", type=str, help="ROS robot pose topic name" )
    parser.add_argument( "-ui", "--udp-identifier",  dest="udpIdentifier", default=RobotTracking.DEFAULT_UDP_IDENTIFIER,  type=int, help="Connection identifier for the UDP communication" )
    parser.add_argument( "-up", "--udp-listen-port", dest="udpListenPort", default=RobotTracking.DEFAULT_UDP_LISTEN_PORT, type=int, help="Listening port for the UDP communication" )
    parser.add_argument( "-ur", "--udp-reply-port",  dest="udpReplyPort",  default=RobotTracking.DEFAULT_UDP_REPLY_PORT,  type=int, help="Reply port for the UDP communication" )
    args = parser.parse_args( rospy.myargv( )[1:] )

    params = ConfigurationSection( )
    params.add( 'enable', 1 )
    params.add( 'rosRobotTrack', args.rosRobotTrack )
    params.add( 'udpIdentifier', args.udpIdentifier )
    params.add( 'udpListenPort', args.udpListenPort )
    params.add( 'udpReplyPort', args.udpReplyPort )

    handler = RobotTracking( params )
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
