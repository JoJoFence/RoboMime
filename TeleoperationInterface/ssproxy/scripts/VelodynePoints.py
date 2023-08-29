#!/usr/bin/python
import argparse
import rospy
import signal
import struct
import time
from sensor_msgs.msg import PointCloud2
from Configuration import ConfigurationSection
from Serialization import Serialization
from NetClient import NetClient
from NetServer import NetServer
from UdpServer import UdpServer

class VelodynePoints:
    DEFAULT_UDP_IDENTIFIER  = 0xE11CA263
    DEFAULT_UDP_LISTEN_PORT = 9568
    DEFAULT_UDP_REPLY_PORT  = 9568

    def __init__( self, params = None ):
        self._udpCommunicator = None
        self._udpIdentifier = VelodynePoints.DEFAULT_UDP_IDENTIFIER
        self._udpListenPort = VelodynePoints.DEFAULT_UDP_LISTEN_PORT
        self._udpReplyPort = VelodynePoints.DEFAULT_UDP_REPLY_PORT
        self._clients = list( )
        self._sensor = None
        try:
            enable = 0
            rosVelodynePoints = "/velodyne_points"
            if params is not None:
                enable = int( params.get( 'enable', enable ) )
                rosVelodynePoints = str( params.get( 'rosVelodynePoints', rosVelodynePoints ) )
                self._udpIdentifier = int( params.get( 'udpIdentifier', self._udpIdentifier ) )
                self._udpListenPort = int( params.get( 'udpListenPort', self._udpListenPort ) )
                self._udpReplyPort = int( params.get( 'udpReplyPort', self._udpReplyPort ) )
            if enable != 0:
                rospy.init_node( "SS_VelodynePoints", anonymous = True )
                self._sensor = rospy.Subscriber( rosVelodynePoints, PointCloud2, self.onVelodynePointReceived )
                #self._udpCommunicator = UdpServer( self._udpListenPort, self._udpReplyPort, "VelodynePoints" )
                #self._udpCommunicator.setCommunicationWaitTime( 11 )
        except Exception as ex:
            print( "[VelodynePoints] Error: %s" % ex )
    #END __init__( )

    def __del__( self ):
        if self._sensor is not None:
            self._sensor.unregister( )
            self._sensor = None
        for client in self._clients:
            client.disconnect( True )
            print( "[VelodynePoints] disconnected from %s" % str(client.address( )) )
        del self._clients[:]
        self._udpCommunicator = None
    #END __del__( )

    def addConnection( self, sock, addr ):
        print( "[VelodynePoints] add connection, %s" % str(addr) )
        self._clients.append( NetClient( sock, addr ) )
    #END addConnection( )

    def threadUpdateInterval( self ):
        return 15000 # 15 miliseconds
    #END threadUpdateInterval( )

    def onVelodynePointReceived( self, data ):
        packet = Serialization.packPointCloud2( data )
        for client in self._clients:
            client.send( packet )
        #self._udpCommunicator.broadcast( struct.pack( "<I", self._udpIdentifier ) + packet )
    #END onVelodynePointReceived( )

    def update( self ):
        clientsToRemove = list( )
        for client in self._clients:
            if client.update( ):
                packet = client.receive( )
                while packet is not None:
                    # ignore
                    msg = client.receive( )
            else:
                clientsToRemove.append( client )
        for client in clientsToRemove:
            self._clients.remove( client )
            client.disconnect( False )
            print( "[VelodynePoints] disconnected from %s" % str(client.address( )) )


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
#END VelodynePoints


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
    parser.add_argument( "-t", "--topic", dest="rosVelodynePoints", default="/velodyne_points", type=str, help="ROS velodyne point cloud topic name" )
    parser.add_argument( "-ui", "--udp-identifier",  dest="udpIdentifier", default=VelodynePoints.DEFAULT_UDP_IDENTIFIER,  type=int, help="Connection identifier for the UDP communication" )
    parser.add_argument( "-up", "--udp-listen-port", dest="udpListenPort", default=VelodynePoints.DEFAULT_UDP_LISTEN_PORT, type=int, help="Listening port for the UDP communication" )
    parser.add_argument( "-ur", "--udp-reply-port",  dest="udpReplyPort",  default=VelodynePoints.DEFAULT_UDP_REPLY_PORT,  type=int, help="Reply port for the UDP communication" )
    args = parser.parse_args( rospy.myargv( )[1:] )

    params = ConfigurationSection( )
    params.add( 'enable', 1 )
    params.add( 'rosVelodynePoints', args.rosVelodynePoints )
    params.add( 'udpIdentifier', args.udpIdentifier )
    params.add( 'udpListenPort', args.udpListenPort )
    params.add( 'udpReplyPort', args.udpReplyPort )

    server = NetServer( args.udpListenPort )
    handler = VelodynePoints( params )
    lastUpdated = time.time( ) * 1000000.0
    while threadRunning:
        now = time.time( ) * 1000000.0
        elapsed = now - lastUpdated
        diff = handler.threadUpdateInterval( ) - elapsed
        if diff <= 0.0:
            lastUpdated = now

            server.update( )
            client = server.getNextClientToMove( )
            while threadRunning and server.isRunning( ) and client is not None:
                if client["name"] == "VelodynePoints":
                    handler.addConnection( client["socket"], client["address"] )
                else:
                    print( "[VelodynePoints] Invalid connection type %s. Disconnected from %s" % (str(client["name"]), str(client["address"])) )
                    client["socket"].close( )
                client = server.getNextClientToMove( )

            handler.update( )
            time.sleep( 0 )
        else:
            time.sleep( diff / 4000000 )
#END workerMain( )

if __name__ == '__main__':
    workerMain( )
#END if-else
