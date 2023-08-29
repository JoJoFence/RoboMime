#!/usr/bin/python
import argparse
from os import makedirs
import cv2
import rospy
import signal
import struct
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage #, Image
from Configuration import ConfigurationSection
from Serialization import Serialization
from UdpServer import UdpServer

class VideoCapture:
    DEFAULT_UDP_IDENTIFIER  = 0x1E270A7D
    DEFAULT_UDP_LISTEN_PORT = 9561
    DEFAULT_UDP_REPLY_PORT  = 9561

    def __init__( self, params = None ):
        self._udpCommunicator = None
        self._udpIdentifier = VideoCapture.DEFAULT_UDP_IDENTIFIER
        self._udpListenPort = VideoCapture.DEFAULT_UDP_LISTEN_PORT
        self._udpReplyPort = VideoCapture.DEFAULT_UDP_REPLY_PORT
        self._imageBridge = CvBridge( )
        self._camera = None
        try:
            enable = 0
            rosVideoName = "/usb_cam/image_raw/compressed"
            if params is not None:
                enable = int( params.get( 'enable', enable ) )
                rosVideoName = str( params.get( 'rosVideoName', rosVideoName ) )
                self._udpIdentifier = int( params.get( 'udpIdentifier', self._udpIdentifier ) )
                self._udpListenPort = int( params.get( 'udpListenPort', self._udpListenPort ) )
                self._udpReplyPort = int( params.get( 'udpReplyPort', self._udpReplyPort ) )
            if enable != 0:
                rospy.init_node( "SS_VideoCapture", anonymous = True )
                self._camera = rospy.Subscriber( rosVideoName, CompressedImage, self.onCompressedImageReceived )
                #self._camera = rospy.Subscriber( rosVideoName, Image, self.onImageReceived )
                self._udpCommunicator = UdpServer( self._udpListenPort, self._udpReplyPort, "VideoCapture" )
                self._udpCommunicator.setCommunicationWaitTime( 11 )
        except Exception as ex:
            print( "[VideoCapture] Error: %s" % ex )
    #END __init__( )

    def __del__( self ):
        if self._camera is not None:
            self._camera.unregister( )
            self._camera = None
        self._imageBridge = None
        self._udpCommunicator = None
    #END __del__( )

    def threadUpdateInterval( self ):
        return 100 # 0.1 miliseconds
    #END threadUpdateInterval( )

    def onCompressedImageReceived( self, data ):
        packet = struct.pack( "<I", self._udpIdentifier ) + Serialization.packCompressedImage( data )
        self._udpCommunicator.broadcast( packet )
    #END onCompressedImageReceived( )

    def onImageReceived( self, data ):
        try: # Convert ROS Image message to OpenCV2
            cv2_img = self._imageBridge.imgmsg_to_cv2( data, data.encoding )
        except CvBridgeError as ex:
            print( "[VideoCapture] CvBridgeError: %s" % ex )
            cv2_img = None
        if cv2_img != None:
            flag, cv2_img = cv2.imencode( ".jpg", cv2_img )
            msg = CompressedImage( )
            msg.header = data.header
            msg.format = "jpeg"
            msg.data = bytearray( cv2_img )
            packet = struct.pack( "<I", self._udpIdentifier ) + Serialization.packCompressedImage( msg )
            self._udpCommunicator.broadcast( packet )
    #END onImageReceived( )

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
#END VideoCapture


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
    parser.add_argument( "-c", "--camera-capture", dest="rosVideoName", default="/usb_cam/image_raw/compressed", type=str, help="ROS video topic name" )
    parser.add_argument( "-ui", "--udp-identifier",  dest="udpIdentifier", default=VideoCapture.DEFAULT_UDP_IDENTIFIER,  type=int, help="Connection identifier for the UDP communication" )
    parser.add_argument( "-up", "--udp-listen-port", dest="udpListenPort", default=VideoCapture.DEFAULT_UDP_LISTEN_PORT, type=int, help="Listening port for the UDP communication" )
    parser.add_argument( "-ur", "--udp-reply-port",  dest="udpReplyPort",  default=VideoCapture.DEFAULT_UDP_REPLY_PORT,  type=int, help="Reply port for the UDP communication" )
    args = parser.parse_args( rospy.myargv( )[1:] )

    params = ConfigurationSection( )
    params.add( 'enable', 1 )
    params.add( 'rosVideoName', args.rosVideoName )
    params.add( 'udpIdentifier', args.udpIdentifier )
    params.add( 'udpListenPort', args.udpListenPort )
    params.add( 'udpReplyPort', args.udpReplyPort )

    handler = VideoCapture( params )
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
