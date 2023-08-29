#!/usr/bin/python
import alsaaudio
import argparse
import rospy
import signal
import struct
import time
from audio_common_msgs.msg import AudioData #, AudioInfo
from Configuration import ConfigurationSection
from Serialization import Serialization
from UdpServer import UdpServer

class AudioCapture:
    DEFAULT_UDP_IDENTIFIER  = 0x19AB0A7D
    DEFAULT_UDP_LISTEN_PORT = 9562
    DEFAULT_UDP_REPLY_PORT  = 9562

    def __init__( self, params = None ):
        self._udpCommunicator = None
        self._udpIdentifier = AudioCapture.DEFAULT_UDP_IDENTIFIER
        self._udpListenPort = AudioCapture.DEFAULT_UDP_LISTEN_PORT
        self._udpReplyPort = AudioCapture.DEFAULT_UDP_REPLY_PORT
        self._audioData = None
        self._audioInfo = None
        try:
            enable = 0
            rosAudioCaptureName = "/audio"
            rosAudioCapInfoName = "/audio_info"
            if params is not None:
                enable = int( params.get( 'enable', enable ) )
                rosAudioCaptureName = str( params.get( 'rosAudioCaptureName', rosAudioCaptureName ) )
                rosAudioCapInfoName = str( params.get( 'rosAudioInfoName', rosAudioCapInfoName ) )
                self._udpIdentifier = int( params.get( 'udpIdentifier', self._udpIdentifier ) )
                self._udpListenPort = int( params.get( 'udpListenPort', self._udpListenPort ) )
                self._udpReplyPort = int( params.get( 'udpReplyPort', self._udpReplyPort ) )
            if enable != 0:
                rospy.init_node( "SS_AudioCapture", anonymous = True )
                self._audioData = rospy.Subscriber( rosAudioCaptureName, AudioData, self.onAudioDataReceived )
                #self._audioInfo = rospy.Subscriber( rosAudioCapInfoName, AudioInfo, self.onAudioInfoReceived )
                self._udpCommunicator = UdpServer( self._udpListenPort, self._udpReplyPort, "AudioCapture" )
                self._udpCommunicator.setCommunicationWaitTime( 11 )
        except Exception as ex:
            print( "[AudioCapture] Error: %s" % ex )
    #END __init__( )

    def __del__( self ):
        if self._audioData is not None:
            self._audioData.unregister( )
            self._audioData = None
        if self._audioInfo is not None:
            self._audioInfo.unregister( )
            self._audioInfo = None
        self._udpCommunicator = None
    #END __del__( )

    def threadUpdateInterval( self ):
        return 100 # 0.1 miliseconds
    #END threadUpdateInterval( )

    def onAudioDataReceived( self, data ):
        packet = struct.pack( "<I", self._udpIdentifier ) + Serialization.packAudioData( data )
        self._udpCommunicator.broadcast( packet )
    #END onAudioDataReceived( )

    def onAudioInfoReceived( self, data ):
        packet = struct.pack( "<I", self._udpIdentifier ) + Serialization.packAudioInfo( data )
        self._udpCommunicator.broadcast( packet )
    #END onAudioInfoReceived( )

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
#END AudioCapture


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
    parser.add_argument( "-a", "--audio-capture", dest="rosAudioCaptureName", default="/audio",      type=str, help="ROS audio capture topic name" )
    parser.add_argument( "-i", "--audio-info",    dest="rosAudioCapInfoName", default="/audio_info", type=str, help="ROS audio information topic name" )
    parser.add_argument( "-ui", "--udp-identifier",  dest="udpIdentifier", default=AudioCapture.DEFAULT_UDP_IDENTIFIER,  type=int, help="Connection identifier for the UDP communication" )
    parser.add_argument( "-up", "--udp-listen-port", dest="udpListenPort", default=AudioCapture.DEFAULT_UDP_LISTEN_PORT, type=int, help="Listening port for the UDP communication" )
    parser.add_argument( "-ur", "--udp-reply-port",  dest="udpReplyPort",  default=AudioCapture.DEFAULT_UDP_REPLY_PORT,  type=int, help="Reply port for the UDP communication" )
    args = parser.parse_args( rospy.myargv( )[1:] )

    params = ConfigurationSection( )
    params.add( 'enable', 1 )
    params.add( 'rosAudioCaptureName', args.rosAudioCaptureName )
    params.add( 'rosAudioInfoName', args.rosAudioCapInfoName )
    params.add( 'udpIdentifier', args.udpIdentifier )
    params.add( 'udpListenPort', args.udpListenPort )
    params.add( 'udpReplyPort', args.udpReplyPort )

    handler = AudioCapture( params )
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
