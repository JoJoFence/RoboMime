#!/usr/bin/python
import alsaaudio
import argparse
import rospy
import signal
import struct
import time
from Configuration import ConfigurationSection
from Serialization import Serialization
from UdpServer import UdpServer

class AudioPlayback:
    DEFAULT_UDP_IDENTIFIER  = 0x66AFDD87
    DEFAULT_UDP_LISTEN_PORT = 9563
    DEFAULT_UDP_REPLY_PORT  = 9563
    MAX_DURATION_PER_BUFFER = 0.5
    PCM_OFFSET = 8 + 2 * 3

    def __init__( self, params = None ):
        self._udpCommunicator = None
        self._udpIdentifier = AudioPlayback.DEFAULT_UDP_IDENTIFIER
        self._udpListenPort = AudioPlayback.DEFAULT_UDP_LISTEN_PORT
        self._udpReplyPort = AudioPlayback.DEFAULT_UDP_REPLY_PORT
        self._audioPlayer = None
        self._avgBytesPerSecond = 16000 * 16/8 * 1
        self._buffering = 0.5
        self._playData = list( )
        try:
            enable = 0
            device = "default"
            rate = 16000
            channels = 1
            periodsize = 1600
            buffering = 0.5
            if params is not None:
                enable = int( params.get( 'enable', enable ) )
                device = str( params.get( 'device', device ) )
                rate = int( params.get( 'rate', rate ) )
                channels = int( params.get( 'channels', channels ) )
                periodsize = int( params.get( 'periodsize', periodsize ) )
                buffering = float( params.get( 'buffering', buffering ) )
                self._udpIdentifier = int( params.get( 'udpIdentifier', self._udpIdentifier ) )
                self._udpListenPort = int( params.get( 'udpListenPort', self._udpListenPort ) )
                self._udpReplyPort = int( params.get( 'udpReplyPort', self._udpReplyPort ) )
            if enable != 0:
                rospy.init_node( "SS_AudioPlayback", anonymous = True )
                self._audioPlayer = alsaaudio.PCM( alsaaudio.PCM_PLAYBACK, alsaaudio.PCM_NORMAL, device )
                self._audioPlayer.setformat( alsaaudio.PCM_FORMAT_S16_LE )
                self._audioPlayer.setrate( rate )
                self._audioPlayer.setchannels( channels )
                self._audioPlayer.setperiodsize( periodsize )
                self._avgBytesPerSecond = rate * 16/8 * channels
                self._buffering = buffering
                self._udpCommunicator = UdpServer( self._udpListenPort, self._udpReplyPort, "AudioPlayback" )
                self._udpCommunicator.setCommunicationWaitTime( 11 )
        except Exception as ex:
            print( "[AudioPlayback] Error: %s" % ex )
            print( "[AudioPlayback] failed to open the speaker" )
            self._audioPlayer = None
    #END __init__( )

    def __del__( self ):
        del self._playData[:]
        self._audioPlayer = None
        self._udpCommunicator = None
    #END __del__( )

    def threadUpdateInterval( self ):
        return 1000 # 1 miliseconds
    #END threadUpdateInterval( )

    def update( self ):
        if self._udpCommunicator is None:
            return

        self._udpCommunicator.update( )
        packet, client = self._udpCommunicator.receive( )
        while packet is not None:
            if len( packet ) < 4 or struct.unpack( "<I", packet[:4] )[0] != self._udpIdentifier:
                # unexpected packet data -> disconnect
                client.disconnect( )
            else:
                packet = packet[4:]
                if len( packet ) > 0:
                    pcm = Serialization.unpackAudioData( packet ).data
                    duration = float(len( pcm )) / self._avgBytesPerSecond
                    if len( self._playData ) <= 0: # first data
                        self._playData.append( { "timestamp" : time.time( ) + self._buffering, "duration": duration, "pcm": pcm } )
                    elif len( self._playData ) == 1 and self._playData[0]["duration"] < AudioPlayback.MAX_DURATION_PER_BUFFER: # append to the first data
                        self._playData[0]["duration"] = self._playData[0]["duration"] + duration
                        self._playData[0]["pcm"] = self._playData[0]["pcm"] + pcm
                    else: # from this point, all data has to be played continuously
                        self._playData.append( { "timestamp" : 0, "duration": duration, "pcm": pcm } )
                    client.send( struct.pack( "<I", self._udpIdentifier ) )
            packet, client = self._udpCommunicator.receive( )

        if len( self._playData ) > 0 and self._playData[0]["timestamp"] <= time.time( ):
            data = self._playData.pop( 0 )["pcm"]
            self._audioPlayer.write( data )
    #END update( )
#END AudioPlayback


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
    parser.add_argument( "-ad", "--audio-device",      dest="audioDevice",     default="default", type=str,   help="audio device name" )
    parser.add_argument( "-ar", "--audio-sample-rate", dest="audioSampleRate", default=16000,     type=int,   help="audio sample rate" )
    parser.add_argument( "-ac", "--audio-channels",    dest="audioChannels",   default=1,         type=int,   help="audio channels" )
    parser.add_argument( "-ap", "--audio-periodsize",  dest="audioPeriodsize", default=1600,      type=int,   help="audio period size" )
    parser.add_argument( "-ab", "--audio-buffering",   dest="audioBuffering",  default=0.5,       type=float, help="Buffering duration for the audio data" )
    parser.add_argument( "-ui", "--udp-identifier",  dest="udpIdentifier", default=AudioPlayback.DEFAULT_UDP_IDENTIFIER,  type=int, help="Connection identifier for the UDP communication" )
    parser.add_argument( "-up", "--udp-listen-port", dest="udpListenPort", default=AudioPlayback.DEFAULT_UDP_LISTEN_PORT, type=int, help="Listening port for the UDP communication" )
    parser.add_argument( "-ur", "--udp-reply-port",  dest="udpReplyPort",  default=AudioPlayback.DEFAULT_UDP_REPLY_PORT,  type=int, help="Reply port for the UDP communication" )
    args = parser.parse_args( rospy.myargv( )[1:] )

    params = ConfigurationSection( )
    params.add( 'enable', 1 )
    params.add( 'device', args.audioDevice )
    params.add( 'rate', args.audioSampleRate )
    params.add( 'channels', args.audioChannels )
    params.add( 'periodsize', args.audioPeriodsize )
    params.add( 'buffering', args.audioBuffering )
    params.add( 'udpIdentifier', args.udpIdentifier )
    params.add( 'udpListenPort', args.udpListenPort )
    params.add( 'udpReplyPort', args.udpReplyPort )

    handler = AudioPlayback( params )
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
