using AuthenticTeleoperation.Messages;
using AuthenticTeleoperation.Messages.Audio;
using System;

namespace AuthenticTeleoperation.Robots
{
    public sealed class AudioCapture : RosProxyUdpConnection
    {
        private MediaBufferPool _pool = null;


        public AudioCapture( MediaBufferPool pool = null ) : base( 0x19AB0A7D, Robovie.CTYPE_AUDIO_CAPTURE )
        {
            _pool = pool ?? new MediaBufferPool( ( ) => { return new MediaBuffer( ); } );
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            if( length >= sizeof( MsgType ) )
            {
                if( BitConverter.ToUInt16( packet, offset ) == (ushort)MsgType.AUDIO_INFO )
                {
                    AudioInfo info = new AudioInfo( );
                    info.Deserialize( packet, offset, length );
                    NotifyDataAvailable( info );
                }
                else
                {
                    MediaBuffer buffer = _pool.Dequeue( );
                    AudioData data = new AudioData( buffer );
                    data.Deserialize( packet, offset, length );
                    NotifyDataAvailable( data );
                    buffer.RemoveRef( );
                }
            }
        }

    }
}
