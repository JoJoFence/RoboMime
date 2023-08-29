using AuthenticTeleoperation.Messages;
using AuthenticTeleoperation.Messages.Sensor;

namespace AuthenticTeleoperation.Robots
{
    public sealed class VideoCapture : RosProxyUdpConnection
    {
        private MediaBufferPool _pool = null;


        public VideoCapture( MediaBufferPool pool = null ) : base( 0x1E270A7D, Robovie.CTYPE_VIDEO_CAPTURE )
        {
            _pool = pool ?? new MediaBufferPool( ( ) => { return new MediaBuffer( ); } );
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            MediaBuffer buffer = _pool.Dequeue( );
            CompressedImage data = new CompressedImage( buffer );
            data.Deserialize( packet, offset, length );
            NotifyDataAvailable( data );
            buffer.RemoveRef( );
        }

    }
}
