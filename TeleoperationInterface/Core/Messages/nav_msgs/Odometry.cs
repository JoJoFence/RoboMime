using AuthenticTeleoperation.Messages.Geometry;
using AuthenticTeleoperation.Messages.Standard;
using System;
using System.Diagnostics;
using System.Text;

namespace AuthenticTeleoperation.Messages.Navigation
{
    public sealed class Odometry : Message
    {
        private Header _header = new Header( );
        private PoseWithCovariance _pose = new PoseWithCovariance( );
        private TwistWithCovariance _twist = new TwistWithCovariance( );
        private string _childFrameID = string.Empty;
        private byte[] _szChildFrameID = null;


        public Odometry( )
        {
        }


        public Header Header
        {
            get { return _header; }
            set { _header = value ?? new Header( ); }
        }


        public string ChildFrameID
        {
            get { return _childFrameID; }
            set
            {
                _childFrameID = value ?? string.Empty;
                _szChildFrameID = null;
            }
        }


        public PoseWithCovariance Pose
        {
            get { return _pose; }
            set { _pose = value ?? new PoseWithCovariance( ); }
        }


        public TwistWithCovariance Twist
        {
            get { return _twist; }
            set { _twist = value ?? new TwistWithCovariance( ); }
        }


        public override int MessageSize
        {
            get
            {
                if( _szChildFrameID == null )
                {
                    _szChildFrameID = !string.IsNullOrEmpty( _childFrameID ) ? Encoding.UTF8.GetBytes( _childFrameID ) : new byte[0];
                }
                return MSG__TYPE_SIZE + sizeof( int ) + _szChildFrameID.Length + Header.MessageSize + Pose.MessageSize + Twist.MessageSize;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( int ) + Header.MessageSize + Pose.MessageSize + Twist.MessageSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.NAVIGATION_ODOMETRY; }
        }


        public override unsafe int Deserialize( byte[] data, int offset, int length )
        {
            int nBytes = 0;
            Debug.Assert( data != null );
            Debug.Assert( 0 <= offset && offset < data.Length );
            Debug.Assert( 0 <= length && offset + length <= data.Length );
            if( data != null && 0 <= offset && offset < data.Length && 0 <= length && offset + length <= data.Length && length >= MessageMinimumSize )
            {
                fixed( byte* src = &data[offset] )
                {
                    ushort msgType = *(ushort*)(src);
                    Debug.Assert( msgType == MessageType );
                    int cfLength = *(int*)(src + MSG__TYPE_SIZE);
                    int dataOffset = MSG__TYPE_SIZE + sizeof( int );
                    cfLength = Math.Min( length - dataOffset, cfLength );
                    ChildFrameID = cfLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, cfLength ) : string.Empty;
                    nBytes = dataOffset + cfLength;
                    nBytes += Header.Deserialize( data, offset + nBytes, length - nBytes );
                    nBytes += Pose.Deserialize( data, offset + nBytes, length - nBytes );
                    nBytes += Twist.Deserialize( data, offset + nBytes, length - nBytes );
                }
            }
            return nBytes;
        }


        public override unsafe int Serialize( byte[] data, int offset, int length )
        {
            int nBytes = 0;
            Debug.Assert( data != null );
            Debug.Assert( 0 <= offset && offset < data.Length );
            Debug.Assert( 0 <= length && offset + length <= data.Length );
            if( data != null && 0 <= offset && offset < data.Length && 0 <= length && offset + length <= data.Length && length >= MessageSize )
            {
                fixed( byte* dst = &data[offset] )
                {
                    *(ushort*)(dst) = MessageType;
                    *(int*)(dst + MSG__TYPE_SIZE) = _szChildFrameID.Length;
                    Buffer.BlockCopy( _szChildFrameID, 0, data, offset + MSG__TYPE_SIZE + sizeof( int ), _szChildFrameID.Length );
                    nBytes = MSG__TYPE_SIZE + sizeof( int ) + _szChildFrameID.Length;
                    nBytes += Header.Serialize( data, offset + nBytes, length - nBytes );
                    nBytes += Pose.Serialize( data, offset + nBytes, length - nBytes );
                    nBytes += Twist.Serialize( data, offset + nBytes, length - nBytes );
                }
            }
            return nBytes;
        }

    }
}
