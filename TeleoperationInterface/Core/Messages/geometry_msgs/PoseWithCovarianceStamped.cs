using AuthenticTeleoperation.Messages.Standard;
using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Geometry
{
    public sealed class PoseWithCovarianceStamped : Message
    {
        private Header _header = new Header( );
        private PoseWithCovariance _pose = new PoseWithCovariance( );


        public PoseWithCovarianceStamped( )
        {
        }


        public Header Header
        {
            get { return _header; }
            set { _header = value ?? new Header( ); }
        }


        public PoseWithCovariance Pose
        {
            get { return _pose; }
            set { _pose = value ?? new PoseWithCovariance( ); }
        }


        public override int MessageSize
        {
            get { return MSG__TYPE_SIZE + Header.MessageSize + Pose.MessageSize; }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + Header.MessageMinimumSize + Pose.MessageMinimumSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.GEOMETRY_POSEWITHCOVARIANCESTAMPED; }
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
                    nBytes = MSG__TYPE_SIZE;
                    nBytes += Header.Deserialize( data, offset + nBytes, length - nBytes );
                    nBytes += Pose.Deserialize( data, offset + nBytes, length - nBytes );
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
                    nBytes = MSG__TYPE_SIZE;
                    nBytes += Header.Serialize( data, offset + nBytes, length - nBytes );
                    nBytes += Pose.Serialize( data, offset + nBytes, length - nBytes );
                }
            }
            return nBytes;
        }

    }
}
