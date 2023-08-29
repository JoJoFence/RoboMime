using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Geometry
{
    public sealed class Pose : Message
    {
        private Point _position = new Point( );
        private Quaternion _orientation = new Quaternion( );


        public Pose( )
        {
        }


        public Point Position
        {
            get { return _position; }
            set { _position = value ?? new Point( ); }
        }


        public Quaternion Orientation
        {
            get { return _orientation; }
            set { _orientation = value ?? new Quaternion( ); }
        }


        public override int MessageSize
        {
            get { return MSG__TYPE_SIZE + Position.MessageSize + Orientation.MessageSize; }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + Position.MessageMinimumSize + Orientation.MessageMinimumSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.GEOMETRY_POSE; }
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
                    nBytes += Position.Deserialize( data, offset + nBytes, length - nBytes );
                    nBytes += Orientation.Deserialize( data, offset + nBytes, length - nBytes );
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
                    nBytes += Position.Serialize( data, offset + nBytes, length - nBytes );
                    nBytes += Orientation.Serialize( data, offset + nBytes, length - nBytes );
                }
            }
            return nBytes;
        }

    }
}
