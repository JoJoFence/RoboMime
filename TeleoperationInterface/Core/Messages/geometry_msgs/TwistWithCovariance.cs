using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Geometry
{
    public sealed class TwistWithCovariance : Message
    {
        private Vector3 _linear = new Vector3( );
        private Vector3 _angular = new Vector3( );


        public TwistWithCovariance( )
        {
            Covariance = new double[36];
        }


        public Vector3 Linear
        {
            get { return _linear; }
            set { _linear = value ?? new Vector3( ); }
        }


        public Vector3 Angular
        {
            get { return _angular; }
            set { _angular = value ?? new Vector3( ); }
        }


        public double[] Covariance
        {
            get;
            private set;
        }


        public override int MessageSize
        {
            get { return MSG__TYPE_SIZE + Linear.MessageSize + Angular.MessageSize + Covariance.Length * sizeof( double ); }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + Linear.MessageMinimumSize + Angular.MessageMinimumSize + Covariance.Length * sizeof( double ); }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.GEOMETRY_TWISTWITHCOVARIANCE; }
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
                    for( int i = 0; i < Covariance.Length; ++i, nBytes += sizeof( double ) )
                    {
                        Covariance[i] = *(double*)(src + nBytes);
                    }
                    nBytes += Linear.Deserialize( data, offset + nBytes, length - nBytes );
                    nBytes += Angular.Deserialize( data, offset + nBytes, length - nBytes );
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
                    for( int i = 0; i < Covariance.Length; ++i, nBytes += sizeof( double ) )
                    {
                        *(double*)(dst + nBytes) = Covariance[i];
                    }
                    nBytes += Linear.Serialize( data, offset + nBytes, length - nBytes );
                    nBytes += Angular.Serialize( data, offset + nBytes, length - nBytes );
                }
            }
            return nBytes;
        }

    }
}
