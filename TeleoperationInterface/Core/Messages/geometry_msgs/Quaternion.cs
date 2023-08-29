﻿using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Geometry
{
    public sealed class Quaternion : Message
    {
        public Quaternion( )
        {
        }


        public double X
        {
            get;
            set;
        }


        public double Y
        {
            get;
            set;
        }


        public double Z
        {
            get;
            set;
        }


        public double W
        {
            get;
            set;
        }


        public override int MessageSize
        {
            get { return MSG__TYPE_SIZE + sizeof( double ) * 4; }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( double ) * 4; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.GEOMETRY_QUATERNION; }
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
                    X = *(double*)(src + MSG__TYPE_SIZE + sizeof( double ) * 0);
                    Y = *(double*)(src + MSG__TYPE_SIZE + sizeof( double ) * 1);
                    Z = *(double*)(src + MSG__TYPE_SIZE + sizeof( double ) * 2);
                    W = *(double*)(src + MSG__TYPE_SIZE + sizeof( double ) * 3);
                    nBytes = MSG__TYPE_SIZE + sizeof( double ) * 4;
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
                    *(double*)(dst + MSG__TYPE_SIZE + sizeof( double ) * 0) = X;
                    *(double*)(dst + MSG__TYPE_SIZE + sizeof( double ) * 1) = Y;
                    *(double*)(dst + MSG__TYPE_SIZE + sizeof( double ) * 2) = Z;
                    *(double*)(dst + MSG__TYPE_SIZE + sizeof( double ) * 3) = W;
                    nBytes = MSG__TYPE_SIZE + sizeof( double ) * 4;
                }
            }
            return nBytes;
        }

    }
}
