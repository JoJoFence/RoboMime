using AuthenticTeleoperation.Messages.Standard;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Sensor
{
    public sealed class PointCloud2 : Message
    {
        private Header _header = new Header( );


        public PointCloud2( )
        {
            Fields = new List<PointField>( );
            Data = new byte[0];
        }


        public Header Header
        {
            get { return _header; }
            set { _header = value ?? new Header( ); }
        }


        public uint Height
        {
            get;
            set;
        }


        public uint Width
        {
            get;
            set;
        }


        public List<PointField> Fields
        {
            get;
            private set;
        }


        public bool IsBigendian
        {
            get;
            set;
        }


        public uint PointStep
        {
            get;
            set;
        }


        public uint RowStep
        {
            get;
            set;
        }


        public byte[] Data
        {
            get;
            set;
        }


        public bool IsDense
        {
            get;
            set;
        }


        public override int MessageSize
        {
            get
            {
                if( Data == null )
                {
                    Data = new byte[0];
                }
                int nBytes = MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 2 + Header.MessageSize + Data.Length;
                for( int i = 0; i < Fields.Count; ++i )
                {
                    if( Fields[i] == null )
                    {
                        Fields[i] = new PointField( );
                    }
                    nBytes += Fields[i].MessageSize;
                }
                return nBytes;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 2 + Header.MessageMinimumSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.SENSOR_POINTCLOUD2; }
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
                    IsBigendian = *(src + MSG__TYPE_SIZE + sizeof( byte ) * 0) != 0;
                    IsDense = *(src + MSG__TYPE_SIZE + sizeof( byte ) * 1) != 0;
                    Height = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 0);
                    Width = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 1);
                    PointStep = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 2);
                    RowStep = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 3);
                    int dataLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 0);
                    int fieldCount = *(int*)(src + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 1);
                    nBytes = MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 2;
                    nBytes += Header.Deserialize( data, offset + nBytes, length - nBytes );
                    dataLength = Math.Max( 0, Math.Min( length - nBytes, dataLength ) );
                    Data = new byte[dataLength];
                    Buffer.BlockCopy( data, offset + nBytes, Data, 0, dataLength );
                    nBytes += dataLength;
                    Fields.Clear( );
                    for( int i = 0; i < fieldCount; ++i )
                    {
                        PointField field = new PointField( );
                        nBytes += field.Deserialize( data, offset + nBytes, length - nBytes );
                        Fields.Add( field );
                    }
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
                    *(dst + MSG__TYPE_SIZE + sizeof( byte ) * 0) = (byte)(IsBigendian ? 1 : 0);
                    *(dst + MSG__TYPE_SIZE + sizeof( byte ) * 1) = (byte)(IsDense ? 1 : 0);
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 0) = Height;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 1) = Width;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 2) = PointStep;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 3) = RowStep;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 0) = Data.Length;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 1) = Fields.Count;
                    nBytes = MSG__TYPE_SIZE + sizeof( byte ) * 2 + sizeof( uint ) * 4 + sizeof( int ) * 2;
                    nBytes += Header.Serialize( data, offset + nBytes, length - nBytes );
                    Buffer.BlockCopy( Data, 0, data, offset + nBytes, Data.Length );
                    nBytes += Data.Length;
                    for( int i = 0; i < Fields.Count; ++i )
                    {
                        nBytes += Fields[i].Serialize( data, offset + nBytes, length - nBytes );
                    }
                }
            }
            return nBytes;
        }

    }
}
