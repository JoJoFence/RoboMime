using System;
using System.Diagnostics;
using System.Text;

namespace AuthenticTeleoperation.Messages.Sensor
{
    public sealed class PointField : Message
    {
        public const byte INT8 = 1;
        public const byte UINT8 = 2;
        public const byte INT16 = 3;
        public const byte UINT16 = 4;
        public const byte INT32 = 5;
        public const byte UINT32 = 6;
        public const byte FLOAT32 = 7;
        public const byte FLOAT64 = 8;
        private string _name = string.Empty;
        private byte[] _szName = null;


        public PointField( )
        {
        }


        public string Name
        {
            get { return _name; }
            set
            {
                _name = value ?? string.Empty;
                _szName = null;
            }
        }


        public uint Offset
        {
            get;
            set;
        }


        public byte DataType
        {
            get;
            set;
        }


        public uint Count
        {
            get;
            set;
        }


        public override int MessageSize
        {
            get
            {
                if( _szName == null )
                {
                    _szName = !string.IsNullOrEmpty( _name ) ? Encoding.UTF8.GetBytes( _name ) : new byte[0];
                }
                return MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) + _szName.Length;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ); }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.SENSOR_POINTFIELD; }
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
                    DataType = *(src + MSG__TYPE_SIZE);
                    Offset = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 0);
                    Count = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 1);
                    int nameLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2);
                    int dataOffset = MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int );
                    nameLength = Math.Min( length - dataOffset, nameLength );
                    Name = nameLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, nameLength ) : string.Empty;
                    nBytes = dataOffset + nameLength;
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
                    *(dst + MSG__TYPE_SIZE) = DataType;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 0) = Offset;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 1) = Count;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2) = _szName.Length;
                    int dataOffset = MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int );
                    Buffer.BlockCopy( _szName, 0, data, offset + dataOffset, _szName.Length );
                    nBytes = dataOffset + _szName.Length;
                }
            }
            return nBytes;
        }

    }
}
