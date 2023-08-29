using System;
using System.Diagnostics;
using System.Text;

namespace AuthenticTeleoperation.Messages.Standard
{
    public sealed class Header : Message
    {
        private string _frameID = string.Empty;
        private byte[] _szFrameID = null;


        public Header( )
        {
        }


        public uint Sequence
        {
            get;
            set;
        }


        public uint StampSeconds
        {
            get;
            set;
        }


        public uint StampNanoseconds
        {
            get;
            set;
        }


        public string FrameID
        {
            get { return _frameID; }
            set
            {
                _frameID = value ?? string.Empty;
                _szFrameID = null;
            }
        }


        public override int MessageSize
        {
            get
            {
                if( _szFrameID == null )
                {
                    _szFrameID = !string.IsNullOrEmpty( _frameID ) ? Encoding.UTF8.GetBytes( _frameID ) : new byte[0];
                }
                return MSG__TYPE_SIZE + sizeof( uint ) * 3 + sizeof( int ) + _szFrameID.Length;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( uint ) * 3 + sizeof( int ); }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.STANDARD_HEADER; }
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
                    Sequence = *(uint*)(src + MSG__TYPE_SIZE + sizeof( uint ) * 0);
                    StampSeconds = *(uint*)(src + MSG__TYPE_SIZE + sizeof( uint ) * 1);
                    StampNanoseconds = *(uint*)(src + MSG__TYPE_SIZE + sizeof( uint ) * 2);
                    int dataLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( uint ) * 3);
                    int dataOffset = MSG__TYPE_SIZE + sizeof( uint ) * 3 + sizeof( int );
                    dataLength = Math.Min( length - dataOffset, dataLength );
                    FrameID = dataLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, dataLength ) : string.Empty;
                    nBytes = dataOffset + dataLength;
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
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( uint ) * 0) = Sequence;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( uint ) * 1) = StampSeconds;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( uint ) * 2) = StampNanoseconds;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( uint ) * 3) = _szFrameID.Length;
                    Buffer.BlockCopy( _szFrameID, 0, data, offset + MSG__TYPE_SIZE + sizeof( uint ) * 3 + sizeof( int ), _szFrameID.Length );
                    nBytes = MSG__TYPE_SIZE + sizeof( uint ) * 3 + sizeof( int ) + _szFrameID.Length;
                }
            }
            return nBytes;
        }

    }
}
