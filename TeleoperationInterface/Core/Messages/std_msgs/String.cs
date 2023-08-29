using System;
using System.Diagnostics;
using System.Text;

namespace AuthenticTeleoperation.Messages.Standard
{
    public sealed class String : Message
    {
        private string _data = string.Empty;
        private byte[] _szData = null;


        public String( )
        {
        }


        public string Data
        {
            get { return _data; }
            set
            {
                _data = value ?? string.Empty;
                _szData = null;
            }
        }


        public override int MessageSize
        {
            get
            {
                if( _szData == null )
                {
                    _szData = !string.IsNullOrEmpty( _data ) ? Encoding.UTF8.GetBytes( _data ) : new byte[0];
                }
                return MSG__TYPE_SIZE + sizeof( int ) + _szData.Length;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( int ); }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.STANDARD_STRING; }
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
                    int dataLength = *(int*)(src + MSG__TYPE_SIZE);
                    int dataOffset = MSG__TYPE_SIZE + sizeof( int );
                    dataLength = Math.Min( length - dataOffset, dataLength );
                    Data = dataLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, dataLength ) : string.Empty;
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
                    *(int*)(dst + MSG__TYPE_SIZE) = _szData.Length;
                    Buffer.BlockCopy( _szData, 0, data, offset + MSG__TYPE_SIZE + sizeof( int ), _szData.Length );
                    nBytes = MSG__TYPE_SIZE + sizeof( int ) + _szData.Length;
                }
            }
            return nBytes;
        }

    }
}
