using AuthenticTeleoperation.Messages.Standard;
using System;
using System.Diagnostics;
using System.Text;

namespace AuthenticTeleoperation.Messages.Sensor
{
    public sealed class CompressedImage : Message
    {
        private MediaBuffer _data = null;
        private Header _header = new Header( );
        private string _format = string.Empty;
        private byte[] _szFormat = null;


        public CompressedImage( )
        {
            _data = new MediaBuffer( );
        }


        public CompressedImage( MediaBuffer data )
        {
            _data = data ?? throw new ArgumentNullException( );
            _data.AddRef( );
        }


        public Header Header
        {
            get { return _header; }
            set { _header = value ?? new Header( ); }
        }


        public string Format
        {
            get { return _format; }
            set
            {
                _format = value ?? string.Empty;
                _szFormat = null;
            }
        }


        public MediaBuffer Data
        {
            get { return _data; }
            set
            {
                _data.RemoveRef( );
                _data = value ?? new MediaBuffer( );
                _data.AddRef( );
            }
        }


        public override int MessageSize
        {
            get
            {
                if( _szFormat == null )
                {
                    _szFormat = !string.IsNullOrEmpty( _format ) ? Encoding.UTF8.GetBytes( _format ) : new byte[0];
                }
                return MSG__TYPE_SIZE + sizeof( int ) * 2 + Header.MessageSize + _szFormat.Length + Data.Length;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( int ) * 2 + Header.MessageMinimumSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.SENSOR_COMPRESSEDIMAGE; }
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
                    int fLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( int ) * 0);
                    int dLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( int ) * 1);
                    int dataOffset = MSG__TYPE_SIZE + sizeof( int ) * 2;
                    dataOffset += Header.Deserialize( data, offset + dataOffset, length - dataOffset );
                    fLength = Math.Min( length - dataOffset, fLength );
                    Format = fLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, fLength ) : string.Empty;
                    dataOffset += fLength;
                    dLength = Math.Min( length - dataOffset, dLength );
                    Data.EnsureCapacity( dLength );
                    Data.Length = dLength;
                    Buffer.BlockCopy( data, offset + dataOffset, Data, 0, dLength );
                    nBytes = dataOffset + dLength;
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
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 0) = _szFormat.Length;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 1) = Data.Length;
                    int dataOffset = MSG__TYPE_SIZE + sizeof( int ) * 2;
                    dataOffset += Header.Serialize( data, offset + dataOffset, length - dataOffset );
                    Buffer.BlockCopy( _szFormat, 0, data, offset + dataOffset, _szFormat.Length );
                    dataOffset += _szFormat.Length;
                    Buffer.BlockCopy( Data, 0, data, offset + dataOffset, _data.Length );
                    nBytes = dataOffset + _data.Length;
                }
            }
            return nBytes;
        }

    }
}
