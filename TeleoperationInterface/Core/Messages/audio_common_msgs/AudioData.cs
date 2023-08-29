using System;
using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Audio
{
    public sealed class AudioData : Message
    {
        private MediaBuffer _data = null;


        public AudioData( )
        {
            _data = new MediaBuffer( );
        }


        public AudioData( MediaBuffer data )
        {
            _data = data ?? throw new ArgumentNullException( );
            _data.AddRef( );
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
            get { return MSG__TYPE_SIZE + sizeof( int ) + _data.Length; }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( int ); }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.AUDIO_DATA; }
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
                    Data.EnsureCapacity( dataLength );
                    Data.Length = dataLength;
                    Buffer.BlockCopy( data, offset + dataOffset, Data, 0, dataLength );
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
                    *(int*)(dst + MSG__TYPE_SIZE) = _data.Length;
                    Buffer.BlockCopy( Data, 0, data, offset + MSG__TYPE_SIZE + sizeof( int ), _data.Length );
                    nBytes = MSG__TYPE_SIZE + sizeof( int ) + _data.Length;
                }
            }
            return nBytes;
        }

    }
}
