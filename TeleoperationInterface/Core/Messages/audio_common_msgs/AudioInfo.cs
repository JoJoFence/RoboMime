using System;
using System.Diagnostics;
using System.Text;

namespace AuthenticTeleoperation.Messages.Audio
{
    public sealed class AudioInfo : Message
    {
        private string _sampleFormat = string.Empty;
        private byte[] _szSampleFormat = null;
        private string _codingFormat = string.Empty;
        private byte[] _szCodingFormat = null;


        public AudioInfo( )
        {
        }


        public byte Channels
        {
            get;
            set;
        }


        public uint SampleRate
        {
            get;
            set;
        }


        public string SampleFormat
        {
            get { return _sampleFormat; }
            set
            {
                _sampleFormat = value ?? string.Empty;
                _szSampleFormat = null;
            }
        }


        public uint Bitrate
        {
            get;
            set;
        }


        public string CodingFormat
        {
            get { return _codingFormat; }
            set
            {
                _codingFormat = value ?? string.Empty;
                _szCodingFormat = null;
            }
        }


        public override int MessageSize
        {
            get
            {
                if( _szSampleFormat == null )
                {
                    _szSampleFormat = !string.IsNullOrEmpty( _sampleFormat ) ? Encoding.UTF8.GetBytes( _sampleFormat ) : new byte[0];
                }
                if( _szCodingFormat == null )
                {
                    _szCodingFormat = !string.IsNullOrEmpty( _codingFormat ) ? Encoding.UTF8.GetBytes( _codingFormat ) : new byte[0];
                }
                return MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 2 + _szSampleFormat.Length + _szCodingFormat.Length;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 2; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.AUDIO_INFO; }
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
                    Channels = *(src + MSG__TYPE_SIZE);
                    SampleRate = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 0);
                    Bitrate = *(uint*)(src + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 1);
                    int sfLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 0);
                    int cfLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 1);
                    int dataOffset = MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 2;
                    sfLength = Math.Min( length - dataOffset, sfLength );
                    SampleFormat = sfLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, sfLength ) : string.Empty;
                    dataOffset += sfLength;
                    cfLength = Math.Min( length - dataOffset, cfLength );
                    CodingFormat = cfLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, cfLength ) : string.Empty;
                    nBytes = dataOffset + cfLength;
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
                    *(dst + MSG__TYPE_SIZE) = Channels;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 0) = SampleRate;
                    *(uint*)(dst + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 1) = Bitrate;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 0) = _szSampleFormat.Length;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 1) = _szCodingFormat.Length;
                    int dataOffset = MSG__TYPE_SIZE + sizeof( byte ) + sizeof( uint ) * 2 + sizeof( int ) * 2;
                    Buffer.BlockCopy( _szSampleFormat, 0, data, offset + dataOffset, _szSampleFormat.Length );
                    dataOffset += _szSampleFormat.Length;
                    Buffer.BlockCopy( _szCodingFormat, 0, data, offset + dataOffset, _szCodingFormat.Length );
                    nBytes = dataOffset + _szCodingFormat.Length;
                }
            }
            return nBytes;
        }

    }
}
