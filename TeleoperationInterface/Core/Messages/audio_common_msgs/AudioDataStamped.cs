﻿using AuthenticTeleoperation.Messages.Standard;
using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Audio
{
    public sealed class AudioDataStamped : Message
    {
        private Header _header = null;
        private AudioData _audio = null;


        public AudioDataStamped( )
        {
            _header = new Header( );
            _audio = new AudioData( );
        }


        public AudioDataStamped( MediaBuffer audio )
        {
            _header = new Header( );
            _audio = new AudioData( audio );
        }


        public Header Header
        {
            get { return _header; }
            set { _header = value ?? new Header( ); }
        }


        public AudioData Audio
        {
            get { return _audio; }
            set { _audio = value ?? new AudioData( ); }
        }


        public override int MessageSize
        {
            get { return MSG__TYPE_SIZE + Header.MessageSize + Audio.MessageSize; }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + Header.MessageMinimumSize + Audio.MessageMinimumSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.AUDIO_DATA_STAMPED; }
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
                    nBytes += Header.Deserialize( data, offset + nBytes, length - nBytes );
                    nBytes += Audio.Deserialize( data, offset + nBytes, length - nBytes );
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
                    nBytes += Header.Serialize( data, offset + nBytes, length - nBytes );
                    nBytes += Audio.Serialize( data, offset + nBytes, length - nBytes );
                }
            }
            return nBytes;
        }

    }
}
