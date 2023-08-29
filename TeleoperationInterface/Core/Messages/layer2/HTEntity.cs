using AuthenticTeleoperation.Messages.Standard;
using System;
using System.Diagnostics;
using System.Text;

namespace AuthenticTeleoperation.Messages.Layer2
{
    public sealed class HTEntity : Message
    {
        public const byte HUMAN = 0;
        public const byte ROBOT = 1;
        public const byte OTHER = 2;
        private Header _header = new Header( );
        private string _optionFields = string.Empty;
        private byte[] _szOptionFields = null;
        private string _k2body = string.Empty;
        private byte[] _szK2body = null;


        public HTEntity( )
        {
        }


        public Header Header
        {
            get { return _header; }
            set { _header = value ?? new Header( ); }
        }


        public int ID
        {
            get;
            set;
        }


        public int UniqueID
        {
            get;
            set;
        }


        public int Type
        {
            get;
            set;
        }


        public float X
        {
            get;
            set;
        }


        public float Y
        {
            get;
            set;
        }


        public float Z
        {
            get;
            set;
        }


        public float BodyOrientation
        {
            get;
            set;
        }


        public float MotionDirection
        {
            get;
            set;
        }


        public float Velocity
        {
            get;
            set;
        }


        public float HeadOrientation
        {
            get;
            set;
        }


        public bool IsSpeaking
        {
            get;
            set;
        }


        public string OptionFields
        {
            get { return _optionFields; }
            set
            {
                _optionFields = value ?? string.Empty;
                _szOptionFields = null;
            }
        }


        public string K2body
        {
            get { return _k2body; }
            set
            {
                _k2body = value ?? string.Empty;
                _szK2body = null;
            }
        }


        public override int MessageSize
        {
            get
            {
                if( _szOptionFields == null )
                {
                    _szOptionFields = !string.IsNullOrEmpty( _optionFields ) ? Encoding.UTF8.GetBytes( _optionFields ) : new byte[0];
                }
                if( _szK2body == null )
                {
                    _szK2body = !string.IsNullOrEmpty( _k2body ) ? Encoding.UTF8.GetBytes( _k2body ) : new byte[0];
                }
                return MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 2 + _szOptionFields.Length + _szK2body.Length + Header.MessageSize;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 2 + Header.MessageMinimumSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.Layer2_HTEntity; }
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
                    ID = *(int*)(src + MSG__TYPE_SIZE + sizeof( int ) * 0);
                    UniqueID = *(int*)(src + MSG__TYPE_SIZE + sizeof( int ) * 1);
                    Type = *(int*)(src + MSG__TYPE_SIZE + sizeof( int ) * 2);
                    X = *(float*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 0);
                    Y = *(float*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 1);
                    Z = *(float*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 2);
                    BodyOrientation = *(float*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 3);
                    MotionDirection = *(float*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 4);
                    Velocity = *(float*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 5);
                    HeadOrientation = *(float*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 6);
                    IsSpeaking = *(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7) != 0;
                    int ofLength = *(int*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 0);
                    int k2Length = *(int*)(src + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 1);
                    int dataOffset = MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 2;
                    ofLength = Math.Min( length - dataOffset, ofLength );
                    OptionFields = ofLength > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, ofLength ) : string.Empty;
                    dataOffset += ofLength;
                    k2Length = Math.Min( length - dataOffset, k2Length );
                    K2body = k2Length > 0 ? Encoding.UTF8.GetString( data, offset + dataOffset, k2Length ) : string.Empty;
                    nBytes = dataOffset + k2Length;
                    nBytes += Header.Deserialize( data, offset + nBytes, length - nBytes );
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
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 0) = ID;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 1) = UniqueID;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 2) = Type;
                    *(float*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 0) = X;
                    *(float*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 1) = Y;
                    *(float*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 2) = Z;
                    *(float*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 3) = BodyOrientation;
                    *(float*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 4) = MotionDirection;
                    *(float*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 5) = Velocity;
                    *(float*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 6) = HeadOrientation;
                    *(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7) = (byte)(IsSpeaking ? 1 : 0);
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 0) = _szOptionFields.Length;
                    *(int*)(dst + MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 1) = _szK2body.Length;
                    nBytes = MSG__TYPE_SIZE + sizeof( int ) * 3 + sizeof( float ) * 7 + sizeof( byte ) + sizeof( int ) * 2;
                    Buffer.BlockCopy( _szOptionFields, 0, data, offset + nBytes, _szOptionFields.Length );
                    nBytes += _szOptionFields.Length;
                    Buffer.BlockCopy( _szK2body, 0, data, offset + nBytes, _szK2body.Length );
                    nBytes += _szK2body.Length;
                    nBytes += Header.Serialize( data, offset + nBytes, length - nBytes );
                }
            }
            return nBytes;
        }

    }
}
