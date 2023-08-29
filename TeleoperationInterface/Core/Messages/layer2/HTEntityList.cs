using AuthenticTeleoperation.Messages.Standard;
using System.Collections.Generic;
using System.Diagnostics;

namespace AuthenticTeleoperation.Messages.Layer2
{
    public sealed class HTEntityList : Message
    {
        private Header _header = new Header( );


        public HTEntityList( )
        {
            List = new List<HTEntity>( );
        }


        public Header Header
        {
            get { return _header; }
            set { _header = value ?? new Header( ); }
        }


        public List<HTEntity> List
        {
            get;
            private set;
        }


        public override int MessageSize
        {
            get
            {
                int nBytes = MSG__TYPE_SIZE + sizeof( int ) + Header.MessageSize;
                for( int i = 0; i < List.Count; ++i )
                {
                    if( List[i] == null )
                    {
                        List[i] = new HTEntity( );
                    }
                    nBytes += List[i].MessageSize;
                }
                return nBytes;
            }
        }


        internal override int MessageMinimumSize
        {
            get { return MSG__TYPE_SIZE + sizeof( int ) + Header.MessageMinimumSize; }
        }


        internal override ushort MessageType
        {
            get { return (ushort)MsgType.Layer2_HTEntityList; }
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
                    int count = *(int*)(src + MSG__TYPE_SIZE);
                    nBytes = MSG__TYPE_SIZE + sizeof( int );
                    nBytes += Header.Deserialize( data, offset + nBytes, length - nBytes );
                    List.Clear( );
                    for( int i = 0; i < count; ++i )
                    {
                        HTEntity entity = new HTEntity( );
                        nBytes += entity.Deserialize( data, offset + nBytes, length - nBytes );
                        List.Add( entity );
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
                    *(int*)(dst + MSG__TYPE_SIZE) = List.Count;
                    nBytes = MSG__TYPE_SIZE + sizeof( int );
                    nBytes += Header.Serialize( data, offset + nBytes, length - nBytes );
                    for( int i = 0; i < List.Count; ++i )
                    {
                        nBytes += List[i].Serialize( data, offset + nBytes, length - nBytes );
                    }
                }
            }
            return nBytes;
        }

    }
}
