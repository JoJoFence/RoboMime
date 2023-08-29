using SSSoftworks.Data;
using System;
using System.Diagnostics;

namespace AuthenticTeleoperation.Messages
{
    public sealed class MediaBuffer : RecycleMemory
    {
        public MediaBuffer( )
        {
        }


        public int GetData( byte[] data, int offset, int length )
        {
            Debug.Assert( data != null );
            Debug.Assert( 0 <= offset && offset < data.Length );
            Debug.Assert( 0 <= length && offset + length <= data.Length );
            if( data != null && 0 <= offset && offset < data.Length && 0 <= length && offset + length <= data.Length )
            {
                length = Math.Min( Length, length );
                Buffer.BlockCopy( Data, 0, data, offset, length );
                return length;
            }
            return 0;
        }


        public int SetData( byte[] data, int offset, int length )
        {
            Debug.Assert( data != null );
            Debug.Assert( 0 <= offset && offset < data.Length );
            Debug.Assert( 0 <= length && offset + length <= data.Length );
            if( data != null && 0 <= offset && offset < data.Length && 0 <= length && offset + length <= data.Length )
            {
                EnsureCapacity( length );
                Length = length;
                Buffer.BlockCopy( data, offset, Data, 0, length );
                return length;
            }
            return 0;
        }


        protected override void OnRecycleRequesting( )
        {
            Length = 0;
        }


        public static implicit operator byte[]( MediaBuffer buf ) => buf.Data;

    }
}
