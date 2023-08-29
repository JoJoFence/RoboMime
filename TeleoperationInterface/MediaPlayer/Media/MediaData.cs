using SSSoftworks.Data;
using System;
using System.Diagnostics;
using System.IO;
using System.Threading;

namespace MediaPlayer.Media
{
    public sealed class MediaData : IRecyclable
    {
        private byte[] _headers = new byte[sizeof( int ) + sizeof( long ) + sizeof( int )];
        private Mutex _refMutex = new Mutex( );
        private int _refCount = 1;


        public MediaData( )
        {
            Data = new byte[0];
        }


        public MediaData( int capacity )
        {
            Data = new byte[Math.Max( 0, capacity )];
        }


        public event EventHandler RecycleRequested;


        public int Capacity
        {
            get { return Data.Length; }
            set
            {
                Debug.Assert( value >= 0 );
                if( Data.Length != value )
                {
                    Data = new byte[value];
                }
            }
        }


        public byte[] Data
        {
            get;
            internal set;
        }


        public unsafe int MediaType
        {
            get
            {
                fixed( byte* ptr = _headers )
                {
                    return *((int*)ptr);
                }
            }
            set
            {
                fixed( byte* ptr = _headers )
                {
                    *((int*)ptr) = value;
                }
            }
        }


        public int Offset
        {
            get;
            set;
        }


        public unsafe int Length
        {
            get
            {
                fixed( byte* ptr = _headers )
                {
                    return *((int*)(ptr + sizeof( int ) + sizeof( long )));
                }
            }
            set
            {
                fixed( byte* ptr = _headers )
                {
                    *((int*)(ptr + sizeof( int ) + sizeof( long ))) = value;
                }
            }
        }


        public unsafe long Timestamp
        {
            get
            {
                fixed( byte* ptr = _headers )
                {
                    return *((long*)(ptr + sizeof( int )));
                }
            }
            set
            {
                fixed( byte* ptr = _headers )
                {
                    *(long*)(ptr + sizeof( int )) = value;
                }
            }
        }


        public int RefCount
        {
            get { return _refCount; }
        }


        public void AddRef( )
        {
            _refMutex.WaitOne( );
            _refCount++;
            _refMutex.ReleaseMutex( );
        }


        public void RemoveRef( )
        {
            _refMutex.WaitOne( );
            if( --_refCount <= 0 )
            {
                Debug.Assert( _refCount == 0 );
                _refCount = 0;
                _refMutex.ReleaseMutex( );
                MediaType = 0;
                Offset = 0;
                Length = 0;
                Timestamp = 0;
                RecycleRequested?.Invoke( this, EventArgs.Empty );
            }
            else
            {
                _refMutex.ReleaseMutex( );
            }
        }


        public void EnsureCapacity( int value )
        {
            if( Capacity < value )
            {
                Capacity = value;
            }
        }


        public void ReadFrom( BinaryReader reader )
        {
            Debug.Assert( reader != null );
            if( reader != null )
            {
                Offset = 0;
                reader.Read( _headers, 0, _headers.Length );
                EnsureCapacity( Length );
                reader.Read( Data, 0, Length );
            }
        }


        public void WriteTo( BinaryWriter writer )
        {
            Debug.Assert( writer != null );
            if( writer != null )
            {
                writer.Write( _headers );
                writer.Write( Data, Offset, Length - Offset );
            }
        }

    }
}
