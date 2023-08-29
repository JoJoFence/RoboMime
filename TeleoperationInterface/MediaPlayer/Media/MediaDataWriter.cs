using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;

namespace MediaPlayer.Media
{
    public sealed class MediaDataWriter : IDisposable
    {
        private Queue<MediaData> _data = new Queue<MediaData>( );
        private Mutex _dataMutex = new Mutex( );
        private string _fileName = string.Empty;
        private bool _threadQuitImmediate = false;
        private bool _threadRunning = false;
        private Thread _thread = null;


        public MediaDataWriter( string name = null )
        {
            AudioFormat = Guid.Empty;
            Name = name ?? string.Empty;
            VideoFormat = Guid.Empty;
        }


        public Guid AudioFormat
        {
            get;
            private set;
        }


        public string Name
        {
            get;
            private set;
        }


        public Guid VideoFormat
        {
            get;
            private set;
        }


        public void Dispose( )
        {
            Stop( );
            Clear( );
        }


        public void Clear( )
        {
            _dataMutex.WaitOne( );
            while( _data.Count > 0 )
            {
                MediaData data = _data.Dequeue( );
                _dataMutex.ReleaseMutex( );
                data.RemoveRef( );
                _dataMutex.WaitOne( );
            }
            _dataMutex.ReleaseMutex( );
        }


        public bool Start( string filename, Guid video, Guid audio )
        {
            if( !_threadRunning )
            {
                Clear( );
                VideoFormat = video;
                AudioFormat = audio;
                _fileName = filename;
                _threadRunning = true;
                _thread = new Thread( WorkerThreadMain );
                _thread.IsBackground = true;
                _thread.Name = Name;
                _thread.Start( );
                return true;
            }
            return false;
        }


        public void Stop( bool quitImmediate = false )
        {
            _threadQuitImmediate = quitImmediate;
            _threadRunning = false;
            if( _thread != null )
            {
                _thread.Join( );
                _thread = null;
            }
        }


        public void Write( MediaData data )
        {
            if( _threadRunning )
            {
                data.AddRef( );
                _dataMutex.WaitOne( );
                _data.Enqueue( data );
                _dataMutex.ReleaseMutex( );
            }
        }


        private void WorkerThreadMain( )
        {
            BinaryWriter writer = null;
            try
            {
                writer = new BinaryWriter( File.Create( _fileName ) );
                writer.Write( VideoFormat.ToByteArray( ) );
                writer.Write( AudioFormat.ToByteArray( ) );
            }
            catch( Exception ex )
            {
                Console.Error.WriteLine( ex.Message );
            }
            if( writer != null )
            {
                while( _threadRunning )
                {
                    if( _data.Count > 0 )
                    {
                        _dataMutex.WaitOne( );
                        if( _data.Count > 0 )
                        {
                            MediaData data = _data.Dequeue( );
                            _dataMutex.ReleaseMutex( );
                            data.WriteTo( writer );
                            data.RemoveRef( );
                        }
                        else
                        {
                            _dataMutex.ReleaseMutex( );
                        }
                    }
                    Thread.Yield( );
                    Thread.Sleep( 0 );
                }
                while( !_threadQuitImmediate && _data.Count > 0 )
                {
                    _dataMutex.WaitOne( );
                    if( _data.Count > 0 )
                    {
                        MediaData data = _data.Dequeue( );
                        _dataMutex.ReleaseMutex( );
                        data.WriteTo( writer );
                        data.RemoveRef( );
                    }
                    else
                    {
                        _dataMutex.ReleaseMutex( );
                    }
                    Thread.Yield( );
                    Thread.Sleep( 0 );
                }
                writer.Close( );
                writer.Dispose( );
            }
        }

    }
}
