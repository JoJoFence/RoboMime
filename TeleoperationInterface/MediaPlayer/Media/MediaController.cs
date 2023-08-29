using MediaPlayer.SocialForceModule;
using NAudio.Wave;
using SSSoftworks.Collections;
using SSSoftworks.Data;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

namespace MediaPlayer.Media
{
    internal sealed class MediaController : INotifyPropertyChanged, IWaveProvider
    {
        private RecyclePool<MediaData> _bufferPool = null;
        private Dictionary<int, Agent> _agentsAll = new Dictionary<int, Agent>( );
        private CircularArray<byte> _audioData = new CircularArray<byte>( 4 * 48000 * (32 / 8) * 2 );
        private Mutex _audioDataMutex = new Mutex( );
        private int _audioSampleCount = 0;
        private IWavePlayer _audioPlayer = null;
        private long _mediaCurrentTimestamp = 0;
        private double _mediaPlaybackSpeed = 1.0;
        private MediaDataReader _mediaReader = null;
        private MediaStopwatch _mediaStopwatch = new MediaStopwatch( );
        private DispatcherTimer _updateTimer = null;
        private BitmapSource _videoSample = null;
        private int _videoSampleCount = 0;


        public MediaController( )
        {
            _agentsAll.Add( 0, new Agent( "Robot" ) );
            _mediaReader = new MediaDataReader( _bufferPool );
            _mediaReader.AudioCaptureFormatChanged += MediaReader_AudioCaptureFormatChanged;
            _mediaReader.DataAvailable += MediaReader_DataAvailable;
            _updateTimer = new DispatcherTimer( );
            _updateTimer.Interval = TimeSpan.FromMilliseconds( 1000.0 / 60.0 );
            _updateTimer.Tick += UpdateTimer_Tick;
            WaveFormat = new WaveFormat( 24000, 16, 1 );
        }


        public event PropertyChangedEventHandler PropertyChanged;


        public event EventHandler<Vector3D> JoystickDataAvailable;


        public event EventHandler<SocialForce> SocialForceComputed;


        public int AudioSampleCount
        {
            get
            {
                int value = _audioSampleCount;
                _audioSampleCount = 0;
                return value;
            }
        }


        public int MediaLength
        {
            get { return _mediaReader.DataCount; }
        }


        public int MediaPosition
        {
            get { return _mediaReader.DataPosition; }
            set
            {
                _mediaReader.DataPosition = value;
                NotifyPropertyChanged( );
            }
        }


        public double PlaybackSpeed
        {
            get { return _mediaPlaybackSpeed; }
            set
            {
                _mediaPlaybackSpeed = Math.Max( 0.0001, value );
                NotifyPropertyChanged( );
            }
        }


        public BitmapSource VideoSample
        {
            get { return _videoSample; }
            private set
            {
                _videoSample = value;
                NotifyPropertyChanged( );
            }
        }


        public int VideoSampleCount
        {
            get
            {
                int value = _videoSampleCount;
                _videoSampleCount = 0;
                return value;
            }
        }


        public WaveFormat WaveFormat
        {
            get;
            private set;
        }


        public void Start( string filename )
        {
            Stop( );
            if( _mediaReader.Open( filename ) )
            {
                _mediaCurrentTimestamp = 0;
                _mediaReader.BaseTimestamp = _mediaReader.FirstMediaTimestamp;
                _updateTimer.Start( );
                _mediaStopwatch.Reset( );
                _mediaStopwatch.Resume( );
            }
            NotifyPropertyChanged( "MediaLength" );
            NotifyPropertyChanged( "MediaPosition" );
        }


        public void Stop( )
        {
            _mediaStopwatch.Pause( );
            _updateTimer.Stop( );
            _mediaReader.Close( );
            if( _audioPlayer != null )
            {
                _audioPlayer.Stop( );
                _audioPlayer.Dispose( );
                _audioPlayer = null;
            }
            _agentsAll.Clear( );
            _agentsAll.Add( 0, new Agent( "Robot" ) );
        }


        int IWaveProvider.Read( byte[] buffer, int offset, int count )
        {
            int nBytes = Math.Min( count, _audioData.Count );
            if( nBytes > 0 )
            {
                _audioDataMutex.WaitOne( );
                nBytes = _audioData.Take( buffer, offset, count );
                _audioDataMutex.ReleaseMutex( );
            }
            else
            {
                nBytes = Math.Min( count, WaveFormat.AverageBytesPerSecond / 20 );
                Array.Clear( buffer, offset, nBytes );
            }
            return nBytes;
        }


        private void MediaReader_AudioCaptureFormatChanged( object sender, WaveFormat e )
        {
            _updateTimer.Dispatcher.BeginInvoke( new Action( ( ) =>
            {
                if( _audioPlayer != null )
                {
                    _audioPlayer.Stop( );
                    _audioPlayer.Dispose( );
                }
                WaveFormat = e;
                _audioPlayer = new WasapiOut( );
                _audioPlayer.Init( this );
                _audioPlayer.Play( );
            } ) );
        }


        private void MediaReader_DataAvailable( object sender, MediaData e )
        {
            switch( e.MediaType )
            {
            case MediaDataReader.MEDIA_TYPE_VIDEO_CAPTURE:
                {
                    //---DATA FORMAT----
                    // UInt32 secs
                    // UInt32 nsecs
                    // UInt16 IMAGE Format
                    // UInt16 Image Width
                    // UInt16 Image Height
                    // UInt16 Pixel Depth
                    // --> IMAGE DATA
                    //------------------
                    int offset = e.Offset + (sizeof( uint ) * 2 + sizeof( ushort ) * 4);
                    int length = e.Length - (sizeof( uint ) * 2 + sizeof( ushort ) * 4);
                    BitmapSource bitmap = null;
                    try
                    {
                        using( MemoryStream ms = new MemoryStream( e.Data, offset, length ) )
                        {
                            BitmapDecoder decoder = new JpegBitmapDecoder( ms, BitmapCreateOptions.None, BitmapCacheOption.OnLoad );
                            decoder.Frames[0].Freeze( );
                            bitmap = decoder.Frames[0];
                        }
                    }
                    catch( Exception ex )
                    {
                        Console.Error.WriteLine( ex.Message );
                        Console.Error.WriteLine( ex.StackTrace );
                    }
                    _videoSampleCount++;
                    VideoSample = bitmap;
                }
                break;
            case MediaDataReader.MEDIA_TYPE_AUDIO_CAPTURE:
                {
                    //---DATA FORMAT----
                    // UInt32 secs
                    // UInt32 nsecs
                    // UInt16 ALSA PCM Format
                    // UInt16 Sample Rate
                    // UInt16 Number of Channels
                    // --> PCM DATA
                    //------------------
                    int offset = e.Offset + (sizeof( uint ) * 2 + sizeof( ushort ) * 3);
                    int length = e.Length - (sizeof( uint ) * 2 + sizeof( ushort ) * 3);
                    _audioDataMutex.WaitOne( );
                    _audioData.Append( e.Data, offset, length );
                    _audioDataMutex.ReleaseMutex( );
                    _audioSampleCount += e.Length / ((WaveFormat.BitsPerSample / 8) * WaveFormat.Channels);
                }
                break;
            case MediaDataReader.MEDIA_TYPE_JOYSTICK:
                {
                    //---DATA FORMAT----
                    // UInt32 secs
                    // UInt32 nsecs
                    // UInt16 dummy
                    // UInt16 dummy
                    // UInt16 dummy
                    // Double X
                    // Double Y
                    // Double Rotation
                    //------------------
                    int offset = e.Offset + (sizeof( uint ) * 2 + sizeof( ushort ) * 3);
                    int length = e.Length - (sizeof( uint ) * 2 + sizeof( ushort ) * 3);
                    double directionX = BitConverter.ToDouble( e.Data, offset + sizeof( double ) * 0 );
                    double directionY = BitConverter.ToDouble( e.Data, offset + sizeof( double ) * 1 );
                    double rotation = BitConverter.ToDouble( e.Data, offset + sizeof( double ) * 2 );
                    JoystickDataAvailable?.Invoke( this, new Vector3D( directionX, directionY, rotation ) );
                }
                break;
            }
        }


        private void NotifyPropertyChanged( [CallerMemberName] string name = "" )
        {
            PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( name ) );
        }


        private void UpdateTimer_Tick( object sender, EventArgs e )
        {
            _mediaCurrentTimestamp += (long)(_mediaStopwatch.ElapsedTimestamp * _mediaPlaybackSpeed + 0.5);
            _mediaReader.Update( _mediaCurrentTimestamp );
            _mediaStopwatch.Reset( );
            NotifyPropertyChanged( "MediaPosition" );
        }


        //private void OnOdometryDataAvailable( object sender, MediaSample e )
        //{
        //}


        //private void OnTrackingDataAvailable( object sender, MediaSample e )
        //{
        //    List<Agent> current = new List<Agent>( );
        //    SocialForce sf = new SocialForce( );
        //    sf.AddAgent( _agentsAll[0] );
        //    int offset = 0;
        //    if( e.Length >= sizeof( int ) * 2 )
        //    {
        //        bool isHuman = BitConverter.ToInt32( e.Data, offset + sizeof( int ) * 0 ) != 0;
        //        int count = BitConverter.ToInt32( e.Data, offset + sizeof( int ) * 1 );
        //        int length = e.Length - sizeof( int ) * 2;
        //        offset += sizeof( int ) * 2;
        //        while( length >= sizeof( int ) * 3 + sizeof( float ) * 7 )
        //        {
        //            int id = BitConverter.ToInt32( e.Data, offset );
        //            length -= sizeof( int ) * 3 + sizeof( float ) * 7;
        //            offset += sizeof( int ) * 3 + sizeof( float ) * 7;
        //            Debug.Assert( _agentsAll.ContainsKey( id ) );
        //            sf.AddAgent( _agentsAll[id] );
        //            current.Add( _agentsAll[id] );
        //        }
        //    }
        //    TrackingDataAvailable?.Invoke( this, current.ToArray( ) );
        //    sf.Update( e.Timestamp / 10000000.0 );
        //    SocialForceComputed?.Invoke( this, sf );
        //}


        //private void OnTrackingDataPreview( object sender, MediaSample e )
        //{
        //    int offset = 0;
        //    if( e.Length >= sizeof( int ) * 2 )
        //    {
        //        bool isHuman = BitConverter.ToInt32( e.Data, offset + sizeof( int ) * 0 ) != 0;
        //        int count = BitConverter.ToInt32( e.Data, offset + sizeof( int ) * 1 );
        //        int length = e.Length - sizeof( int ) * 2;
        //        offset += sizeof( int ) * 2;
        //        while( length >= sizeof( int ) * 3 + sizeof( float ) * 7 )
        //        {
        //            int index = 0;
        //            int id = BitConverter.ToInt32( e.Data, offset + sizeof( int ) * index++ );
        //            int unique_id = BitConverter.ToInt32( e.Data, offset + sizeof( int ) * index++ );
        //            int type = BitConverter.ToInt32( e.Data, offset + sizeof( int ) * index++ );
        //            length -= sizeof( int ) * 3;
        //            offset += sizeof( int ) * 3;

        //            index = 0;
        //            Point3D position;
        //            double orientation;
        //            if( isHuman )
        //            {
        //                position = new Point3D(
        //                    BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ),
        //                    BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ),
        //                    BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ) );
        //                float body_orientation = BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ );
        //                float motion_direction = BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ );
        //                float velocity = BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ );
        //                float head_orientation = BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ );
        //                orientation = body_orientation;
        //            }
        //            else
        //            {
        //                id = 0;
        //                position = new Point3D(
        //                   BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ),
        //                   BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ),
        //                   BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ) );
        //                orientation = new Quaternion(
        //                   BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ),
        //                   BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ),
        //                   BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ),
        //                   BitConverter.ToSingle( e.Data, offset + sizeof( float ) * index++ ) ).Angle;
        //            }
        //            length -= sizeof( float ) * 7;
        //            offset += sizeof( float ) * 7;
        //            if( !_agentsAll.ContainsKey( id ) )
        //            {
        //                _agentsAll.Add( id, new Agent( id.ToString( ) ) );
        //            }
        //            _agentsAll[id].Add( position.X, position.Y, e.Timestamp / 10000000.0 );
        //        }
        //    }
        //}

    }
}
