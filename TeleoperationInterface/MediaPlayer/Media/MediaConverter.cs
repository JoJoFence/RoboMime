using SSSoftworks.Data;
using SSSoftworks.Media;
using SSSoftworks.Media.Converter;
using SSSoftworks.Media.MediaFoundation;
using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Windows.Documents;
using System.Windows.Media.Imaging;

namespace MediaPlayer.Media
{
    internal sealed class MediaConverter : IImageSampleProvider, INotifyPropertyChanged, IWaveSampleProvider
    {
        private const int IMAGE_FPS = 20;
        private RecyclePool<MediaSample> _samplePool = new RecyclePool<MediaSample>( ( ) => { return new MediaSample( ); } );
        private JpegToBgr32 _cvtJpegToBgr32 = new JpegToBgr32( );
        private BgrToYuy2 _cvtBgrToYuv2 = new BgrToYuy2( );
        private SampleRateConverter _cvtWaveRate = new SampleRateConverter( );

        private bool _dataAudioConfig = false;
        private int _dataAudioChannels = 0;
        private int _dataAudioRate = 0;
        private double _dataAudioFirstTimestamp = long.MinValue;
        private long _dataAudioLastTimestamp = 0;
        private bool _dataVideoConfig = false;
        private int _dataVideoHeight = 0;
        private int _dataVideoWidth = 0;
        private double _dataVideoFirstTimestamp = long.MinValue;
        private long _dataVideoLastTimestamp = 0;

        private string _fileInput = string.Empty;
        private string _fileOutput = string.Empty;
        private long _mediaAudioTimestamp = 0;
        private long _mediaVideoTimestamp = 0;
        private double _progress = 0.0;
        private MediaWriter _mediaWriter = null;
        private ManualResetEvent _workerIsRunning = new ManualResetEvent( false );
        private Thread _worker = null;


        public MediaConverter( )
        {
            _cvtJpegToBgr32.Initialize( this );
        }


        public event PropertyChangedEventHandler PropertyChanged;


        public event EventHandler Started;


        public event EventHandler Stopped;


        public bool IsRunning
        {
            get { return _workerIsRunning.WaitOne( 0 ); }
            private set
            {
                if( value )
                {
                    _workerIsRunning.Set( );
                }
                else
                {
                    _workerIsRunning.Reset( );
                }
                NotifyPropertyChanged( );
            }
        }


        public double Progress
        {
            get { return _progress; }
            private set
            {
                _progress = Math.Max( 0.0, Math.Min( 100.0, value ) );
                NotifyPropertyChanged( );
            }
        }


        public BitmapSource VideoSample
        {
            get { return _cvtBgrToYuv2.Image; }
        }


        Guid IImageSampleProvider.ImageFormat
        {
            get { return MFMediaFormat.MFVideoFormat_MJPG; }
        }


        int IImageSampleProvider.ImageHeight
        {
            get { return _dataVideoHeight; }
        }


        int IImageSampleProvider.ImageWidth
        {
            get { return _dataVideoWidth; }
        }


        Guid IWaveSampleProvider.WaveFormat
        {
            get { return MFMediaFormat.MFAudioFormat_Float; }
        }


        int IWaveSampleProvider.WaveBitsPerSample
        {
            get { return 32; }
        }


        int IWaveSampleProvider.WaveChannels
        {
            get { return _dataAudioChannels; }
        }


        int IWaveSampleProvider.WaveSampleRate
        {
            get { return _dataAudioRate; }
        }


        public void Start( string recfile, string outfile )
        {
            Stop( );
            IsRunning = true;
            _fileInput = recfile;
            _fileOutput = outfile;
            _worker = new Thread( WorkerThreadMain );
            _worker.IsBackground = true;
            _worker.Start( );
        }


        public void Stop( )
        {
            if( _worker != null )
            {
                IsRunning = false;
                _worker.Join( );
                _worker = null;
            }
        }


        private void NotifyPropertyChanged( [CallerMemberName] string name = "" )
        {
            PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( name ) );
        }


        private void UpdateAudioConfiguration( int sampleRate, int channels )
        {
            if( !_dataAudioConfig )
            {
                _dataAudioConfig = true;
                _dataAudioChannels = channels;
                _dataAudioRate = sampleRate;
                _cvtWaveRate.Initialize( this );
                _cvtWaveRate.WaveSampleRate = 48000;
                _mediaWriter.ConfigureAudio( new AudioFormat( MFMediaFormat.MFAudioFormat_PCM, 48000, 16, _dataAudioChannels ), new AudioFormat( MFMediaFormat.MFAudioFormat_AAC, 48000, 16, _dataAudioChannels ) );
            }
            if( !_mediaWriter.IsRunning && _dataAudioConfig && _dataVideoConfig )
            {
                _mediaWriter.Start( _fileOutput );
            }
        }


        private void UpdateVideoConfiguration( int width, int height )
        {
            if( !_dataVideoConfig )
            {
                _dataVideoConfig = true;
                _dataVideoWidth = width;
                _dataVideoHeight = height;
                _cvtBgrToYuv2.Initialize( _cvtJpegToBgr32 );
                _mediaWriter.ConfigureVideo( new VideoFormat( MFMediaFormat.MFVideoFormat_YUY2, _dataVideoWidth, _dataVideoHeight, IMAGE_FPS ), new VideoFormat( MFMediaFormat.MFVideoFormat_H264, _dataVideoWidth, _dataVideoHeight, IMAGE_FPS ) );
            }
            if( !_mediaWriter.IsRunning && _dataAudioConfig && _dataVideoConfig )
            {
                _mediaWriter.Start( _fileOutput );
            }
        }


        private void WorkerThreadMain( )
        {
            Progress = 0.0;
            Started?.Invoke( this, EventArgs.Empty );

            _dataAudioConfig = false;
            _dataAudioFirstTimestamp = double.NaN;
            _dataAudioLastTimestamp = 0;
            _dataVideoConfig = false;
            _dataVideoFirstTimestamp = double.NaN;
            _dataVideoLastTimestamp = 0;
            _mediaAudioTimestamp = 0;
            _mediaVideoTimestamp = 0;
            _mediaWriter = new MediaWriter( );
            _mediaWriter.Clear( );

            MediaDataReader reader = new MediaDataReader( );
            reader.DataAvailable += WorkerProcessData;
            reader.Open( _fileInput );
            long currTimestamp = 0L;
            long endTimestamp = reader.LastMediaTimestamp + 10000L;
            long timeSteps = Math.Max( 10000000L, endTimestamp / 10000L );
            while( _workerIsRunning.WaitOne( 0 ) && currTimestamp <= endTimestamp )
            {
                currTimestamp += timeSteps;
                reader.Update( currTimestamp );
                Progress = currTimestamp * 100.0 / endTimestamp;
                NotifyPropertyChanged( "VideoSample" );
                Thread.Yield( );
                Thread.Sleep( 0 );
            }
            reader.Close( );

            _mediaWriter.Stop( );

            IsRunning = false;
            Progress = 100.0;
            Stopped?.Invoke( this, EventArgs.Empty );
        }


        private void WorkerProcessData( object sender, MediaData e )
        {
            switch( e.MediaType )
            {
            case MediaDataReader.MEDIA_TYPE_VIDEO_CAPTURE:
                {
                    uint sec = BitConverter.ToUInt32( e.Data, e.Offset + sizeof( uint ) * 0 );
                    uint nsec = BitConverter.ToUInt32( e.Data, e.Offset + sizeof( uint ) * 1 );
                    double time = sec + nsec * 10e-9D;
                    if( double.IsNaN( _dataVideoFirstTimestamp ) )
                    {
                        _dataVideoFirstTimestamp = time;
                    }
                    long timestamp = (long)((time - _dataVideoFirstTimestamp) * 1000.0 * 10000.0 + 0.5);
                    long duration = Math.Max( 0, timestamp - _dataVideoLastTimestamp );
                    _dataVideoLastTimestamp = timestamp;

                    int offset = e.Offset + (sizeof( uint ) * 2 + sizeof( ushort ) * 4);
                    int length = e.Length - (sizeof( uint ) * 2 + sizeof( ushort ) * 4);
                    MediaSample jpeg = _samplePool.Dequeue( );
                    MediaSample bgr = _samplePool.Dequeue( );
                    MediaSample yuv = _samplePool.Dequeue( );
                    jpeg.EnsureCapacity( length );
                    jpeg.Length = length;
                    //jpeg.Timestamp = timestamp;
                    //jpeg.Duration = duration;
                    jpeg.Timestamp = _mediaVideoTimestamp;
                    jpeg.Duration = 10000000L / IMAGE_FPS;
                    jpeg.StreamIndex = 0;
                    _mediaVideoTimestamp += jpeg.Duration;
                    Buffer.BlockCopy( e.Data, offset, jpeg.Data, 0, jpeg.Length );
                    if( _cvtJpegToBgr32.Convert( jpeg, bgr ) )
                    {
                        UpdateVideoConfiguration( _cvtJpegToBgr32.ImageWidth, _cvtJpegToBgr32.ImageHeight );
                        if( _cvtBgrToYuv2.Convert( bgr, yuv ) )
                        {
                            _mediaWriter.WriteVideoSample( yuv );
                        }
                    }
                    jpeg.RemoveRef( );
                    bgr.RemoveRef( );
                    yuv.RemoveRef( );
                }
                break;
            case MediaDataReader.MEDIA_TYPE_AUDIO_CAPTURE:
                {
                    uint sec = BitConverter.ToUInt32( e.Data, e.Offset + sizeof( uint ) * 0 );
                    uint nsec = BitConverter.ToUInt32( e.Data, e.Offset + sizeof( uint ) * 1 );
                    double time = sec + nsec * 10e-9D;
                    if( double.IsNaN( _dataAudioFirstTimestamp ) )
                    {
                        _dataAudioFirstTimestamp = time;
                    }
                    long timestamp = (long)((time - _dataAudioFirstTimestamp) * 1000.0 * 10000.0 + 0.5);
                    long duration = Math.Max( 0, timestamp - _dataAudioLastTimestamp );
                    _dataAudioLastTimestamp = timestamp;

                    int sampleRate = BitConverter.ToUInt16( e.Data, e.Offset + sizeof( uint ) * 2 + sizeof( ushort ) * 1 );
                    int channels = BitConverter.ToUInt16( e.Data, e.Offset + sizeof( uint ) * 2 + sizeof( ushort ) * 2 );
                    UpdateAudioConfiguration( sampleRate, channels );

                    int offset = e.Offset + (sizeof( uint ) * 2 + sizeof( ushort ) * 3);
                    int length = e.Length - (sizeof( uint ) * 2 + sizeof( ushort ) * 3);
                    MediaSample floatIn = _samplePool.Dequeue( );
                    MediaSample floatOut = _samplePool.Dequeue( );
                    MediaSample pcm = _samplePool.Dequeue( );
                    ConvertPcm16ToFloat( floatIn, e.Data, offset, length );
                    //floatIn.Timestamp = timestamp;
                    //floatIn.Duration = duration;
                    double AVERAGE_BYTES_PER_MILLISECOND = (_dataAudioRate * (16 / 8) * _dataAudioChannels) / 1000.0;
                    double DURATION_PER_BYTE = 1.0 / AVERAGE_BYTES_PER_MILLISECOND;
                    floatIn.Timestamp = _mediaAudioTimestamp;
                    floatIn.Duration = (long)(length * DURATION_PER_BYTE) * 10000L;
                    floatIn.StreamIndex = 0;
                    _mediaAudioTimestamp += floatIn.Duration;
                    if( _cvtWaveRate.Convert( floatIn, floatOut ) )
                    {
                        ConvertFloatToPcm( floatOut, pcm );
                        _mediaWriter.WriteAudioSample( pcm );
                    }
                    floatIn.RemoveRef( );
                    floatOut.RemoveRef( );
                    pcm.RemoveRef( );
                }
                break;
            case MediaDataReader.MEDIA_TYPE_JOYSTICK:
            case MediaDataReader.MEDIA_TYPE_ODOMETRY:
            case MediaDataReader.MEDIA_TYPE_TRACKING:
                break;
            case MediaDataReader.MEDIA_TYPE__LENGTH:
            default:
                Console.WriteLine(_fileInput );
                break;
            }
        }


        private static unsafe void ConvertPcm16ToFloat( MediaSample sample, byte[] buffer, int offset, int length )
        {
            sample.EnsureCapacity( length * sizeof( float ) / sizeof( short ) );
            sample.Length = length * sizeof( float ) / sizeof( short );
            fixed( byte* pSRC = &buffer[offset], pDST = sample.Data )
            {
                short* src = (short*)pSRC;
                float* dst = (float*)pDST;
                for( int i = 0; i < length; i += sizeof( short ), ++dst, ++src )
                {
                    *dst = *src / 32768.0f;
                }
            }
        }


        private static unsafe void ConvertFloatToPcm( MediaSample inSample, MediaSample outSample )
        {
            outSample.EnsureCapacity( inSample.Length * sizeof( short ) / sizeof( float ) );
            outSample.Length = inSample.Length * sizeof( short ) / sizeof( float );
            outSample.Timestamp = inSample.Timestamp;
            outSample.Duration = inSample.Duration;
            outSample.StreamIndex = inSample.StreamIndex;
            fixed( byte* pSRC = inSample.Data, pDST = outSample.Data )
            {
                float* src = (float*)pSRC;
                short* dst = (short*)pDST;
                for( int i = 0; i < inSample.Length; i += sizeof( float ), ++dst, ++src )
                {
                    *dst = (short)(*src * 32768.0f);
                }
            }
        }

    }
}
