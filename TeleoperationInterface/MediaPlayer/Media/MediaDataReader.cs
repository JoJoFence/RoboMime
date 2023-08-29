using NAudio.Wave;
using SSSoftworks.Data;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;

namespace MediaPlayer.Media
{
    public sealed class MediaDataReader : INotifyPropertyChanged
    {
        private const int ALSA_PCM_FORMAT_S16_LE = 2;
        public const byte MEDIA_TYPE_VIDEO_CAPTURE = 0;
        public const byte MEDIA_TYPE_AUDIO_CAPTURE = 1;
        public const byte MEDIA_TYPE_AUDIO_PLAYBACK = 2;
        public const byte MEDIA_TYPE_JOYSTICK = 2;
        public const byte MEDIA_TYPE_ODOMETRY = 3;
        public const byte MEDIA_TYPE_TRACKING = 4;
        public const byte MEDIA_TYPE__LENGTH = 5;
        private delegate void DataHandler( MediaData data );
        private RecyclePool<MediaData> _bufferPool = null;
        private Dictionary<int, DataHandler> _handler = new Dictionary<int, DataHandler>( );
        private long _baseTimestamp = 0;
        private BinaryReader _file = null;
        private List<DataEntry> _fileEntries = new List<DataEntry>( );
        private int _fileEntryIndex = 0;
        private string _fileName = string.Empty;
        private int _pcmSampleRate = 0;
        private int _pcmChannels = 0;


        public MediaDataReader( RecyclePool<MediaData> bufferPool = null )
        {
            _bufferPool = bufferPool ?? new RecyclePool<MediaData>( ( ) => { return new MediaData( ); } );
            _handler.Add( MEDIA_TYPE_VIDEO_CAPTURE, ProcessOtherData );
            _handler.Add( MEDIA_TYPE_AUDIO_CAPTURE, ProcessAudioData );
            _handler.Add( MEDIA_TYPE_JOYSTICK, ProcessOtherData );
            _handler.Add( MEDIA_TYPE_ODOMETRY, ProcessOtherData );
            _handler.Add( MEDIA_TYPE_TRACKING, ProcessOtherData );
        }


        public event EventHandler<MediaData> DataAvailable;


        public event EventHandler<WaveFormat> AudioCaptureFormatChanged;


        public event EventHandler<WaveFormat> AudioPlaybackFormatChanged;


        public event PropertyChangedEventHandler PropertyChanged;


        public long BaseTimestamp
        {
            get { return _baseTimestamp; }
            set
            {
                _baseTimestamp = value;
                NotifyPropertyChanged( );
            }
        }


        public long TimestampOffset
        {
            get { return _baseTimestamp; }
            set
            {
                _baseTimestamp = value;
                NotifyPropertyChanged( );
            }
        }


        public int DataCount
        {
            get { return _fileEntries.Count; }
        }


        public int DataPosition
        {
            get { return _fileEntryIndex; }
            set
            {
                MediaData dummy = CreateDummy( MEDIA_TYPE_JOYSTICK, sizeof( uint ) * 2 + sizeof( double ) * 3 );
                DataAvailable?.Invoke( this, dummy );
                dummy.RemoveRef( );

                DataEntry[] latestData = new DataEntry[MEDIA_TYPE__LENGTH];
                int index = -1;
                value = Math.Max( 0, Math.Min( _fileEntries.Count, value ) );
                while( ++index < _fileEntries.Count && index != value )
                {
                    DataEntry entry = _fileEntries[index];
                    latestData[entry.MediaType] = entry;
                }
                foreach( DataEntry entry in latestData )
                {
                    MediaData data = _bufferPool.Dequeue( );
                    try
                    {
                        _file.BaseStream.Position = entry.FilePosition;
                        data.ReadFrom( _file );
                    }
                    catch( Exception ex )
                    {
                        Console.Error.WriteLine( ex.Message );
                        data?.RemoveRef( );
                        data = null;
                    }
                    if( data != null )
                    {
                        if( _handler.ContainsKey( data.MediaType ) )
                        {
                            _handler[data.MediaType]( data );
                        }
                        data.RemoveRef( );
                    }
                }
                _fileEntryIndex = index;
                try
                {
                    _file.BaseStream.Position = _fileEntryIndex < _fileEntries.Count ? _fileEntries[_fileEntryIndex].FilePosition : _file.BaseStream.Length;
                }
                catch( Exception ex )
                {
                    Console.Error.WriteLine( ex.Message );
                }
                NotifyPropertyChanged( );
            }
        }


        public long DataTimestamp
        {
            get;
            private set;
        }


        public FileInfo FileInfo
        {
            get;
            private set;
        }


        public string FileName
        {
            get { return _fileName; }
            set
            {
                if( _file == null )
                {
                    _fileName = value ?? string.Empty;
                    FileInfo = new FileInfo( _fileName );
                    NotifyPropertyChanged( );
                    NotifyPropertyChanged( "FileInfo" );
                }
            }
        }


        public long FirstMediaTimestamp
        {
            get;
            private set;
        }


        public long LastMediaTimestamp
        {
            get;
            private set;
        }


        public Guid MediaAudioFormat
        {
            get;
            private set;
        }


        public Guid MediaVideoFormat
        {
            get;
            private set;
        }


        public void Close( )
        {
            _fileEntries.Clear( );
            _fileEntryIndex = 0;
            if( _file != null )
            {
                _file.Close( );
                _file.Dispose( );
                _file = null;
            }
            _pcmSampleRate = 0;
            _pcmChannels = 0;
            DataTimestamp = 0;
            FirstMediaTimestamp = 0;
            LastMediaTimestamp = 0;
            NotifyPropertyChanged( "DataCount" );
            NotifyPropertyChanged( "DataPosition" );
            NotifyPropertyChanged( "DataTimestamp" );
            NotifyPropertyChanged( "FirstMediaTimestamp" );
            NotifyPropertyChanged( "FirstROSTimestamp" );
            NotifyPropertyChanged( "LastMediaTimestamp" );
            NotifyPropertyChanged( "LastROSTimestamp" );
        }


        public bool Open( string filename )
        {
            Close( );
            FileName = filename;
            try
            {
                byte[] guid = new byte[16];
                _file = new BinaryReader( FileInfo.Open( FileMode.Open ) );
                _file.Read( guid, 0, guid.Length );
                MediaVideoFormat = new Guid( guid );
                _file.Read( guid, 0, guid.Length );
                MediaAudioFormat = new Guid( guid );
                ReadFileInformation( );
                return true;
            }
            catch( Exception ex )
            {
                Console.Error.WriteLine( ex.Message );
            }
            return false;
        }


        public void Update( long mediaTimestamp )
        {
            while( _fileEntryIndex < _fileEntries.Count && _fileEntries[_fileEntryIndex].MediaTimestamp <= mediaTimestamp )
            {
                DataEntry entry = _fileEntries[_fileEntryIndex++];
                MediaData data = _bufferPool.Dequeue( );
                try
                {
                    _file.BaseStream.Position = entry.FilePosition;
                    data.ReadFrom( _file );
                }
                catch( Exception ex )
                {
                    Console.Error.WriteLine( ex.Message );
                    data?.RemoveRef( );
                    data = null;
                }
                DataTimestamp = entry.MediaTimestamp;
                if( data != null )
                {
                    if( _handler.ContainsKey( data.MediaType ) )
                    {
                        _handler[data.MediaType]( data );
                    }
                    data.RemoveRef( );
                }
            }
            NotifyPropertyChanged( "DataPosition" );
            NotifyPropertyChanged( "DataTimestamp" );
        }


        private MediaData CreateDummy( int mediaType, int length )
        {
            MediaData dummy = _bufferPool.Dequeue( );
            dummy.EnsureCapacity( length );
            dummy.Length = length;
            dummy.MediaType = mediaType;
            dummy.Timestamp = 0;
            return dummy;
        }


        private void NotifyPropertyChanged( [CallerMemberName] string name = "" )
        {
            PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( name ) );
        }


        private void ProcessAudioData( MediaData data )
        {
            if( data.Length > 0 )
            {
                int bits = BitConverter.ToUInt16( data.Data, sizeof( uint ) * 2 + sizeof( ushort ) * 0 );
                Debug.Assert( bits == ALSA_PCM_FORMAT_S16_LE );
                bits = 16;
                int sampleRate = BitConverter.ToUInt16( data.Data, sizeof( uint ) * 2 + sizeof( ushort ) * 1 );
                int channels = BitConverter.ToUInt16( data.Data, sizeof( uint ) * 2 + sizeof( ushort ) * 2 );
                if( _pcmSampleRate != sampleRate || _pcmChannels != channels )
                {
                    _pcmSampleRate = sampleRate;
                    _pcmChannels = channels;
                    AudioCaptureFormatChanged?.Invoke( this, new WaveFormat( _pcmSampleRate, bits, _pcmChannels ) );
                }
                DataAvailable?.Invoke( this, data );
            }
        }


        private void ProcessOtherData( MediaData data )
        {
            if( data.Length > 0 )
            {
                DataAvailable?.Invoke( this, data );
            }
        }


        private void ReadFileInformation( )
        {
            _fileEntries.Clear( );
            _fileEntryIndex = 0;
            if( _file.BaseStream.Position < _file.BaseStream.Length )
            {
                long firstPosition = _file.BaseStream.Position;
                int mediaType = _file.ReadInt32( );
                long mediaTimestamp = FirstMediaTimestamp = _file.ReadInt64( );
                int length = _file.ReadInt32( );
                _file.BaseStream.Position += length;
                _fileEntries.Add( new DataEntry( ) { FilePosition = firstPosition, MediaType = mediaType, MediaTimestamp = mediaTimestamp, MediaLength = length } );
                while( _file.BaseStream.Position < _file.BaseStream.Length )
                {
                    long position = _file.BaseStream.Position;
                    mediaType = _file.ReadInt32( );
                    mediaTimestamp = _file.ReadInt64( );
                    length = _file.ReadInt32( );

                    _file.BaseStream.Position += length;
                    _fileEntries.Add( new DataEntry( ) { FilePosition = position, MediaType = mediaType, MediaTimestamp = mediaTimestamp, MediaLength = length } );
                }
                LastMediaTimestamp = mediaTimestamp;
                _file.BaseStream.Position = firstPosition; // go back to where we were

                NotifyPropertyChanged( "FirstMediaTimestamp" );
                NotifyPropertyChanged( "FirstROSTimestamp" );
                NotifyPropertyChanged( "LastMediaTimestamp" );
                NotifyPropertyChanged( "LastROSTimestamp" );
            }
            NotifyPropertyChanged( "DataCount" );
            NotifyPropertyChanged( "DataPosition" );
        }


        private sealed class DataEntry
        {

            public long FilePosition
            {
                get;
                set;
            }


            public int MediaType
            {
                get;
                set;
            }


            public long MediaTimestamp
            {
                get;
                set;
            }


            public int MediaLength
            {
                get;
                set;
            }

        }

    }
}