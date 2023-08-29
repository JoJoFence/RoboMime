using AuthenticTeleoperation.Messages;
using NAudio.Wave;
using System;
using UnityEngine;

namespace AuthenticTeleoperation.Media
{
    internal sealed class AudioStreamCapture : MonoBehaviour
    {
        private MediaBufferPool _bufferPool = null;
        private bool _bufferPoolOwner = false;
        private int _sampleCount = 0;
        private int _sampleCountDivider = 1;
        private WaveFormat _waveFormat = null;
        private WaveIn _waveIn = null;


        public event EventHandler<MediaBuffer> DataAvailable;


        public MediaBufferPool BufferPool
        {
            get { return _bufferPool; }
            set
            {
                if( _bufferPoolOwner )
                {
                    _bufferPool.Clear( );
                }
                _bufferPool = value;
                if( _bufferPool != null )
                {
                    _bufferPoolOwner = false;
                }
                else
                {
                    _bufferPool = new MediaBufferPool( ( ) => { return new MediaBuffer( ); } );
                    _bufferPoolOwner = true;
                }
            }
        }


        public string MediaInformation
        {
            get;
            private set;
        } = "None";


        public int SampleCount
        {
            get
            {
                int value = _sampleCount;
                _sampleCount = 0;
                return value;
            }
        }


        public void CloseDevice( )
        {
            if( _waveIn != null )
            {
                _waveIn.StopRecording( );
                _waveIn.Dispose( );
                _waveIn.DataAvailable -= WaveIn_DataAvailable;
                _waveIn = null;
            }
            if( _bufferPoolOwner )
            {
                _bufferPool.Clear( );
            }
        }


        public void OpenDevice( int sampleRate, int bitsPerSample, int channels )
        {
            CloseDevice( );
            _sampleCount = 0;
            _sampleCountDivider = (bitsPerSample / 8) * channels;
            _waveFormat = new WaveFormat( sampleRate, bitsPerSample, channels );
            _waveIn = new WaveIn( );
            _waveIn.WaveFormat = _waveFormat;
            _waveIn.DataAvailable += WaveIn_DataAvailable;
            _waveIn.StartRecording( );
            MediaInformation = string.Format( "{0}Hz {1}-bit {2}-channel", sampleRate, bitsPerSample, channels );
        }


        private void Start( )
        {
            _bufferPool = new MediaBufferPool( ( ) => { return new MediaBuffer( ); } );
            _bufferPoolOwner = true;
        }


        private void OnDisable( )
        {
            CloseDevice( );
        }


        private void WaveIn_DataAvailable( object sender, WaveInEventArgs e )
        {
            if( e.BytesRecorded > 0 )
            {
                MediaBuffer data = _bufferPool.Dequeue( );
                data.EnsureCapacity( e.BytesRecorded );
                data.Length = e.BytesRecorded;
                Buffer.BlockCopy( e.Buffer, 0, data, 0, e.BytesRecorded );
                _sampleCount += e.BytesRecorded / _sampleCountDivider;
                DataAvailable?.Invoke( this, data );
                data.RemoveRef( );
            }
        }

    }
}
