using AuthenticTeleoperation.Messages;
using NAudio.Wave;
using SSSoftworks.Collections;
using System;
using System.Threading;
using UnityEngine;

namespace AuthenticTeleoperation.Media
{
    internal sealed class AudioStreamPlayer : MonoBehaviour, IWaveProvider
    {
        private CircularArray<byte> _audioData = new CircularArray<byte>( 48000 * (32 / 8) * 2 * 4 );
        private Mutex _audioDataMutex = new Mutex( );
        private int _sampleCount = 0;
        private int _sampleCountDivider = 1;
        private WaveFormat _waveFormat = null;
        private WaveOut _wavePlayer = null;


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


        WaveFormat IWaveProvider.WaveFormat
        {
            get { return _waveFormat; }
        }


        public void CloseDevice( )
        {
            if( _wavePlayer != null )
            {
                _wavePlayer.Stop( );
                _wavePlayer.Dispose( );
                _wavePlayer = null;
            }
            _audioDataMutex.WaitOne( );
            _audioData.Clear( );
            _audioDataMutex.ReleaseMutex( );
        }


        public void OpenDevice( int sampleRate, int bitsPerSample, int channels )
        {
            CloseDevice( );
            _sampleCount = 0;
            _sampleCountDivider = (bitsPerSample / 8) * channels;
            _waveFormat = new WaveFormat( sampleRate, bitsPerSample, channels );
            _wavePlayer = new WaveOut( );
            _wavePlayer.Init( this );
            _wavePlayer.Play( );
            MediaInformation = string.Format( "{0}Hz {1}-bit {2}-channel", sampleRate, bitsPerSample, channels );
        }


        public void Play( MediaBuffer data )
        {
            if( _wavePlayer != null && data != null && data.Length > 0 )
            {
                _audioDataMutex.WaitOne( );
                if( _audioData.Count > _waveFormat.AverageBytesPerSecond )
                {
                    _audioData.Clear( );
                }
                _audioData.Append( data, 0, data.Length );
                _audioDataMutex.ReleaseMutex( );
                _sampleCount += data.Length / _sampleCountDivider;
            }
        }


        int IWaveProvider.Read( byte[] buffer, int offset, int length )
        {
            int nBytes = Math.Min( length, _audioData.Count );
            if( nBytes > 0 )
            {
                _audioDataMutex.WaitOne( );
                nBytes = _audioData.Take( buffer, offset, length );
                _audioDataMutex.ReleaseMutex( );
            }
            else
            {
                nBytes = Math.Min( length, _waveFormat.AverageBytesPerSecond / 20 );
                Array.Clear( buffer, offset, nBytes );
            }
            return nBytes;
        }


        private void OnDisable( )
        {
            CloseDevice( );
        }

    }
}
