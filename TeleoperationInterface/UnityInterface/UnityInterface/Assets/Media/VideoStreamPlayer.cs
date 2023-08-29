using AuthenticTeleoperation.Messages;
using System;
using System.Threading;
using UnityEngine;

namespace AuthenticTeleoperation.Media
{
    internal class VideoStreamPlayer : MonoBehaviour
    {
        private int _pixelHeight = 0;
        private int _pixelWidth = 0;
        private int _sampleCount = 0;
        private Texture2D _texture = null;
        private byte[] _videoSample = null;
        private AutoResetEvent _videoSampleUpdated = new AutoResetEvent( false );
        public Renderer uiRenderer = null;


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


        public void Play( MediaBuffer data )
        {
            if( data != null && data.Length > 0 )
            {
                byte[] buffer = new byte[data.Length];
                Buffer.BlockCopy( data.Data, 0, buffer, 0, buffer.Length );
                _videoSample = buffer;
                _videoSampleUpdated.Set( );
            }
        }


        private void Start( )
        {
            _texture = new Texture2D( 640, 360 );
            if( uiRenderer != null )
            {
                uiRenderer.material.mainTexture = _texture;
            }
        }


        private void Update( )
        {
            if( _videoSampleUpdated.WaitOne( 0 ) )
            {
                byte[] buffer = _videoSample;
                _sampleCount++;
                _texture.LoadImage( buffer );
                if( _pixelWidth != _texture.width || _pixelHeight != _texture.height )
                {
                    _pixelWidth = _texture.width;
                    _pixelHeight = _texture.height;
                    MediaInformation = string.Format( "{0}x{1} pixels", _pixelWidth, _pixelHeight );
                }
            }
        }

    }
}
