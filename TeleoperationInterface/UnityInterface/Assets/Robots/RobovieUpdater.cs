using AuthenticTeleoperation.Configurations;
using AuthenticTeleoperation.Media;
using AuthenticTeleoperation.Messages;
using AuthenticTeleoperation.Messages.Audio;
using AuthenticTeleoperation.Messages.Geometry;
using AuthenticTeleoperation.Messages.Layer2;
using AuthenticTeleoperation.Messages.Sensor;
using SS.HRIKU.Network.Tcp;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace AuthenticTeleoperation.Robots
{
    internal sealed class RobovieUpdater : MonoBehaviour
    {
        private const int NETWORK_PORT = 9407;
        private RobovieJoint[] _joints = null;
        private double[] _jointsAngles = null;
        private MediaBufferPool _mediaBufferPool = null;
        private NetServer _poseReceiver = null;
        private Robovie _robot = null;
        private bool _talkPushToTalk = false;

        public RobovieJoint uiShoulderPitchR = null;
        public RobovieJoint uiShoulderRollR = null;
        public RobovieJoint uiElbowYawR = null;
        public RobovieJoint uiElbowPitchR = null;
        public RobovieJoint uiShoulderPitchL = null;
        public RobovieJoint uiShoulderRollL = null;
        public RobovieJoint uiElbowYawL = null;
        public RobovieJoint uiElbowPitchL = null;
        public RobovieJoint uiNeckYaw = null;
        public RobovieJoint uiHeadPitch = null;
        public RobovieJoint uiHeadRoll = null;
        public TextMeshPro uiTalkInformation = null;

        public VideoStreamPlayer mVideoPlayer = null;
        public AudioStreamPlayer mAudioPlayer = null;
        public AudioStreamCapture mAudioCapture = null;


        internal void UpdateJointAngles( double[] angles )
        {
            _jointsAngles = angles;
        }


        private void Awake( )
        {
            _mediaBufferPool = new MediaBufferPool( ( ) => { return new MediaBuffer( ); } );
        }


        private void Start( )
        {
            AppConfiguration config = AppConfiguration.Instance;
            _joints = new RobovieJoint[(int)RobovieJointType._LENGTH] { uiShoulderPitchR, uiShoulderRollR, uiElbowYawR, uiElbowPitchR, uiShoulderPitchL, uiShoulderRollL, uiElbowYawL, uiElbowPitchL, uiNeckYaw, uiHeadRoll, uiHeadPitch };
            for( int i = 0; i < _joints.Length; ++i )
            {
                Debug.Assert( _joints[i] != null );
                _joints[i].ID = i;
            }
            SetRobovieJointParameters( config.Joints );
            _robot = new Robovie( );
            _robot.HostAddress = config.HostAddress;
            _robot.HostPort = config.HostPort;
            _robot.AddConnection( new VideoCapture( _mediaBufferPool ) );
            _robot.AddConnection( new AudioCapture( _mediaBufferPool ) );
            _robot.AddConnection( new AudioPlayback( ) );
            _robot.AddConnection( new RobotCmdVel( ) );
            _robot.AddConnection( new RobotOdometry( ) );
            _robot.AddConnection( new RobotPose6D( ) );
            _robot.AddConnection( new HumanTrackedL2( ) );
            _robot.AddConnection( new VelodynePoints( ) );
            _robot.AddMessageHandler( Robovie.CTYPE_VIDEO_CAPTURE, RobotHandler_VideoCapture );
            _robot.AddMessageHandler( Robovie.CTYPE_AUDIO_CAPTURE, RobotHandler_AudioCapture );
            _robot.AddMessageHandler( Robovie.CTYPE_ROBOT_POSE_6D, RobotHandler_RobotTracking );
            _robot.AddMessageHandler( Robovie.CTYPE_HUMAN_TRACKED_L2, RobotHandler_HumanTracking );
            _robot.AddMessageHandler( Robovie.CTYPE_VELODYNE_POINTS, RobotHandler_VelodynePoints );
            _robot.Connect( );
            _poseReceiver = new NetServer( );
            _poseReceiver.AddMessageHandler( new JointMessageHandler( this ) );
            _poseReceiver.Start( NETWORK_PORT );
#if !UNITY_EDITOR && UNITY_STANDALONE_WIN
            if( mAudioPlayer != null )
            {
                mAudioPlayer.OpenDevice( config.AudioSampleRate, config.AudioBitsPerSample, config.AudioChannels );
            }
            if( mAudioCapture != null )
            {
                mAudioCapture.BufferPool = _mediaBufferPool;
                mAudioCapture.DataAvailable += AudioCapture_DataAvailable;
                mAudioCapture.OpenDevice( config.AudioSampleRate, config.AudioBitsPerSample, config.AudioChannels );
            }
#endif
        }


        private void OnApplicationQuit( )
        {
            if( _robot != null )
            {
                _robot.Disconnect( );
                _robot = null;
            }
            if( _poseReceiver != null )
            {
                _poseReceiver.Stop( );
                _poseReceiver = null;
            }
#if !UNITY_EDITOR && UNITY_STANDALONE_WIN
            if( mAudioPlayer != null )
            {
                mAudioPlayer.CloseDevice( );
            }
            if( mAudioCapture != null )
            {
                mAudioCapture.CloseDevice( );
                mAudioCapture.DataAvailable -= AudioCapture_DataAvailable;
            }
#endif
        }


        private void Update( )
        {
            if( Input.GetAxis( "Jump" ) > 0.0f )
            {
                uiTalkInformation.text = "talking. . .";
                _talkPushToTalk = true;
            }
            else if( _talkPushToTalk )
            {
                uiTalkInformation.text = string.Empty;
                _talkPushToTalk = false;
            }
            if( _jointsAngles != null )
            {
                double[] values = _jointsAngles;
                _jointsAngles = null;
                Debug.Assert( _joints.Length == values.Length );
                for( int i = 0; i < _joints.Length; ++i )
                {
                    if( !double.IsInfinity( values[i] ) && !double.IsNaN( values[i] ) )
                    {
                        _joints[i].Angle = values[i];
                    }
                }
            }
        }


        private void SetRobovieJointParameters( IList<JointConfiguration> joints )
        {
            if( _joints != null )
            {
                Debug.Assert( joints.Count == (int)RobovieJointType._LENGTH );
                for( int i = 0; i < _joints.Length; ++i )
                {
                    _joints[i].ConversionGain = joints[i].ConversionGain;
                    _joints[i].ConversionOffset = joints[i].ConversionOffset;
                    _joints[i].MaxAngle = joints[i].MaxAngle;
                    _joints[i].MinAngle = joints[i].MinAngle;
                    _joints[i].AngleOffset = joints[i].AngleOffset;
                    _joints[i].PoseValue = joints[i].DefaultPoseValue;
                }
            }
        }


        private void AudioCapture_DataAvailable( object sender, MediaBuffer e )
        {
            if( _talkPushToTalk )
            {
                _robot.SendMessage( Robovie.CTYPE_AUDIO_PLAYBACK, new AudioData( e ) );
            }
        }


        private void RobotHandler_VideoCapture( object sender, Message message )
        {
            var data = message as CompressedImage;
            Debug.Assert( data != null );
            mVideoPlayer.Play( data.Data );
        }


        private void RobotHandler_AudioCapture( object sender, Message message )
        {
            var data = message as AudioData;
            Debug.Assert( data != null );
            mAudioPlayer.Play( data.Data );
        }


        private void RobotHandler_RobotTracking( object sender, Message message )
        {
            var data = message as PoseWithCovarianceStamped;
            Debug.Assert( data != null );
        }


        private void RobotHandler_HumanTracking( object sender, Message message )
        {
            var data = message as HTEntityList;
            Debug.Assert( data != null );
        }


        private void RobotHandler_VelodynePoints( object sender, Message message )
        {
            var data = message as PointCloud2;
            Debug.Assert( data != null );
            Debug.Log( string.Format( "{0}: {1} bytes", "VelodynePoints", data.MessageSize ) );
        }

    }
}
