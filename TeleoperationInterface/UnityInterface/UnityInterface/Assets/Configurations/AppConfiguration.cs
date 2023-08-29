using SSSoftworks.JSON;
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace AuthenticTeleoperation.Configurations
{
    internal sealed class AppConfiguration
    {
        public const int NUM_AXES = 11;
        private const string SETTING_FILE = "configuration.json";

        private static AppConfiguration __instance = null;
        private static object __mutex = new object( );

        public static AppConfiguration Instance
        {
            get
            {
                if( __instance == null )
                {
                    lock( __mutex )
                    {
                        if( __instance == null )
                        {
                            __instance = new AppConfiguration( );
                        }
                    }
                }
                return __instance;
            }
        }


        private bool _isConfigurationUpdated = false;
        private string _hostAddress = "127.0.0.1";
        private int _hostPort = 9560;
        private float _maxForeHindSpeed = 0.7f;
        private float _maxFlankSpeed = 0.35f;
        private float _maxTurnSpeed = 1.0f;
        private int _audioSampleRate = 16000;
        private int _audioBitsPerSample = 16;
        private int _audioChannels = 1;
        private string _audioMicrophone = "";
        private string _audioSpeaker = "";


        private AppConfiguration( )
        {
            Joints = new List<JointConfiguration>( );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 150.0, MinAngle = -150.0, AngleOffset = 0.0, ConversionGainNumerator = 180.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = 0.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 10.0, MinAngle = -75.0, AngleOffset = 75.0, ConversionGainNumerator = 75.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = -5.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 45.0, MinAngle = -40.0, AngleOffset = 35.0, ConversionGainNumerator = 90.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = 0.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 145.0, MinAngle = 15.0, AngleOffset = 0.0, ConversionGainNumerator = 77.0, ConversionGainDenominator = 5.0, ConversionOffset = 90.0, DefaultPoseValue = -5.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 150.0, MinAngle = -150.0, AngleOffset = 0.0, ConversionGainNumerator = 180.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = 0.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 10.0, MinAngle = -75.0, AngleOffset = -75.0, ConversionGainNumerator = 75.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = -5.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 45.0, MinAngle = -40.0, AngleOffset = -35.0, ConversionGainNumerator = 90.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = 0.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 145.0, MinAngle = 15.0, AngleOffset = 0.0, ConversionGainNumerator = 77.0, ConversionGainDenominator = 5.0, ConversionOffset = 90.0, DefaultPoseValue = -5.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 90.0, MinAngle = -90.0, AngleOffset = 0.0, ConversionGainNumerator = 90.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = 0.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 40.0, MinAngle = -50.0, AngleOffset = 0.0, ConversionGainNumerator = 40.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = 0.0 } );
            Joints.Add( new JointConfiguration( ) { MaxAngle = 40.0, MinAngle = -40.0, AngleOffset = 0.0, ConversionGainNumerator = 80.0, ConversionGainDenominator = 5.0, ConversionOffset = 0.0, DefaultPoseValue = 0.0 } );
        }


        public double[] DefaultJointValues
        {
            get
            {
                Debug.Assert( Joints.Count == NUM_AXES );
                double[] value = new double[NUM_AXES];
                for( int i = 0; i < Joints.Count; ++i )
                {
                    value[i] = Joints[i].DefaultPoseValue;
                }
                return value;
            }
        }


        public List<JointConfiguration> Joints
        {
            get;
            private set;
        }


        public bool IsConfigurationUpdated
        {
            get
            {
                if( _isConfigurationUpdated )
                {
                    return true;
                }
                else
                {
                    foreach( JointConfiguration joint in Joints )
                    {
                        if( joint.IsConfigurationUpdated )
                        {
                            _isConfigurationUpdated = true;
                            return true;
                        }
                    }
                    return false;
                }
            }
            set
            {
                _isConfigurationUpdated = value;
                if( !_isConfigurationUpdated )
                {
                    foreach( JointConfiguration joint in Joints )
                    {
                        joint.IsConfigurationUpdated = value;
                    }
                }
            }
        }


        public string HostAddress
        {
            get { return _hostAddress; }
            set
            {
                _hostAddress = value ?? "127.0.0.1";
                IsConfigurationUpdated = true;
            }
        }


        public int HostPort
        {
            get { return _hostPort; }
            set
            {
                _hostPort = value;
                IsConfigurationUpdated = true;
            }
        }


        public float MaxForeHindSpeed
        {
            get { return _maxForeHindSpeed; }
            set
            {
                _maxForeHindSpeed = Math.Max( 0.0f, Math.Min( 10.0f, value ) );
                IsConfigurationUpdated = true;
            }
        }


        public float MaxFlankSpeed
        {
            get { return _maxFlankSpeed; }
            set
            {
                _maxFlankSpeed = Math.Max( 0.0f, Math.Min( 10.0f, value ) );
                IsConfigurationUpdated = true;
            }
        }


        public float MaxTurnSpeed
        {
            get { return _maxTurnSpeed; }
            set
            {
                _maxTurnSpeed = Math.Max( 0.0f, Math.Min( 10.0f, value ) );
                IsConfigurationUpdated = true;
            }
        }


        public int AudioSampleRate
        {
            get { return _audioSampleRate; }
            set
            {
                if( _audioSampleRate != value )
                {
                    _audioSampleRate = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public int AudioBitsPerSample
        {
            get { return _audioBitsPerSample; }
            set
            {
                if( _audioBitsPerSample != value )
                {
                    _audioBitsPerSample = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public int AudioChannels
        {
            get { return _audioChannels; }
            set
            {
                if( _audioChannels != value )
                {
                    _audioChannels = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public string MicrophoneDeviceName
        {
            get { return _audioMicrophone; }
            set
            {
                value = value ?? string.Empty;
                if( _audioMicrophone != value )
                {
                    _audioMicrophone = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public string SpeakerDeviceName
        {
            get { return _audioSpeaker; }
            set
            {
                value = value ?? string.Empty;
                if( _audioSpeaker != value )
                {
                    _audioSpeaker = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public void Read( )
        {
            if( JSONParser.FileToJSON( SETTING_FILE ) is JSONStruct json )
            {
                HostAddress = json.GetString( "HostAddress", HostAddress );
                HostPort = json.GetInt32( "HostPort", HostPort );
                MaxForeHindSpeed = json.GetSingle( "MaxForeHindSpeed", MaxForeHindSpeed );
                MaxFlankSpeed = json.GetSingle( "MaxFlankSpeed", MaxFlankSpeed );
                MaxTurnSpeed = json.GetSingle( "MaxTurnSpeed", MaxTurnSpeed );
                AudioSampleRate = json.GetInt32( "AudioSampleRate", AudioSampleRate );
                AudioBitsPerSample = json.GetInt32( "AudioBitsPerSample", AudioBitsPerSample );
                AudioChannels = json.GetInt32( "AudioChannels", AudioChannels );
                MicrophoneDeviceName = json.GetString( "MicrophoneDeviceName", MicrophoneDeviceName );
                SpeakerDeviceName = json.GetString( "SpeakerDeviceName", SpeakerDeviceName );
                JSONArray jsJoints = json.GetArray( "RobovieJoints" );
                for( int i = 0; i < Joints.Count && i < jsJoints.Count; ++i )
                {
                    JSONStruct jsJoint = jsJoints[i] as JSONStruct;
                    Joints[i].MaxAngle = jsJoint.GetDouble( "MaxAngle", Joints[i].MaxAngle );
                    Joints[i].MinAngle = jsJoint.GetDouble( "MinAngle", Joints[i].MinAngle );
                    Joints[i].AngleOffset = jsJoint.GetDouble( "AngleOffset", Joints[i].AngleOffset );
                    Joints[i].ConversionGainNumerator = jsJoint.GetDouble( "ConversionGainNumerator", Joints[i].ConversionGainNumerator );
                    Joints[i].ConversionGainDenominator = jsJoint.GetDouble( "ConversionGainDenominator", Joints[i].ConversionGainDenominator );
                    Joints[i].ConversionOffset = jsJoint.GetDouble( "ConversionOffset", Joints[i].ConversionOffset );
                    Joints[i].DefaultPoseValue = jsJoint.GetDouble( "DefaultPoseValue", Joints[i].DefaultPoseValue );
                }
                IsConfigurationUpdated = false;
            }
            else
            {
                Write( true );
            }
        }


        public void Write( bool force = false )
        {
            if( force || IsConfigurationUpdated )
            {
                JSONStruct json = new JSONStruct( );
                json.Add( "HostAddress", new JSONString( HostAddress ) );
                json.Add( "HostPort", new JSONNumber( HostPort ) );
                json.Add( "MaxForeHindSpeed", new JSONNumber( MaxForeHindSpeed ) );
                json.Add( "MaxFlankSpeed", new JSONNumber( MaxFlankSpeed ) );
                json.Add( "MaxTurnSpeed", new JSONNumber( MaxTurnSpeed ) );
                json.Add( "AudioSampleRate", new JSONNumber( AudioSampleRate ) );
                json.Add( "AudioBitsPerSample", new JSONNumber( AudioBitsPerSample ) );
                json.Add( "AudioChannels", new JSONNumber( AudioChannels ) );
                json.Add( "MicrophoneDeviceName", new JSONString( MicrophoneDeviceName ) );
                json.Add( "SpeakerDeviceName", new JSONString( SpeakerDeviceName ) );
                JSONArray jsJoints = new JSONArray( );
                foreach( JointConfiguration joint in Joints )
                {
                    JSONStruct jsJoint = new JSONStruct( );
                    jsJoint.Add( "MaxAngle", new JSONNumber( joint.MaxAngle ) );
                    jsJoint.Add( "MinAngle", new JSONNumber( joint.MinAngle ) );
                    jsJoint.Add( "AngleOffset", new JSONNumber( joint.AngleOffset ) );
                    jsJoint.Add( "ConversionGainNumerator", new JSONNumber( joint.ConversionGainNumerator ) );
                    jsJoint.Add( "ConversionGainDenominator", new JSONNumber( joint.ConversionGainDenominator ) );
                    jsJoint.Add( "ConversionOffset", new JSONNumber( joint.ConversionOffset ) );
                    jsJoint.Add( "DefaultPoseValue", new JSONNumber( joint.DefaultPoseValue ) );
                    jsJoints.Add( jsJoint );
                }
                try
                {
                    File.WriteAllText( SETTING_FILE, json.ToIndentedString( ) );
                    IsConfigurationUpdated = false;
                }
                catch
                {
                }
            }
        }

    }
}
