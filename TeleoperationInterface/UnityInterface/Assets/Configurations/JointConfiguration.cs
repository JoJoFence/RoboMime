namespace AuthenticTeleoperation.Configurations
{
    internal sealed class JointConfiguration
    {
        private double _defaultPoseValue = 0.0;
        private double _maxAngle = 180.0;
        private double _minAngle = 180.0;
        private double _angleOffset = 0.0;
        private double _conversionGainNumerator = 0.0;
        private double _conversionGainDenominator = 0.0;
        private double _conversionOffset = 0.0;


        public JointConfiguration( )
        {
            IsConfigurationUpdated = true;
        }


        public bool IsConfigurationUpdated
        {
            get;
            set;
        }


        public double DefaultPoseValue
        {
            get { return _defaultPoseValue; }
            set
            {
                if( _defaultPoseValue != value )
                {
                    _defaultPoseValue = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public double MaxAngle
        {
            get { return _maxAngle; }
            set
            {
                if( _maxAngle != value )
                {
                    _maxAngle = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public double MinAngle
        {
            get { return _minAngle; }
            set
            {
                if( _minAngle != value )
                {
                    _minAngle = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public double AngleOffset
        {
            get { return _angleOffset; }
            set
            {
                if( _angleOffset != value )
                {
                    _angleOffset = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public double ConversionGain
        {
            get { return _conversionGainNumerator / _conversionGainDenominator; }
        }


        public double ConversionGainNumerator
        {
            get { return _conversionGainNumerator; }
            set
            {
                if( _conversionGainNumerator != value )
                {
                    _conversionGainNumerator = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public double ConversionGainDenominator
        {
            get { return _conversionGainDenominator; }
            set
            {
                if( _conversionGainDenominator != value )
                {
                    _conversionGainDenominator = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public double ConversionOffset
        {
            get { return _conversionOffset; }
            set
            {
                if( _conversionOffset != value )
                {
                    _conversionOffset = value;
                    IsConfigurationUpdated = true;
                }
            }
        }


        public void CopyFrom( JointConfiguration other )
        {
            _defaultPoseValue = other._defaultPoseValue;
            _maxAngle = other._maxAngle;
            _minAngle = other._minAngle;
            _angleOffset = other._angleOffset;
            _conversionGainNumerator = other._conversionGainNumerator;
            _conversionGainDenominator = other._conversionGainDenominator;
            _conversionOffset = other.ConversionOffset;
            IsConfigurationUpdated = true;
        }

    }
}
