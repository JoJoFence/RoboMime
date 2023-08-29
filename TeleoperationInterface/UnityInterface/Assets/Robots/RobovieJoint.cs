using MotionEditor.Robots;
using System;
using UnityEngine;
using UnityEngine.UI;

namespace AuthenticTeleoperation.Robots
{
    internal sealed class RobovieJoint : MonoBehaviour, IRobovieJoint
    {
        public enum JOINT_AXIS
        {
            X_AXIS,
            Y_AXIS,
            Z_AXIS,
            X_REVERSE,
            Y_REVERSE,
            Z_REVERSE,
        }
        private double _angle = double.NaN; // current angle
        private double _angleMax = double.NaN;
        private double _angleMin = double.NaN;
        private double _angleOffset = 0.0;
        private double _angleOffsetModel = 0.0; // 3D model's angle offset
        private double _cvt_AxisToAngle_Gain = 1.0;
        private double _cvt_AxisToAngle_Offset = 0.0;
        private InputField _uiInputField = null;
        private Slider _uiSlider = null;
        public JOINT_AXIS axis = JOINT_AXIS.Z_AXIS;
        public GameObject model = null;


        public event JointValueChangedEventHandler ValueChanged;


        public double Angle
        {   // current angle
            get { return _angle; }
            set
            {
                value = Math.Max( _angleMin, Math.Min( _angleMax, ClampDegrees( value ) ) );
                if( _angle != value )
                {
                    _angle = value;
                    if( _uiInputField != null )
                    {
                        _uiInputField.text = string.Format( "{0:0.0}", _angle );
                    }
                    if( _uiSlider != null )
                    {
                        _uiSlider.value = (float)_angle;
                    }
                    if( model != null )
                    {
                        Vector3 modelAngle = model.transform.localRotation.eulerAngles;
                        switch( axis )
                        {
                        case JOINT_AXIS.X_AXIS:
                            modelAngle.x = (float)(_angleOffset + _angleOffsetModel + Angle);
                            break;
                        case JOINT_AXIS.Y_AXIS:
                            modelAngle.y = (float)(_angleOffset + _angleOffsetModel + Angle);
                            break;
                        case JOINT_AXIS.Z_AXIS:
                            modelAngle.z = (float)(_angleOffset + _angleOffsetModel + Angle);
                            break;
                        case JOINT_AXIS.X_REVERSE:
                            modelAngle.x = (float)(_angleOffset + _angleOffsetModel - Angle);
                            break;
                        case JOINT_AXIS.Y_REVERSE:
                            modelAngle.y = (float)(_angleOffset + _angleOffsetModel - Angle);
                            break;
                        case JOINT_AXIS.Z_REVERSE:
                            modelAngle.z = (float)(_angleOffset + _angleOffsetModel - Angle);
                            break;
                        }
                        model.transform.localRotation = Quaternion.Euler( modelAngle );
                    }
                    ValueChanged?.Invoke( this, new JointValueChangedEventArgs( ID, value, PoseValue ) );
                }
            }
        }


        public double AngleOffset
        {
            get { return _angleOffset; }
            set { _angleOffset = double.IsInfinity( value ) || double.IsNaN( value ) ? 0.0 : value; }
        }


        public double ConversionGain
        {
            get { return _cvt_AxisToAngle_Gain; }
            set { _cvt_AxisToAngle_Gain = double.IsInfinity( value ) || double.IsNaN( value ) || value == 0.0 ? 1.0 : value; }
        }


        public double ConversionOffset
        {
            get { return _cvt_AxisToAngle_Offset; }
            set { _cvt_AxisToAngle_Offset = double.IsInfinity( value ) || double.IsNaN( value ) ? 0.0 : value; }
        }


        public int ID
        {
            get;
            set;
        }


        public double MaxAngle
        {
            get { return _angleMax; }
            set
            {
                value = ClampDegrees( value );
                if( _angleMax != value )
                {
                    _angleMax = value;
                    if( _uiSlider != null )
                    {
                        _uiSlider.maxValue = (float)_angleMax;
                    }
                }
            }
        }


        public double MinAngle
        {
            get { return _angleMin; }
            set
            {
                value = ClampDegrees( value );
                if( _angleMin != value )
                {
                    _angleMin = value;
                    if( _uiSlider != null )
                    {
                        _uiSlider.minValue = (float)_angleMin;
                    }
                }
            }
        }


        public double PoseValue
        {
            get { return (_angle - _cvt_AxisToAngle_Offset) / _cvt_AxisToAngle_Gain; }
            set { Angle = value * _cvt_AxisToAngle_Gain + _cvt_AxisToAngle_Offset; }
        }


        private void Awake( )
        {
            Transform child = Utilities.GetDescendantByName( transform, "InputField" );
            if( child != null )
            {
                _uiInputField = child.gameObject.GetComponent<InputField>( );
                if( _uiInputField != null )
                {
                    _uiInputField.onValueChanged.AddListener( OnAngleTextChanged );
                }
            }
            child = Utilities.GetDescendantByName( transform, "Slider" );
            if( child != null )
            {
                _uiSlider = child.gameObject.GetComponent<Slider>( );
                if( _uiSlider != null )
                {
                    _uiSlider.onValueChanged.AddListener( OnAngleValueChanged );
                }
            }
            if( model != null )
            {
                Vector3 angle = model.transform.localRotation.eulerAngles;
                switch( axis )
                {
                case JOINT_AXIS.X_AXIS:
                case JOINT_AXIS.X_REVERSE:
                    _angleOffsetModel += ClampDegrees( angle.x );
                    break;
                case JOINT_AXIS.Y_AXIS:
                case JOINT_AXIS.Y_REVERSE:
                    _angleOffsetModel += ClampDegrees( angle.y );
                    break;
                case JOINT_AXIS.Z_AXIS:
                case JOINT_AXIS.Z_REVERSE:
                    _angleOffsetModel += ClampDegrees( angle.z );
                    break;
                }
            }
        }


        private void OnApplicationQuit( )
        {
            if( _uiInputField != null )
            {
                _uiInputField.onEndEdit.RemoveListener( OnAngleTextChanged );
                _uiInputField = null;
            }
            if( _uiSlider != null )
            {
                _uiSlider.onValueChanged.RemoveListener( OnAngleValueChanged );
                _uiSlider = null;
            }
        }


        private void OnAngleTextChanged( string value )
        {
            if( float.TryParse( value, out float v ) )
            {
                Angle = v;
            }
        }


        private void OnAngleValueChanged( float value )
        {
            Angle = value;
        }


        private static double ClampDegrees( double value )
        {
            while( value < -180.0 )
            {
                value += 360.0;
            }
            while( value > 180.0 )
            {
                value -= 360.0;
            }
            return value;
        }

    }
}
