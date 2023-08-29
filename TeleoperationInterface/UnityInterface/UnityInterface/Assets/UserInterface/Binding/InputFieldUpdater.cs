using AuthenticTeleoperation.UserInterface.Converter;
using System;
using System.Reflection;
using UnityEngine;
using UnityEngine.UI;

namespace AuthenticTeleoperation.UserInterface.Binding
{
    internal sealed class InputFieldUpdater : MonoBehaviour
    {
        private bool _isInitialized = false;
        private bool _isSettingValue = false;
        private object _targetObject = null;
        private PropertyInfo _targetProperty;
        private object _propertyValue = null;
        private InputField _uiInputField = null;
        private ValueConverter _valueConverter = null;


        public event EventHandler<string> TextChanged;


        public void Initialize( object targetObject, string targetPropertyName, ValueConverter converter = null )
        {
            System.Diagnostics.Debug.Assert( targetObject != null && !string.IsNullOrWhiteSpace( targetPropertyName ) );
            PropertyInfo targetProperty = targetObject.GetType( ).GetProperty( targetPropertyName );
            System.Diagnostics.Debug.Assert( targetProperty != null );
            _targetObject = targetObject ?? throw new ArgumentNullException( );
            _targetProperty = targetProperty ?? throw new ArgumentNullException( );
            _valueConverter = converter ?? PrimitiveToStringConverter.Create( _targetProperty.PropertyType );
            _isInitialized = true;
        }


        public void Initialize( object targetObject, PropertyInfo targetProperty, ValueConverter converter = null )
        {
            System.Diagnostics.Debug.Assert( targetObject != null && targetProperty != null );
            _targetObject = targetObject ?? throw new ArgumentNullException( );
            _targetProperty = targetProperty ?? throw new ArgumentNullException( );
            _valueConverter = converter ?? PrimitiveToStringConverter.Create( _targetProperty.PropertyType );
            _isInitialized = true;
        }


        private void OnEnable( )
        {
            if( gameObject.TryGetComponent( out _uiInputField ) )
            {
                _uiInputField.onValueChanged.AddListener( OnTextChanged );
            }
        }


        private void OnDisable( )
        {
            if( _uiInputField != null )
            {
                _uiInputField.onValueChanged.RemoveListener( OnTextChanged );
                _uiInputField = null;
            }
            _isInitialized = false;
        }


        private void OnTextChanged( string value )
        {
            if( !_isSettingValue )
            {
                object obj = _valueConverter.ConvertBack( value );
                if( !IsEqual( _propertyValue, obj ) )
                {
                    _propertyValue = obj;
                    _targetProperty.SetValue( _targetObject, _propertyValue );
                    TextChanged?.Invoke( this, value );
                }
            }
        }


        private void Update( )
        {
            if( _isInitialized && _uiInputField != null )
            {
                object obj = _targetProperty.GetValue( _targetObject );
                if( !IsEqual( _propertyValue, obj ) )
                {
                    _propertyValue = obj;
                    _isSettingValue = true;
                    _uiInputField.text = _valueConverter.Convert( _propertyValue ).ToString( );
                    _isSettingValue = false;
                }
            }
        }


        private static bool IsEqual( object a, object b )
        {
            if( a == null && b == null )
            {
                return true;
            }
            else if( a != null )
            {
                return a.Equals( b );
            }
            else
            {
                return b.Equals( a );
            }
        }

    }
}
