using AuthenticTeleoperation.UserInterface.Converter;
using System;
using System.Reflection;
using UnityEngine;
using UnityEngine.UI;

namespace AuthenticTeleoperation.UserInterface.Binding
{
    internal sealed class TextUpdater : MonoBehaviour
    {
        private bool _isInitialized = false;
        private object _targetObject = null;
        private PropertyInfo _targetProperty;
        private object _propertyValue = null;
        private Text _uiText = null;
        private ValueConverter _valueConverter = null;


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
            gameObject.TryGetComponent( out _uiText );
        }


        private void OnDisable( )
        {
            _uiText = null;
            _isInitialized = false;
        }


        private void Update( )
        {
            if( _isInitialized && _uiText != null )
            {
                object value = _targetProperty.GetValue( _targetObject );
                if( !IsEqual( _propertyValue, value ) )
                {
                    _propertyValue = value;
                    _uiText.text = _valueConverter.Convert( _propertyValue ).ToString( );
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
