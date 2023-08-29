using System;

namespace AuthenticTeleoperation.UserInterface.Converter
{
    internal sealed class PrimitiveToFloatConverter : ValueConverter
    {
        private PrimitiveType _type = PrimitiveType.Object;


        public PrimitiveToFloatConverter( PrimitiveType type )
        {
            _type = type;
        }


        public override object Convert( object value )
        {
            switch( _type )
            {
            case PrimitiveType.Boolean:
                return (bool)value ? 1.0f : 0.0f;
            case PrimitiveType.Int8:
                return (float)(sbyte)value;
            case PrimitiveType.Int16:
                return (float)(short)value;
            case PrimitiveType.Int32:
                return (float)(int)value;
            case PrimitiveType.Int64:
                return (float)(long)value;
            case PrimitiveType.UInt8:
                return (float)(byte)value;
            case PrimitiveType.UInt16:
                return (float)(ushort)value;
            case PrimitiveType.UInt32:
                return (float)(uint)value;
            case PrimitiveType.UInt64:
                return (float)(ulong)value;
            case PrimitiveType.Single:
                return value;
            case PrimitiveType.Double:
                return (float)(double)value;
            case PrimitiveType.String:
                return float.TryParse( value.ToString( ), out float cvt ) ? cvt : 0.0f;
            case PrimitiveType.Object:
            default:
                return 0.0f;
            }
        }


        public override object ConvertBack( object value )
        {
            switch( _type )
            {
            case PrimitiveType.Boolean:
                return (float)value != 0.0f;
            case PrimitiveType.Int8:
                return (sbyte)(float)value;
            case PrimitiveType.Int16:
                return (short)(float)value;
            case PrimitiveType.Int32:
                return (int)(float)value;
            case PrimitiveType.Int64:
                return (long)(float)value;
            case PrimitiveType.UInt8:
                return (byte)(float)value;
            case PrimitiveType.UInt16:
                return (ushort)(float)value;
            case PrimitiveType.UInt32:
                return (uint)(float)value;
            case PrimitiveType.UInt64:
                return (ulong)(float)value;
            case PrimitiveType.Single:
                return value;
            case PrimitiveType.Double:
                return (double)(float)value;
            case PrimitiveType.String:
                return ((float)value).ToString( "0.00" );
            case PrimitiveType.Object:
            default:
                return value;
            }
        }


        public static PrimitiveToFloatConverter Create( Type type )
        {
            if( type == typeof( bool ) )
                return new PrimitiveToFloatConverter( PrimitiveType.Boolean );
            else if( type == typeof( sbyte ) )
                return new PrimitiveToFloatConverter( PrimitiveType.Int8 );
            else if( type == typeof( short ) )
                return new PrimitiveToFloatConverter( PrimitiveType.Int16 );
            else if( type == typeof( int ) )
                return new PrimitiveToFloatConverter( PrimitiveType.Int32 );
            else if( type == typeof( long ) )
                return new PrimitiveToFloatConverter( PrimitiveType.Int64 );
            else if( type == typeof( byte ) )
                return new PrimitiveToFloatConverter( PrimitiveType.UInt8 );
            else if( type == typeof( ushort ) )
                return new PrimitiveToFloatConverter( PrimitiveType.UInt16 );
            else if( type == typeof( uint ) )
                return new PrimitiveToFloatConverter( PrimitiveType.UInt32 );
            else if( type == typeof( ulong ) )
                return new PrimitiveToFloatConverter( PrimitiveType.UInt64 );
            else if( type == typeof( float ) )
                return new PrimitiveToFloatConverter( PrimitiveType.Single );
            else if( type == typeof( double ) )
                return new PrimitiveToFloatConverter( PrimitiveType.Double );
            else if( type == typeof( string ) )
                return new PrimitiveToFloatConverter( PrimitiveType.String );
            else
                return new PrimitiveToFloatConverter( PrimitiveType.Object );
        }

    }
}
