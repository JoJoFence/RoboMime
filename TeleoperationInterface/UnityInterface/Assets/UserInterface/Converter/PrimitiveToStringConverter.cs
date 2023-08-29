using System;

namespace AuthenticTeleoperation.UserInterface.Converter
{
    internal sealed class PrimitiveToStringConverter : ValueConverter
    {
        private PrimitiveType _type = PrimitiveType.Object;


        public PrimitiveToStringConverter( PrimitiveType type )
        {
            _type = type;
        }


        public override object Convert( object value )
        {
            switch( _type )
            {
            case PrimitiveType.Single:
            case PrimitiveType.Double:
                return string.Format( "{0:0.00}", value );
            case PrimitiveType.Object:
            case PrimitiveType.Boolean:
            case PrimitiveType.Int8:
            case PrimitiveType.Int16:
            case PrimitiveType.Int32:
            case PrimitiveType.Int64:
            case PrimitiveType.UInt8:
            case PrimitiveType.UInt16:
            case PrimitiveType.UInt32:
            case PrimitiveType.UInt64:
            case PrimitiveType.String:
            default:
                return value.ToString( );
            }
        }


        public override object ConvertBack( object value )
        {
            switch( _type )
            {
            case PrimitiveType.Boolean:
                return bool.TryParse( value.ToString( ), out bool vBoolean ) ? vBoolean : false;
            case PrimitiveType.Int8:
                return sbyte.TryParse( value.ToString( ), out sbyte vInt8 ) ? vInt8 : (sbyte)0;
            case PrimitiveType.Int16:
                return short.TryParse( value.ToString( ), out short vInt16 ) ? vInt16 : (short)0;
            case PrimitiveType.Int32:
                return int.TryParse( value.ToString( ), out int vInt32 ) ? vInt32 : 0;
            case PrimitiveType.Int64:
                return long.TryParse( value.ToString( ), out long vInt64 ) ? vInt64 : 0L;
            case PrimitiveType.UInt8:
                return byte.TryParse( value.ToString( ), out byte vUInt8 ) ? vUInt8 : (byte)0;
            case PrimitiveType.UInt16:
                return ushort.TryParse( value.ToString( ), out ushort vUInt16 ) ? vUInt16 : (ushort)0;
            case PrimitiveType.UInt32:
                return uint.TryParse( value.ToString( ), out uint vUInt32 ) ? vUInt32 : 0U;
            case PrimitiveType.UInt64:
                return ulong.TryParse( value.ToString( ), out ulong vUInt64 ) ? vUInt64 : 0UL;
            case PrimitiveType.Single:
                return float.TryParse( value.ToString( ), out float vSingle ) ? vSingle : 0F;
            case PrimitiveType.Double:
                return double.TryParse( value.ToString( ), out double vDouble ) ? vDouble : 0D;
            case PrimitiveType.Object:
            case PrimitiveType.String:
            default:
                return value;
            }
        }


        public static PrimitiveToStringConverter Create( Type type )
        {
            if( type == typeof( bool ) )
                return new PrimitiveToStringConverter( PrimitiveType.Boolean );
            else if( type == typeof( sbyte ) )
                return new PrimitiveToStringConverter( PrimitiveType.Int8 );
            else if( type == typeof( short ) )
                return new PrimitiveToStringConverter( PrimitiveType.Int16 );
            else if( type == typeof( int ) )
                return new PrimitiveToStringConverter( PrimitiveType.Int32 );
            else if( type == typeof( long ) )
                return new PrimitiveToStringConverter( PrimitiveType.Int64 );
            else if( type == typeof( byte ) )
                return new PrimitiveToStringConverter( PrimitiveType.UInt8 );
            else if( type == typeof( ushort ) )
                return new PrimitiveToStringConverter( PrimitiveType.UInt16 );
            else if( type == typeof( uint ) )
                return new PrimitiveToStringConverter( PrimitiveType.UInt32 );
            else if( type == typeof( ulong ) )
                return new PrimitiveToStringConverter( PrimitiveType.UInt64 );
            else if( type == typeof( float ) )
                return new PrimitiveToStringConverter( PrimitiveType.Single );
            else if( type == typeof( double ) )
                return new PrimitiveToStringConverter( PrimitiveType.Double );
            else if( type == typeof( string ) )
                return new PrimitiveToStringConverter( PrimitiveType.String );
            else
                return new PrimitiveToStringConverter( PrimitiveType.Object );
        }

    }
}
