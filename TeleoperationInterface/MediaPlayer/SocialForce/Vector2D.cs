using System;
using System.Windows;

namespace MediaPlayer.SocialForceModule
{
    public class Vector2D
    {
        public static implicit operator Vector( Vector2D v ) => v != null ? new Vector( v.X, v.Y ) : new Vector( );
        public static explicit operator Vector2D( Vector v ) => new Vector2D( v.X, v.Y );


        public Vector2D( double x = 0.0, double y = 0.0 )
        {
            X = x;
            Y = y;
        }


        public Vector2D( Point point )
        {
            X = point.X;
            Y = point.Y;
        }


        public Vector2D( Vector vector )
        {
            X = vector.X;
            Y = vector.Y;
        }


        public Vector2D( Vector2D other )
        {
            X = other.X;
            Y = other.Y;
        }


        public double X
        {
            get;
            set;
        }


        public double Y
        {
            get;
            set;
        }


        public Vector2D Add( double value )
        {
            X += value;
            Y += value;
            return this;
        }


        public Vector2D Add( double x, double y )
        {
            X += x;
            Y += y;
            return this;
        }


        public Vector2D Add( Vector2D other )
        {
            X += other.X;
            Y += other.Y;
            return this;
        }


        public double Angle( double x, double y )
        {
            return Angle( X, Y, x, y );
        }


        public double Angle( Vector2D other )
        {
            return Angle( X, Y, other.X, other.Y );
        }


        public Vector2D Divide( double value )
        {
            X /= value;
            Y /= value;
            return this;
        }


        public Vector2D Divide( double x, double y )
        {
            X /= x;
            Y /= y;
            return this;
        }


        public Vector2D Divide( Vector2D other )
        {
            X /= other.X;
            Y /= other.Y;
            return this;
        }


        public double Dot( Vector2D other )
        {
            return Dot( X, Y, other.X, other.Y );
        }


        public double Magnitude( )
        {
            return Magnitude( X, Y );
        }


        public Vector2D Multiply( double value )
        {
            X *= value;
            Y *= value;
            return this;
        }


        public Vector2D Multiply( double x, double y )
        {
            X *= x;
            Y *= y;
            return this;
        }


        public Vector2D Multiply( Vector2D other )
        {
            X *= other.X;
            Y *= other.Y;
            return this;
        }


        public Vector2D Normalize( )
        {
            double mag = Magnitude( X, Y );
            if( mag != 0.0 )
            {
                X /= mag;
                Y /= mag;
            }
            else
            {   // in the math world, this should be (nan, nan) but for our convenience...
                X = 0.0;
                Y = 0.0;
            }
            return this;
        }


        public Vector2D Subtract( double value )
        {
            X -= value;
            Y -= value;
            return this;
        }


        public Vector2D Subtract( double x, double y )
        {
            X -= x;
            Y -= y;
            return this;
        }


        public Vector2D Subtract( Vector2D other )
        {
            X -= other.X;
            Y -= other.Y;
            return this;
        }


        public override string ToString( )
        {
            return string.Format( "({0:0.0000}, {1:0.0000})", X, Y );
        }


        public static double Angle( double x1, double y1, double x2, double y2 )
        {   // calculate angle between two 2D vectors
            double dot_prod = Dot( x1, y1, x2, y2 );
            double mag_multi = Magnitude( x1, y1 ) * Magnitude( x2, y2 );
            if( mag_multi != 0.0 )
            {
                return Math.Acos( dot_prod / mag_multi );
            }
            else
            {
                return double.PositiveInfinity;
            }
        }


        public static double Dot( double x1, double y1, double x2, double y2 )
        {   // calculate dot product of two 2D vectors
            return x1 * x2 + y1 * y2;
        }


        public static double Magnitude( double x, double y )
        {   // calculate magnitude of a 2D vector
            return Math.Sqrt( x * x + y * y );
        }


        public static double Magnitude( Vector v )
        {   // calculate magnitude of a 2D vector
            return Math.Sqrt( v.X * v.X + v.Y * v.Y );
        }


        public static Vector2D Normalize( double x, double y )
        {   // calculate magnitude of a 2D vector
            double mag = Magnitude( x, y );
            if( mag != 0.0 )
            {
                return new Vector2D( x / mag, y / mag );
            }
            else
            {
                return new Vector2D( double.NaN, double.NaN );
            }
        }

    }
}
