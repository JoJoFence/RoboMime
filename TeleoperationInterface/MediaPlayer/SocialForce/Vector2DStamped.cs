using System.Windows;

namespace MediaPlayer.SocialForceModule
{
    public class Vector2DStamped : Vector2D
    {

        public Vector2DStamped( double x = 0.0, double y = 0.0, double timestamp = 0.0 )
            : base( x, y )
        {
            Timestamp = timestamp;
        }


        public Vector2DStamped( Point point, double timestamp = 0.0 )
            : base( point.X, point.Y )
        {
            Timestamp = timestamp;
        }


        public Vector2DStamped( Vector vector, double timestamp = 0.0 )
            : base( vector.X, vector.Y )
        {
            Timestamp = timestamp;
        }


        public Vector2DStamped( Vector2D vector, double timestamp = 0.0 )
            : base( vector.X, vector.Y )
        {
            Timestamp = timestamp;
        }


        public Vector2DStamped( Vector2DStamped other )
            : base( other.X, other.Y )
        {
            Timestamp = other.Timestamp;
        }


        public double Timestamp
        {
            get;
            set;
        }

    }
}
