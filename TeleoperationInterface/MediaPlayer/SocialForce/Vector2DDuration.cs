using System.Windows;

namespace MediaPlayer.SocialForceModule
{
    public sealed class Vector2DDuration : Vector2D
    {

        public Vector2DDuration( double x, double y, double beginTimestamp = 0.0, double endTimestamp = 0.0 )
            : base( x, y )
        {
            BeginTimestamp = beginTimestamp;
            EndTimestamp = endTimestamp;
        }


        public Vector2DDuration( Point point, double beginTimestamp = 0.0, double endTimestamp = 0.0 )
            : base( point.X, point.Y )
        {
            BeginTimestamp = beginTimestamp;
            EndTimestamp = endTimestamp;
        }


        public Vector2DDuration( Vector vector, double beginTimestamp = 0.0, double endTimestamp = 0.0 )
            : base( vector.X, vector.Y )
        {
            BeginTimestamp = beginTimestamp;
            EndTimestamp = endTimestamp;
        }


        public Vector2DDuration( Vector2D vector, double beginTimestamp = 0.0, double endTimestamp = 0.0 )
            : base( vector.X, vector.Y )
        {
            BeginTimestamp = beginTimestamp;
            EndTimestamp = endTimestamp;
        }


        public Vector2DDuration( Vector2DDuration other )
            : base( other.X, other.Y )
        {
            BeginTimestamp = other.BeginTimestamp;
            EndTimestamp = other.EndTimestamp;
        }


        public double BeginTimestamp
        {
            get;
            set;
        }


        public double EndTimestamp
        {
            get;
            set;
        }

    }
}
