using System;

namespace AuthenticTeleoperation.Robots
{
    internal delegate void JointValueChangedEventHandler( object sender, JointValueChangedEventArgs e );


    internal sealed class JointValueChangedEventArgs : EventArgs
    {
        public JointValueChangedEventArgs( int identifier, double angle, double value )
        {
            ID = identifier;
            Angle = angle;
            Value = value;
        }


        public int ID
        {
            get;
            private set;
        }


        public double Angle
        {
            get;
            private set;
        }


        public double Value
        {
            get;
            private set;
        }

    }
}
