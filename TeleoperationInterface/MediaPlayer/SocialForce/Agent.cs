using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;

namespace MediaPlayer.SocialForceModule
{
    internal sealed class Agent : INotifyPropertyChanged
    {
        private double[] _cnvMatrix = new double[] { 1.0 }; // convolution matrix
        private List<Vector2DStamped> _conv = new List<Vector2DStamped>( );
        private List<Vector2DStamped> _raw = new List<Vector2DStamped>( );
        private FrameworkElement _uiElement = null;


        public Agent( string name )
        {
            Name = name ?? string.Empty;
        }


        public event PropertyChangedEventHandler PropertyChanged;


        public string Name
        {
            get;
            private set;
        }


        public FrameworkElement UIElement
        {
            get { return _uiElement; }
            set
            {
                _uiElement = value;
                NotifyPropertyChanged( );
            }
        }


        public void Add( double x, double y, double timestamp )
        {   // add a 2D position with timestamp
            int i = _raw.Count;
            while( i > 0 && _raw[i - 1].Timestamp > timestamp )
            {
                i = i - 1;
            }
            _raw.Insert( i, new Vector2DStamped( x, y, timestamp ) );
            if( i == _raw.Count - 1 )
            {
                _conv.Add( ConvolutePosition( timestamp, i ) );
            }
            else
            {
                ConvoluteAllPositions( );
            }
        }


        public Vector2D GetAcceleration( double beginTimestamp, double endTimestamp, double velocityDuration )
        {   // get acceleration
            if( _conv.Count <= 0 )
            {   // we don't have any data
                return new Vector2D( double.NaN, double.NaN );
            }
            Vector2DDuration beginVelocity = GetVelocity( beginTimestamp - velocityDuration, beginTimestamp );
            Vector2DDuration endVelocity = GetVelocity( endTimestamp - velocityDuration, endTimestamp );
            return endVelocity.Subtract( beginVelocity ).Divide( endVelocity.EndTimestamp - beginVelocity.EndTimestamp );
        }


        public Vector2DStamped GetConvolutedPosition( double timestamp, out int index )
        {   // get a convoluted position along with the nearest index at the timestamp
            int i = _conv.Count - 1;
            while( i >= 0 && _conv[i].Timestamp > timestamp )
            {
                i = i - 1;
                if( (0 <= i && i < _conv.Count - 1) || (i == _conv.Count - 1 && _conv[i].Timestamp == timestamp) )
                {
                    if( _conv[i].Timestamp == timestamp )
                    {   // known position
                        index = i;
                        return new Vector2DStamped( _conv[i] );
                    }
                    else
                    {   // in-between estimation
                        double timeDifference = _conv[i + 1].Timestamp - _conv[i].Timestamp;
                        if( timeDifference == 0.0 )
                        {   // error... (two positions' timestamps are the same)
                            index = i;
                            return new Vector2DStamped( _conv[i + 1] );
                        }
                        Vector2DStamped interpolatedPosition = new Vector2DStamped( _conv[i + 1].X - _conv[i].X, _conv[i + 1].Y - _conv[i].Y, timestamp );
                        interpolatedPosition.Multiply( (timestamp - _conv[i].Timestamp) / timeDifference );
                        interpolatedPosition.Add( _conv[i] );
                        index = i;
                        return interpolatedPosition;
                    }
                }
            }
            index = i;
            return 0 <= i && i < _conv.Count ? new Vector2DStamped( _conv[i] ) : null;
        }


        /// <summary>
        /// Get position of the given timestamp
        ///   If there is known position at the given timestamp, this function will returned the known position.
        ///   If there are known positions before and after the timestamp, this function will return linear estimation (i.e., in-between position)
        ///   If the given timestamp is out of the boundary (i.e., before the first timestamped position or after the last timestamped position),
        ///     this function will use velocityDuration to compute the velocity and estimate the position
        ///     Caller can set velocityDuration to be None if they do not want the estimation if the given timestamp is out of the begin and end timestamps
        /// </summary>
        public Vector2DStamped GetPosition( double timestamp, double velocityDuration = double.NaN )
        {
            if( _conv.Count <= 0 )
            {   // we don't have any data
                return new Vector2DStamped( double.NaN, double.NaN, timestamp );
            }
            Vector2DStamped position = GetConvolutedPosition( timestamp, out int i );
            if( position != null && !double.IsNaN( velocityDuration ) )
            {   // calculate estimated position using velocity
                if( i < 0 )
                {   // estimate a position before the first position
                    double diffTime = _conv[0].Timestamp - timestamp;
                    Vector2DDuration velocity = GetVelocity( _conv[0].Timestamp, _conv[0].Timestamp + velocityDuration );
                    return new Vector2DStamped( _conv[0].X - velocity.X * diffTime, _conv[0].Y - velocity.Y * diffTime, timestamp );
                }
                else
                {   // estimate a position after the last position
                    double diffTime = timestamp - _conv[_conv.Count - 1].Timestamp;
                    Vector2DDuration velocity = GetVelocity( _conv[_conv.Count - 1].Timestamp - velocityDuration, _conv[_conv.Count - 1].Timestamp );
                    return new Vector2DStamped( _conv[_conv.Count - 1].X + velocity.Y * diffTime, _conv[_conv.Count - 1].Y + velocity.Y * diffTime, timestamp );
                }
            }
            return position;
        }


        public Vector2DDuration GetVelocity( double beginTimestamp, double endTimestamp )
        {   // get velocity derived from positions between the begin timestamp and the end timestamp
            if( _conv.Count <= 0 )
            {   // we don't have any data
                return new Vector2DDuration( double.NaN, double.NaN, beginTimestamp, endTimestamp );
            }

            Vector2DStamped begin = GetConvolutedPosition( beginTimestamp, out int beginIndex );
            Vector2DStamped end = GetConvolutedPosition( endTimestamp, out int endIndex );
            if( begin == null )
            {
                if( beginIndex < 0 )
                {   // use the first
                    begin = _conv[0];
                }
                else
                {   // use the last
                    begin = _conv[_conv.Count - 1];
                }
                beginIndex = beginIndex + 1;
            }
            if( end == null )
            {
                if( endIndex < 0 )
                {   // use the first
                    end = _conv[0];
                }
                else
                {   // use the last
                    end = _conv[_conv.Count - 1];
                }
            }
            if( begin.Timestamp == end.Timestamp )
            {
                //return new Vector2DDuration( double.NaN, double.NaN, beginTimestamp, endTimestamp );
                return new Vector2DDuration( 0.0, 0.0, beginTimestamp, endTimestamp );
            }

            List<Vector2DStamped> positions = new List<Vector2DStamped>( );
            positions.Add( begin );
            for( int i = beginIndex + 1; i < endIndex; ++i )
            {
                positions.Add( _conv[i] );
            }
            positions.Add( end );

            double sumDiffX = 0.0;
            double sumDiffY = 0.0;
            double totalDistance = 0.0; // total euclidean distance
            for( int i = 0; i < positions.Count - 1; ++i )
            {
                double diffX = positions[i + 1].X - positions[i].X;
                double diffY = positions[i + 1].Y - positions[i].Y;
                sumDiffX = sumDiffX + diffX;
                sumDiffY = sumDiffY + diffY;
                totalDistance = totalDistance + Math.Sqrt( diffX * diffX + diffY * diffY );
            }
            Vector2DStamped direction = new Vector2DStamped( sumDiffX, sumDiffY, begin.Timestamp );
            direction.Normalize( );
            double speed = totalDistance / (end.Timestamp - begin.Timestamp);
            return new Vector2DDuration( direction.Multiply( speed ), begin.Timestamp, end.Timestamp );
        }


        public void SetConvolution( double[] convolution )
        {
            if( convolution == null || convolution.Length <= 0 )
            {
                _cnvMatrix = new double[] { 1.0 };
            }
            else
            {
                double total = 0.0;
                _cnvMatrix = new double[convolution.Length];
                for( int i = 0; i < _cnvMatrix.Length; ++i )
                {
                    total += convolution[i];
                }
                for( int i = 0; i < _cnvMatrix.Length; ++i )
                {
                    _cnvMatrix[i] = convolution[i] / total;
                }
            }
            ConvoluteAllPositions( );
        }


        private void ConvoluteAllPositions( )
        {
            _conv.Clear( );
            for( int i = 0; i < _raw.Count; ++i )
            {
                _conv.Add( ConvolutePosition( _raw[i].Timestamp, i ) );
            }
        }


        private Vector2DStamped ConvolutePosition( double timestamp, int index )
        {
            List<Vector2D> data = GetRawPositions( index + 1 - _cnvMatrix.Length, index );
            Vector2DStamped pos = new Vector2DStamped( );
            pos.Timestamp = timestamp;
            for( int i = 0; i < _cnvMatrix.Length; ++i )
            {
                pos.X = pos.X + data[i].X * _cnvMatrix[i];
                pos.Y = pos.Y + data[i].Y * _cnvMatrix[i];
            }
            return pos;
        }


        private List<Vector2D> GetRawPositions( int begin, int end )
        {
            List<Vector2D> data = new List<Vector2D>( );
            if( _raw.Count <= 0 )
            {   // empty list
                while( begin < end )
                {
                    data.Add( new Vector2D( ) );
                    begin = begin + 1;
                }
            }
            else
            {   // non-empty list
                while( begin <= end )
                {
                    if( begin < 0 ) // unknown data before the first (fill with the first)
                        data.Add( _raw[0] );
                    else if( begin >= _raw.Count ) // unknown data after the last (fill with the last)
                        data.Add( _raw[_raw.Count - 1] );
                    else // known data
                        data.Add( _raw[begin] );
                    begin = begin + 1;
                }
            }
            return data;
        }


        private void NotifyPropertyChanged( [CallerMemberName] string name = "" )
        {
            PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( name ) );
        }

    }
}
