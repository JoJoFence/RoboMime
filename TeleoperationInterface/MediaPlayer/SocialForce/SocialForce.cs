using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;

namespace MediaPlayer.SocialForceModule
{
    internal sealed class SocialForce : INotifyPropertyChanged
    {
        private const double A = 10.0;
        private const double B = 34.0;
        private const double k = 4.9;
        private const double RADIUS = 1.0;
        private const double weight_lambda = 0.29;
        private Dictionary<Agent, Vector> _acceleration = new Dictionary<Agent, Vector>( );
        private Dictionary<Agent, Vector> _currentVelocity = new Dictionary<Agent, Vector>( );
        private Dictionary<Agent, Vector> _sfDiffs = new Dictionary<Agent, Vector>( );
        private Dictionary<Agent, Vector> _sfEstimates = new Dictionary<Agent, Vector>( );
        private Dictionary<Agent, Vector> _sfPosition = new Dictionary<Agent, Vector>( );
        private Dictionary<Agent, Vector> _sfValues = new Dictionary<Agent, Vector>( );
        private double _accelerationDuration = 1.0;
        private double _desireVelocityDuration = 1.0;
        private double _desireVelocityOffset = 1.0;
        private double _velocityDuration = 1.0;

        //private const double A = 1.13;
        //private const double B = 71.0;
        //private const double k = 1.52;
        //private const double weight_lambda = 0.29;
        //private double _accelerationDuration = 0.5;
        //private double _desireVelocityDuration = 1.5;
        //private double _desireVelocityOffset = 3.0;
        //private double _velocityDuration = 1.5;


        public SocialForce( )
        {
            Agents = new List<Agent>( );
        }


        public event PropertyChangedEventHandler PropertyChanged;


        public List<Agent> Agents
        {
            get;
            private set;
        }


        public double AccelerationDuration
        {
            get { return _accelerationDuration; }
            set
            {
                _accelerationDuration = value;
                NotifyPropertyChanged( );
            }
        }


        public double DesireVelocityDuration
        {
            get { return _desireVelocityDuration; }
            set
            {
                _desireVelocityDuration = value;
                NotifyPropertyChanged( );
            }
        }


        public double DesireVelocityOffset
        {
            get { return _desireVelocityOffset; }
            set
            {
                _desireVelocityOffset = value;
                NotifyPropertyChanged( );
            }
        }


        public double VelocityDuration
        {
            get { return _velocityDuration; }
            set
            {
                _velocityDuration = value;
                NotifyPropertyChanged( );
            }
        }


        public void AddAgent( Agent agent )
        {
            if( !_acceleration.ContainsKey( agent ) )
            {
                Agents.Add( agent );
                _acceleration.Add( agent, new Vector( ) );
                _currentVelocity.Add( agent, new Vector( ) );
                _sfDiffs.Add( agent, new Vector( ) );
                _sfEstimates.Add( agent, new Vector( ) );
                _sfPosition.Add( agent, new Vector( ) );
                _sfValues.Add( agent, new Vector( ) );
            }
        }


        public Vector GetAcceleration( Agent agent )
        {
            if( _acceleration.ContainsKey( agent ) )
            {
                return _acceleration[agent];
            }
            return new Vector( );
        }


        public Vector GetCurrentvelocity( Agent agent )
        {
            if( _currentVelocity.ContainsKey( agent ) )
            {
                return _currentVelocity[agent];
            }
            return new Vector( );
        }


        public Vector GetSocialForce( Agent agent )
        {
            if( _sfValues.ContainsKey( agent ) )
            {
                return _sfValues[agent];
            }
            return new Vector( );
        }


        public Vector GetSocialForceDifference( Agent agent )
        {
            if( _sfDiffs.ContainsKey( agent ) )
            {
                return _sfDiffs[agent];
            }
            return new Vector( );
        }


        public Vector GetSocialForceEstimation( Agent agent )
        {
            if( _sfEstimates.ContainsKey( agent ) )
            {
                return _sfEstimates[agent];
            }
            return new Vector( );
        }


        public Vector GetSocialForcePosition( Agent agent )
        {
            if( _sfPosition.ContainsKey( agent ) )
            {
                return _sfPosition[agent];
            }
            return new Vector( );
        }


        public void Update( double timestamp )
        {
            for( int i = 0; i < Agents.Count; ++i )
            {
                Vector i_Position = Agents[i].GetPosition( timestamp, _velocityDuration );
                Vector i_DesireVelocity = Agents[i].GetVelocity( timestamp - _desireVelocityOffset, timestamp - _desireVelocityOffset + _desireVelocityDuration );
                Vector i_DesireForce = i_DesireVelocity / k;
                Vector i_CurrentVelocity = Agents[i].GetVelocity( timestamp - _velocityDuration, timestamp );
                _sfValues[Agents[i]] = new Vector( );
                for( int j = 0; j < Agents.Count; ++j )
                {
                    if( i != j )
                    {
                        Vector j_Position = Agents[j].GetPosition( timestamp, _velocityDuration );
                        Vector d_ij = i_Position - j_Position;
                        Vector d_ji = j_Position - i_Position;
                        double radii = RADIUS + RADIUS;
                        double distance = d_ij.Length;
                        d_ij.Normalize( );
                        Vector social_force = Vector.Multiply( A * Math.Exp( (radii - distance) / B ), d_ij );

                        double theta_weight = Vector2D.Angle( i_CurrentVelocity.X, i_CurrentVelocity.Y, d_ji.X, d_ji.Y );
                        double weight = weight_lambda + (1.0 - weight_lambda) * ((1.0 + Math.Cos( theta_weight )) / 2.0);
                        _sfValues[Agents[i]] += social_force * weight;
                    }
                }
            }

            /*
            for( int i = 0; i < Agents.Count; ++i )
            {
                double t_i = double.PositiveInfinity;
                Vector i_Position = Agents[i].GetPosition( timestamp, _velocityDuration );
                Vector i_DesireVelocity = Agents[i].GetVelocity( timestamp - _desireVelocityOffset, timestamp - _desireVelocityOffset + _desireVelocityDuration );
                Vector i_Acceleration = Agents[i].GetAcceleration( timestamp - _accelerationDuration, timestamp, _velocityDuration );
                Vector i_CurrentVelocity = Agents[i].GetVelocity( timestamp - _velocityDuration, timestamp );

                _acceleration[Agents[i]] = i_Acceleration;
                _currentVelocity[Agents[i]] = i_CurrentVelocity;
                _sfDiffs[Agents[i]] = new Vector( );
                _sfEstimates[Agents[i]] = new Vector( );
                _sfPosition[Agents[i]] = i_Position;
                _sfValues[Agents[i]] = new Vector( );

                for( int j = 0; j < Agents.Count; ++j )
                {
                    if( i != j )
                    {
                        Vector j_Position = Agents[j].GetPosition( timestamp, _velocityDuration );
                        Vector j_DesireVelocity = Agents[j].GetVelocity( timestamp - _desireVelocityOffset, timestamp - _desireVelocityOffset + _desireVelocityDuration );

                        Vector d_ij = i_Position - j_Position;
                        Vector v_ij = j_DesireVelocity - i_DesireVelocity;
                        double theta = Vector2D.Angle( v_ij.X, v_ij.Y, d_ij.X, d_ij.Y );
                        if( Math.Abs( theta ) <= Math.PI * 0.25 )
                        {
                            double tri_base = Math.Cos( theta ) * Vector2D.Magnitude( d_ij );
                            double t_ij = Math.Abs( tri_base ) / Vector2D.Magnitude( v_ij );
                            t_i = Math.Min( t_i, t_ij );
                        }
                    }
                }

                if( !double.IsInfinity( t_i ) )
                {
                    for( int j = 0; j < Agents.Count; ++j )
                    {
                        if( i != j )
                        {
                            Vector j_Position = Agents[j].GetPosition( timestamp, _velocityDuration );
                            Vector j_DesireVelocity = Agents[j].GetVelocity( timestamp - _desireVelocityOffset, timestamp - _desireVelocityOffset + _desireVelocityDuration );

                            Vector d_ij = i_Position - j_Position;
                            Vector v_ij = j_DesireVelocity - i_DesireVelocity;
                            Vector point_t_i = j_Position + v_ij * t_i;
                            Vector dp_ij = i_Position - point_t_i;

                            // compute social force
                            double dp_ij_mag = Vector2D.Magnitude( dp_ij );
                            if( dp_ij_mag != 0.0 )
                            {   // if this is zero, everything will be zero from now on
                                double a_vt = A * (Vector2D.Magnitude( i_DesireVelocity ) / t_i);
                                double e = Math.Exp( -Vector2D.Magnitude( dp_ij ) / B );
                                Vector dp_ij_norm = dp_ij / dp_ij_mag;
                                Vector social_force = new Vector( a_vt * e * dp_ij_norm.X, a_vt * e * dp_ij_norm.Y );

                                // compute weight
                                Vector d_ji = j_Position - i_Position;
                                double theta_weight = Vector2D.Angle( i_DesireVelocity.X, i_DesireVelocity.Y, d_ji.X, d_ji.Y );
                                double weight = weight_lambda + (1.0 - weight_lambda) * ((1.0 + Math.Cos( theta_weight )) / 2.0);
                                Vector social_force_weighted = social_force * weight;
                                //if( !double.IsNaN( social_force_weighted.X ) && !double.IsNaN( social_force_weighted.Y ) )
                                {
                                    _sfValues[Agents[i]] = _sfValues[Agents[i]] + social_force_weighted;

                                    // compute social force difference
                                    Vector estimated_social_force = i_Acceleration - k * (i_DesireVelocity - i_CurrentVelocity);
                                    _sfEstimates[Agents[i]] = _sfEstimates[Agents[i]] + estimated_social_force;

                                    double rad_estimate = Vector2D.Angle( estimated_social_force.X, estimated_social_force.Y, social_force_weighted.X, social_force_weighted.Y );
                                    double Bx = Math.Max( 0.0, Vector2D.Magnitude( estimated_social_force ) * Math.Cos( rad_estimate ) );
                                    double sfdiff = Math.Max( 0.0, Vector2D.Magnitude( social_force_weighted ) - Bx );
                                    _sfDiffs[Agents[i]] = new Vector( _sfDiffs[Agents[i]].X + sfdiff, _sfDiffs[Agents[i]].Y + sfdiff );
                                }
                            }
                        }
                    }
                }
            }
            */
        }


        private void NotifyPropertyChanged( [CallerMemberName] string name = "" )
        {
            PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( name ) );
        }

    }
}
