using AuthenticTeleoperation.Configurations;

namespace AuthenticTeleoperation.Robots
{
    internal enum RobovieJointType : int
    {
        SHOULDER_PITCH_R,
        SHOULDER_ROLL_R,
        ELBOW_YAW_R,
        ELBOW_PITCH_R,
        SHOULDER_PITCH_L,
        SHOULDER_ROLL_L,
        ELBOW_YAW_L,
        ELBOW_PITCH_L,
        NECK_YAW,
        HEAD_ROLL,
        HEAD_PITCH,
        _LENGTH = AppConfiguration.NUM_AXES
    }
}
