package frc.robot.constants;
//contains all constants present in the climber
public class ClimbConstants {
    //Motor Ports
    public static final int LeftMotorPort = 1;
    public static final int RightMotorPort = 2;

    //Climb Limits
    public static final int CLIMBER_FORWARD_LIMIT = 290000;
    public static final int CLIMBER_REVERSE_LIMIT = 10000;
    
    //TODO find out what these things do
    public static final double safeReduceEffort = 0.08;
    public static final double safeStatorLimit = 0.3;
    
    public static final double rampDownFrames = 25;

    public static final double CLIMB_EFFORT_UP = 1;
    public static final double CLIMB_EFFORT_DOWN_ENGAGE = 0.2;
    public static final double CLIMB_EFFORT_DOWN = 1;
}
