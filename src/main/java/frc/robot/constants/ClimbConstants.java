package frc.robot.constants;
//contains all constants present in the climber
public class ClimbConstants {
    //Motor ports
    public static final int LeftMotorPort = 1;
    public static final int RightMotorPort = 2;

    //Climb limits
    public static final int CLIMBER_FORWARD_LIMIT = 290000;
    public static final int CLIMBER_REVERSE_LIMIT = 10000;

    //Defines speeds for climber movement
    public static final double CLIMB_EFFORT_UP = 1;
    public static final double CLIMB_EFFORT_DOWN_ENGAGE = 0.2;
    public static final double CLIMB_EFFORT_DOWN = 1;

    //Helps climb motors stop smoothly  
    public static final double rampDownFrames = 25;

    //Maybe not necissary
    //public static final double safeStatorLimit = 0.3;
    //public static final double safeReduceEffort = 0.25;
}
