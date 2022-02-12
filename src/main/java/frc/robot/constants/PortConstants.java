// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class PortConstants {
    //Controller ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int MANIPULATOR_CONTROLLER_PORT = 1;
  
    //These are the Intake Ports
    //TODO Confirm that these I.Ds are right
    public static final int kIntakeMotorPort = 8;
    public static final int kIntakeSolenoidForwardPort = 2;
    public static final int kIntakeSolenoidReversePort = 3;

    //Climber Motors
    //TODO:  Set these ports correctly.  They were conflicting with the drivetrain.
    public static final int ClimbLeftMotorPort = 0;
    public static final int ClimbRightMotorPort = 0;

    //Drivetrain Motors
    public static final int driveTrainLeftLeaderPort = 2;
    public static final int driveTrainLeftFollowerPort = 4;
    
    public static final int driveTrainRightLeaderPort = 1;
    public static final int driveTrainRightFollowerPort = 3;

    //Indexer Motors
    //TODO: set IDs
    public static final int indexerMotorPort = 0;
    public static final int indexerKickupMotorPort = 0;
 
    //Indexer sensor ports
    //TODO: set IDs
    public static final int initialIndexerBeamBreakPort = 0;
    public static final int kickupIndexerBeamBreakPort = 0;
    public static final int indexerColorSensorPort = 0;


}