package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveConstants {
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double ksVolts = 0.64745;
    public static final double kvVoltSecondsPerMeter = 2.2452;
    public static final double kaVoltSecondsSquaredPerMeter = 0.32914;
    public static final double kTrackwidthMeters = 0.67565;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kPDriveVel = 3.0944;

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
}