/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

// import badlog.lib.BadLog;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.igniterobotics.robotbase.calc.SensorProfile;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import frc.robot.Main;
import frc.robot.constants.Constants;
import frc.robot.constants.PortConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drivetrain extends SubsystemBase {

    /////////////////// CONSTANTS /////////////////// 

    public static final double SPEED_RATE_LIMIT_ARCADE = 3;
    public static final double ROTATION_RATE_LIMIT_ARCADE = 3.0;

    public static final double SLOW_MODE_SPEED_MODIFIER = 0.5;

    public static final double OPEN_LOOP_RAMP = 0.25;

    //default arcade drive modifiers
    public static final double VELOCITY_RAMP_EXPONENT = 2;
    public static final double VELOCITY_LIMIT_MULTIPLIER = 1;
    public static final double TURN_RAMP_EXPONENT = 2;
    public static final double TURN_LIMIT_MULTIPLIER = 0.4;

    public static int WHEEL_DIAMETER_INCHES = 6; //in inches
    public static double WHEEL_DIAMETER_METERS = 0.1524;

    public static double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    /////////////////// END CONSTANTS ///////////////////

    private final WPI_TalonFX leftLeader = new WPI_TalonFX(PortConstants.driveTrainLeftLeaderPort);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(PortConstants.driveTrainLeftFollowerPort);

    private final WPI_TalonFX rightLeader = new WPI_TalonFX(PortConstants.driveTrainRightLeaderPort);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(PortConstants.driveTrainRightFollowerPort);

    private DifferentialDrive m_driveTrain;

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private final DifferentialDriveOdometry m_odometry;

    private Pose2d savedPose;

    private final SlewRateLimiter speedRateLimiter = new SlewRateLimiter(SPEED_RATE_LIMIT_ARCADE);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT_ARCADE);

    // 7.8 is gear ratio
    private SensorProfile sensorProfile = new SensorProfile(2048, 7.8);

    private ReportingNumber leftEncoderPosition = new ReportingNumber("Left Encoder", ReportingLevel.COMPETITON);
    private ReportingNumber rightEncoderPosition = new ReportingNumber("Right Encoder", ReportingLevel.COMPETITON);
    private ReportingNumber leftEncoderVelocity = new ReportingNumber("Left Velocity", ReportingLevel.COMPETITON);
    private ReportingNumber rightEncoderVelocity = new ReportingNumber("Right Velocity", ReportingLevel.COMPETITON);
    private ReportingNumber theta = new ReportingNumber("NavX Angle", ReportingLevel.COMPETITON);

    public Drivetrain() {
        m_odometry = new DifferentialDriveOdometry(navX.getRotation2d()); // assume robot starts at x =0, y=0,
                                                                               // theta = 0

        navX.zeroYaw();

        leftLeader.configFactoryDefault();
        rightLeader.configFactoryDefault();

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        talonConfig.slot0.kP = .01;
        talonConfig.slot0.kI = 0.0;
        talonConfig.slot0.kD = 0.0;
        talonConfig.slot0.integralZone = 400;
        talonConfig.slot0.closedLoopPeakOutput = 1.0;

        talonConfig.openloopRamp = OPEN_LOOP_RAMP;

        leftLeader.configAllSettings(talonConfig);
        leftLeader.enableVoltageCompensation(true);
        leftFollower.configFactoryDefault();

        rightLeader.configAllSettings(talonConfig);
        rightLeader.enableVoltageCompensation(true);
        rightFollower.configFactoryDefault();

        resetEncoders();

        setNeutralMode(NeutralMode.Brake);

        leftLeader.setInverted(false);
        leftFollower.setInverted(false);

        leftLeader.overrideLimitSwitchesEnable(false);
        rightLeader.overrideLimitSwitchesEnable(false);        

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        m_driveTrain = new DifferentialDrive(leftLeader, rightLeader);
    }

    @Override
    public void periodic() {
        m_odometry.update(navX.getRotation2d(), sensorProfile.uToRev(getLeftEncoderPosition()) * WHEEL_CIRCUMFERENCE_METERS,
                      sensorProfile.uToRev(-getRightEncoderPosition()) * WHEEL_CIRCUMFERENCE_METERS);
        leftEncoderPosition.set(getLeftEncoderPosition());              
        rightEncoderPosition.set(getRightEncoderPosition());    
        leftEncoderVelocity.set(getLeftEncoderVel());    
        rightEncoderVelocity.set(getRightEncoderVel());    
        theta.set(navX.getRotation2d().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

        // DifferentialDriveWHeelSpeeds expects meters per second
        return new DifferentialDriveWheelSpeeds(sensorProfile.uToRPS(leftLeader.getSelectedSensorVelocity()) * WHEEL_CIRCUMFERENCE_METERS,
                sensorProfile.uToRPS(-rightLeader.getSelectedSensorVelocity()) * WHEEL_CIRCUMFERENCE_METERS);
    }

    public void resetOdometry() {
        resetEncoders();
        m_odometry.resetPosition(savedPose, Rotation2d.fromDegrees(getAngle()));
    }

    public void resetOdometry(Pose2d startingPose) {
        resetEncoders();
        m_odometry.resetPosition(startingPose, navX.getRotation2d());
    }

    public void arcadeDrive(final double speed, final double rotation, final boolean useSquares) {
        var xSpeed = speedRateLimiter.calculate(safeClamp(speed));
        var zRotation = rotationRateLimiter.calculate(safeClamp(rotation));
        // xSpeed *= Constants.kMaxSpeedMetersPerSecond;
        // zRotation *= Constants.kMaxAngularVelocity;
        m_driveTrain.arcadeDrive(xSpeed, zRotation, useSquares);
    }

    public void tankDrivePower(double leftPower, double rightPower) {
        double leftPowerLimited = safeClamp(leftPower);
        double rightPowerLimited = safeClamp(rightPower);

        m_driveTrain.tankDrive(leftPowerLimited, rightPowerLimited, false);
    }


    // used to drive trajectories
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftLeader.setVoltage(leftVolts);
        this.rightLeader.setVoltage(-rightVolts);
        m_driveTrain.feed();
    }

    private double safeClamp(final double input) {
        if (Double.isNaN(input)) {
            return 0;
        }
        return MathUtil.clamp(input, -1, 1);
    }

    public void driveCurvature(double xSpeed, double zRotation, boolean isQuickTurn) {
        m_driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
    }

    public void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (leftLeader.getSelectedSensorPosition(0) + rightLeader.getSelectedSensorPosition(0)) / 2.0;
    }

    public double getLeftEncoderPosition() {
        return leftLeader.getSelectedSensorPosition(0);
    }

    public double getRightEncoderPosition() {
        return rightLeader.getSelectedSensorPosition(0);
    }

    public void setMaxOutput(final double maxOutput) {
        m_driveTrain.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        navX.zeroYaw();
    }

    /**
     * Returns the absolute position of the yaw axis.
     * 
     * Increments over 360
     * @return
     */
    public double getHeading() {
        return navX.getAngle();
    }

    public void setNeutralMode(final NeutralMode neutralMode) {
        leftLeader.setNeutralMode(neutralMode);
        rightLeader.setNeutralMode(neutralMode);
        leftFollower.setNeutralMode(neutralMode);
        rightFollower.setNeutralMode(neutralMode);
    }

    public void stop() {
        tankDriveVolts(0, 0);
    }

    public double getLeftEncoderVel() {
        return leftLeader.getSelectedSensorVelocity();
    }

    public double getRightEncoderVel() {
        return rightLeader.getSelectedSensorVelocity();
    }

    public double getLeftMasterVoltage() {
        return leftLeader.getMotorOutputVoltage();
    }

    public double getRightMasterVoltage() {
        return rightLeader.getMotorOutputVoltage();
    }

    public double getLeftPercentOutput() {
        return leftLeader.getMotorOutputPercent();
    }

    public double getRightPercentOutput() {
        return rightLeader.getMotorOutputPercent();
    }

    public double getLeftMasterCurrent() {
        return leftLeader.getStatorCurrent();
    }

    public double getRightMasterCurrent() {
        return rightLeader.getStatorCurrent();
    }

    public boolean isConnected() {
        return navX.isConnected();
    }

    public double getAngle() {
        return -navX.getAngle();
    }

    public double getYaw() {
        return -navX.getYaw();
    }

    public double getClosedLoopTarget() {
        return leftLeader.getClosedLoopTarget();
    }

    // TODO: Fix this to be more consistent with inverted and negative voltages.
    // Right now it's sort of a hack
    public void driveDistance(double setpointTicks) {
        // I don't think the arbitary feed forward is really that helpful here
        leftLeader.set(TalonFXControlMode.Position, setpointTicks, DemandType.ArbitraryFeedForward, 0.0007);
        rightLeader.set(TalonFXControlMode.Position, -setpointTicks, DemandType.ArbitraryFeedForward, 0.0007);
    }
}