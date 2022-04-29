// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.igniterobotics.robotbase.preferences.DoublePreference;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.PortConstants;

public class Climber extends SubsystemBase {
  public static final int DEPLOYED_THRESHOLD = 100000;

  private DoublePreference climbUpEffortPref = new DoublePreference("Climb Up Effort", 1);
  private DoublePreference climbDownEffortPref = new DoublePreference("Climb Down Effort", 1);

  private ReportingNumber climberLeftReporter = new ReportingNumber("Climber Left Pos", ReportingLevel.TEST);
  private ReportingNumber climberRightReporter = new ReportingNumber("Climber Right Pos", ReportingLevel.TEST);
  private ReportingNumber climberCurrent = new ReportingNumber("Climber Current", ReportingLevel.TEST);

  private WPI_TalonFX climberLeft;
  private WPI_TalonFX climberRight;

  private final int kTimeoutMs = 30;
  private final int kSlotIdx = 0;
  private final double kP = 0.2;
  private final double kF = 0;
  private final double kI = 0;
  private final double kD = 0;
  private final int kPIDLoopIdx = 0;

  private boolean leftCurrentStopped;
  private boolean rightCurrentStopped;
  private boolean wasDeployed = false;

  private int framesSinceRamp = 0;
  private double initialRampingEffort = 0;
  private boolean isRampingDown = false;

  public Climber() {
    // Assigns motorports to motors
    climberLeft = new WPI_TalonFX(PortConstants.ClimbLeftMotorPort);
    climberRight = new WPI_TalonFX(PortConstants.ClimbRightMotorPort);

    // climberLeft.configOpenloopRamp(1);
    // climberRight.configOpenloopRamp(1);

    // Inverts the right climber to make both climbers work in sinc
    climberRight.setInverted(true);

    // Sets motors to automatically break
    climberLeft.setNeutralMode(NeutralMode.Brake);
    climberRight.setNeutralMode(NeutralMode.Brake);

    // Sets a limit for the climber motor
    climberLeft.configForwardSoftLimitThreshold(ClimbConstants.CLIMBER_FORWARD_LIMIT_LEFT);
    climberLeft.configReverseSoftLimitThreshold(ClimbConstants.CLIMBER_REVERSE_LIMIT_LEFT);
    climberLeft.configForwardSoftLimitEnable(true, 0);
    climberLeft.configReverseSoftLimitEnable(false, 0);

    climberRight.configForwardSoftLimitThreshold(ClimbConstants.CLIMBER_FORWARD_LIMIT_RIGHT);
    climberRight.configReverseSoftLimitThreshold(ClimbConstants.CLIMBER_REVERSE_LIMIT_RIGHT);
    climberRight.configForwardSoftLimitEnable(true, 0);
    climberRight.configReverseSoftLimitEnable(false, 0);
  }

  @Override
  // makes climber smoothly raise
  public void periodic() {
    climberLeftReporter.set(climberLeft.getSelectedSensorPosition());
    climberRightReporter.set(climberRight.getSelectedSensorPosition());
    climberCurrent.set(climberRight.getSupplyCurrent());

    if(getEncoderPos() >= DEPLOYED_THRESHOLD) {
      wasDeployed = true;
    }
  }

  public void reset() {
    wasDeployed = false;
  }

  public boolean wasDeployed() {
    return wasDeployed;
  }

  private void configureMotionMagic() {
    climberLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    climberLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    climberRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    climberRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    climberLeft.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    climberLeft.config_kF(kSlotIdx, kF, kTimeoutMs);
    climberLeft.config_kP(kSlotIdx, kP, kTimeoutMs);
    climberLeft.config_kI(kSlotIdx, kI, kTimeoutMs);
    climberLeft.config_kD(kSlotIdx, kD, kTimeoutMs);
    climberLeft.configMotionCruiseVelocity(5525, kTimeoutMs);
    climberLeft.configMotionAcceleration(5525, kTimeoutMs);
  }

  // Control to make climber extend
  public void goUp() {
    // TODO make this shuffleboard changeable
    climberLeft.set(ControlMode.PercentOutput, Math.abs(climbUpEffortPref.getValue()));
    climberRight.set(ControlMode.PercentOutput, Math.abs(climbUpEffortPref.getValue()));
  }

  // control to make climber pull robot up
  public void goDown() {
    climberLeft.set(ControlMode.PercentOutput, -Math.abs(climbDownEffortPref.getValue()));
    climberRight.set(ControlMode.PercentOutput, -Math.abs(climbDownEffortPref.getValue()));
  }

  // control to make climber pull robot up slowly
  public void goDownEngage() {
    climberLeft.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_EFFORT_DOWN_ENGAGE);
    climberRight.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_EFFORT_DOWN_ENGAGE);
  }

  public void go(double effort) {
    climberLeft.set(ControlMode.PercentOutput, effort);
    climberRight.set(ControlMode.PercentOutput, effort);
  }

  public void setOpenLoop(double percentage) {
    climberLeft.set(ControlMode.PercentOutput, percentage);
  }

  public void reduceMaxSafe() {
    if (!leftCurrentStopped) {
      climberLeft.set(ControlMode.PercentOutput, -ClimbConstants.SAFE_REDUCE_EFFORT);
    }

    if (!rightCurrentStopped) {
      climberRight.set(ControlMode.PercentOutput, -ClimbConstants.SAFE_REDUCE_EFFORT);
    }

    if (climberLeft.getSupplyCurrent() > ClimbConstants.SAFE_STATOR_LIMIT) {
      leftCurrentStopped = true;
      climberLeft.stopMotor();
      climberLeft.setSelectedSensorPosition(0);
    }

    if (climberRight.getSupplyCurrent() > ClimbConstants.SAFE_STATOR_LIMIT) {
      rightCurrentStopped = true;
      climberRight.stopMotor();
      climberRight.setSelectedSensorPosition(0);
    }
  }

  public boolean isClimbLimitMet() {
    return climberLeft.getSupplyCurrent() > ClimbConstants.SAFE_CLIMB_LIMIT || climberRight.getSupplyCurrent() > ClimbConstants.SAFE_CLIMB_LIMIT;
  }

  /**
   * Disables and enables soft limits on climber, depending on parameter set.
   * Also removes follower mode on climberFollower.
   * 
   * For reducing climber to zero
   * 
   * @param set
   */
  public void setNoLimits(boolean set) {
    climberLeft.configForwardSoftLimitEnable(!set, 0);
    climberLeft.configReverseSoftLimitEnable(!set, 0);

    climberRight.configForwardSoftLimitEnable(!set, 0);
    climberRight.configReverseSoftLimitEnable(!set, 0);
  }

  public boolean bothCurrentStopped() {
    return leftCurrentStopped && rightCurrentStopped;
  }

  // public void setOpenLoop(double percentage, double deadband) {
  // percentage = Util.applyDeadband(percentage, Constants.CLIMBER_JOG_DEADBAND);
  // We'll worry about deadband here later. Besides, it makes more sense
  // to use the built-in falon motor deadbands
  // setOpenLoop(percentage);
  // }

  public void setMotionMagicPosition(double position) {
    climberLeft.set(ControlMode.MotionMagic, position);
  }

  // public boolean isMotionMagicDone() {
  // return Math.abs(climberLeft.getClosedLoopTarget() - this.getEncoderPos())
  // <= TOLERANCE;
  // motion magic is a little too much for this, let's focus on this later
  // return true;

  // }
  public void zeroEncoders() {
    climberLeft.setSelectedSensorPosition(0);
    climberRight.setSelectedSensorPosition(0);
  }

  public int getEncoderPos() {
    return (int) climberLeft.getSelectedSensorPosition();
  }

  public double getEncoderVel() {
    return climberLeft.getSelectedSensorVelocity();
  }

  public double getMasterVoltage() {
    return climberLeft.getMotorOutputVoltage();
  }

  public double getFollowerVoltage() {
    return climberRight.getMotorOutputVoltage();
  }

  public double getPercentOutput() {
    return climberLeft.getMotorOutputPercent();
  }

  public double getMasterCurrent() {
    return climberLeft.getOutputCurrent();
  }

  public void zeroSensors() {
    climberLeft.setSelectedSensorPosition(0);
  }

  public boolean isFwdLimitTripped() {
    return climberLeft.getSensorCollection().isFwdLimitSwitchClosed() != 0;
  }

  public boolean isRevLimitTripped() {
    return climberLeft.getSensorCollection().isRevLimitSwitchClosed() != 0;
  }

  public void resetCurrentLimits() {
    leftCurrentStopped = rightCurrentStopped = false;
  }

  public void stopWithRamping() {
    if (!isRampingDown) {
      isRampingDown = true;

      framesSinceRamp = 0;
      initialRampingEffort = climberLeft.get();
    }
  }

  public void stop() {
    climberLeft.stopMotor();
    climberRight.stopMotor();
  }
}