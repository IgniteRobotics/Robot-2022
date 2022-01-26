// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climber extends SubsystemBase {
  public static final double CLIMB_EFFORT_UP = 1;
  public static final double CLIMB_EFFORT_DOWN_ENGAGE = 0.2;
  public static final double CLIMB_EFFORT_DOWN = 1;

  private WPI_TalonFX climberLeader;
  private WPI_TalonFX climberFollower;

  public static final int CLIMBER_FORWARD_LIMIT = 290000;
  public static final int CLIMBER_REVERSE_LIMIT = 10000;

  private final int kTimeoutMs = 30;
  private final int kSlotIdx = 0;
  private final double  kP = 0.2;
  private final double  kF = 0;
  private final double  kI = 0;
  private final double  kD = 0;
  private final int kPIDLoopIdx = 0;

  private boolean leaderCurrentStopped;
  private boolean followerCurrentStopped;

  public static final double safeReduceEffort = 0.08;
  public static final double safeStatorLimit = 0.3;

  public static final double rampDownFrames = 25;
  private int framesSinceRamp = 0;
  private double initialRampingEffort = 0;
  private boolean isRampingDown = false;

  public Climber() {
    climberLeader = new WPI_TalonFX(MotorConstants.kLeftClimberMotorPort);
    climberFollower = new WPI_TalonFX(MotorConstants.kRightClimberMotorPort);

    climberLeader.configFactoryDefault();
    climberFollower.configFactoryDefault();

    // climberLeader.configOpenloopRamp(1);
    // climberFollower.configOpenloopRamp(1);

    climberFollower.setInverted(true);

    climberLeader.setNeutralMode(NeutralMode.Brake);
    climberFollower.setNeutralMode(NeutralMode.Brake);

    climberLeader.configForwardSoftLimitThreshold(CLIMBER_FORWARD_LIMIT);
    climberLeader.configReverseSoftLimitThreshold(CLIMBER_REVERSE_LIMIT);
    climberLeader.configForwardSoftLimitEnable(true, 0);
    climberLeader.configReverseSoftLimitEnable(true, 0);

    climberFollower.configForwardSoftLimitThreshold(CLIMBER_FORWARD_LIMIT);
    climberFollower.configReverseSoftLimitThreshold(CLIMBER_REVERSE_LIMIT);
    climberFollower.configForwardSoftLimitEnable(true, 0);
    climberFollower.configReverseSoftLimitEnable(true, 0);
    
    //addChild("climberLeader- Climber", climberLeader);
    //addChild("climberFollower- Climber", climberFollower);
  }

  @Override
  public void periodic() {
    if(isRampingDown) {
      framesSinceRamp++;

      if(framesSinceRamp >= rampDownFrames) {
        isRampingDown = false;
        stop();
      } else {
        climberLeader.set(ControlMode.PercentOutput, (1 - framesSinceRamp / (double) rampDownFrames) * initialRampingEffort);
        climberFollower.set(ControlMode.PercentOutput, (1 - framesSinceRamp / (double) rampDownFrames) * initialRampingEffort);
      }
    }
  }

  private void configureMotionMagic() {
    climberLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    climberLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
    
    climberFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    climberFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
    
    climberLeader.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		climberLeader.config_kF(kSlotIdx, kF, kTimeoutMs);
		climberLeader.config_kP(kSlotIdx, kP, kTimeoutMs);
		climberLeader.config_kI(kSlotIdx, kI, kTimeoutMs);
		climberLeader.config_kD(kSlotIdx, kD, kTimeoutMs);
		climberLeader.configMotionCruiseVelocity(5525, kTimeoutMs);
		climberLeader.configMotionAcceleration(5525, kTimeoutMs);
  }

  public void goUp() {
    // TODO make this shuffleboard changeable
    climberLeader.set(ControlMode.PercentOutput, CLIMB_EFFORT_UP);
    climberFollower.set(ControlMode.PercentOutput, CLIMB_EFFORT_UP);
  }

  public void goDown() {
    isRampingDown = false;
    climberLeader.set(ControlMode.PercentOutput, -CLIMB_EFFORT_DOWN);
    climberFollower.set(ControlMode.PercentOutput, -CLIMB_EFFORT_DOWN);
  }

  public void goDownEngage() {
    climberLeader.set(ControlMode.PercentOutput, -CLIMB_EFFORT_DOWN_ENGAGE);
    climberFollower.set(ControlMode.PercentOutput, -CLIMB_EFFORT_DOWN_ENGAGE);
  }

  public void go(double effort) {
    climberLeader.set(ControlMode.PercentOutput, effort);
    climberFollower.set(ControlMode.PercentOutput, effort);
  }

  public void setOpenLoop(double percentage) {
    climberLeader.set(ControlMode.PercentOutput, percentage);
  }

  public void reduceMaxSafe() {
    if(!leaderCurrentStopped) {
      climberLeader.set(ControlMode.PercentOutput, -safeReduceEffort);
    }

    if(!followerCurrentStopped) {
      climberFollower.set(ControlMode.PercentOutput, -safeReduceEffort);
    }

    if(climberLeader.getSupplyCurrent() > safeStatorLimit) {
      leaderCurrentStopped = true;
      climberLeader.stopMotor();
      climberLeader.setSelectedSensorPosition(0);
    }

    if(climberFollower.getSupplyCurrent() > safeStatorLimit) {
      followerCurrentStopped = true;
      climberFollower.stopMotor();
      climberFollower.setSelectedSensorPosition(0);
    }
  }

  /**
   * Disables and enables soft limits on climber, depending on parameter set. 
   * Also removes follower mode on climberFollower.
   * 
   * For reducing climber to zero
   * @param set
   */
  public void setNoLimits(boolean set) {
    climberLeader.configForwardSoftLimitEnable(!set, 0);
    climberLeader.configReverseSoftLimitEnable(!set, 0);

    climberFollower.configForwardSoftLimitEnable(!set, 0);
    climberFollower.configReverseSoftLimitEnable(!set, 0);
  }

  public boolean bothCurrentStopped() {
    return leaderCurrentStopped && followerCurrentStopped;
  }

  public void setOpenLoop(double percentage, double deadband) {
    // percentage = Util.applyDeadband(percentage, Constants.CLIMBER_JOG_DEADBAND);
    // We'll worry about deadband here later. Besides, it makes more sense
    // to use the built-in falon motor deadbands
    setOpenLoop(percentage);
  }

  public void setMotionMagicPosition(double position) {
    climberLeader.set(ControlMode.MotionMagic, position);
  }

  public boolean isMotionMagicDone() {
    // return Math.abs(climberLeader.getClosedLoopTarget() - this.getEncoderPos())
    // <= TOLERANCE;
    // motion magic is a little too much for this, let's focus on this later
    return true;

  }
  public void zeroEncoders() {
    climberLeader.setSelectedSensorPosition(0);
    climberFollower.setSelectedSensorPosition(0);
  }

  public int getEncoderPos() {
    return (int) climberLeader.getSelectedSensorPosition();
  }

  public double getEncoderVel() {
    return climberLeader.getSelectedSensorVelocity();
  }

  public double getMasterVoltage() {
    return climberLeader.getMotorOutputVoltage();
  }

  public double getFollowerVoltage() {
    return climberFollower.getMotorOutputVoltage();
  }

  public double getPercentOutput() {
    return climberLeader.getMotorOutputPercent();
  }

  public double getMasterCurrent() {
    return climberLeader.getOutputCurrent();
  }

  public void zeroSensors() {
    climberLeader.setSelectedSensorPosition(0);
  }

  public boolean isFwdLimitTripped() {
    return climberLeader.getSensorCollection().isFwdLimitSwitchClosed() != 0;
  }

  public boolean isRevLimitTripped() {
    return climberLeader.getSensorCollection().isRevLimitSwitchClosed() != 0;
  }

  public void resetCurrentLimits() {
    leaderCurrentStopped = followerCurrentStopped = false;
  }

  public void stopWithRamping() {
    if(!isRampingDown) {
      isRampingDown = true;

      framesSinceRamp = 0;
      initialRampingEffort = climberLeader.get();
    }
  }

  public void stop() {
    climberLeader.stopMotor();
    climberFollower.stopMotor();
  }
}