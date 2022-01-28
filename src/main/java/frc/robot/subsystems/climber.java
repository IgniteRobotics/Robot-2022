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

public class climber extends SubsystemBase {

  private WPI_TalonFX climberLeft;
  private WPI_TalonFX climberRight;

  private final int kTimeoutMs = 30;
  private final int kSlotIdx = 0;
  private final double  kP = 0.2;
  private final double  kF = 0;
  private final double  kI = 0;
  private final double  kD = 0;
  private final int kPIDLoopIdx = 0;

  private boolean leftCurrentStopped;
  private boolean rightCurrentStopped;

  private int framesSinceRamp = 0;
  private double initialRampingEffort = 0;
  private boolean isRampingDown = false;

  public climber() {
    climberLeft = new WPI_TalonFX(ClimbConstants.LeftMotorPort);
    climberRight = new WPI_TalonFX(ClimbConstants.RightMotorPort);

    climberLeft.configFactoryDefault();
    climberRight.configFactoryDefault();

    // climberLeft.configOpenloopRamp(1);
    // climberRight.configOpenloopRamp(1);

    climberRight.setInverted(true);

    climberLeft.setNeutralMode(NeutralMode.Brake);
    climberRight.setNeutralMode(NeutralMode.Brake);

    climberLeft.configForwardSoftLimitThreshold(ClimbConstants.CLIMBER_FORWARD_LIMIT);
    climberLeft.configReverseSoftLimitThreshold(ClimbConstants.CLIMBER_REVERSE_LIMIT);
    climberLeft.configForwardSoftLimitEnable(true, 0);
    climberLeft.configReverseSoftLimitEnable(true, 0);

    climberRight.configForwardSoftLimitThreshold(ClimbConstants.CLIMBER_FORWARD_LIMIT);
    climberRight.configReverseSoftLimitThreshold(ClimbConstants.CLIMBER_REVERSE_LIMIT);
    climberRight.configForwardSoftLimitEnable(true, 0);
    climberRight.configReverseSoftLimitEnable(true, 0);
    
    addChild("climberLeft- Climber", climberLeft);
    addChild("climberRight- Climber", climberRight);
  }

  @Override
  public void periodic() {
    if(isRampingDown) {
      framesSinceRamp++;

      if(framesSinceRamp >= ClimbConstants.rampDownFrames) {
        isRampingDown = false;
        stop();
      } else {
        climberLeft.set(ControlMode.PercentOutput, (1 - framesSinceRamp / (double) ClimbConstants.rampDownFrames) * initialRampingEffort);
        climberRight.set(ControlMode.PercentOutput, (1 - framesSinceRamp / (double) ClimbConstants.rampDownFrames) * initialRampingEffort);
      }
    }
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

  public void goUp() {
    // TODO make this shuffleboard changeable
    climberLeft.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_EFFORT_UP);
    climberRight.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_EFFORT_UP);
  }

  public void goDown() {
    isRampingDown = false;
    climberLeft.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_EFFORT_DOWN);
    climberRight.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_EFFORT_DOWN);
  }

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

  //public void reduceMaxSafe() {
  //  if(!leftCurrentStopped) {
  //    climberLeft.set(ControlMode.PercentOutput, -ClimbConstants.safeReduceEffort);
  //  }

  //  if(!rightCurrentStopped) {
  //    climberRight.set(ControlMode.PercentOutput, -ClimbConstants.safeReduceEffort);
  //  }

    //if(climberLeft.getSupplyCurrent() > ClimbConstants.safeStatorLimit) {
    //  leftCurrentStopped = true;
    //  climberLeft.stopMotor();
    //  climberLeft.setSelectedSensorPosition(0);
    //}

    //if(climberRight.getSupplyCurrent() > ClimbConstants.safeStatorLimit) {
    //  rightCurrentStopped = true;
    //  climberRight.stopMotor();
    //  climberRight.setSelectedSensorPosition(0);
    //}
  //}

  /**
   * Disables and enables soft limits on climber, depending on parameter set. 
   * Also removes follower mode on climberFollower.
   * 
   * For reducing climber to zero
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

  public void setOpenLoop(double percentage, double deadband) {
    // percentage = Util.applyDeadband(percentage, Constants.CLIMBER_JOG_DEADBAND);
    // We'll worry about deadband here later. Besides, it makes more sense
    // to use the built-in falon motor deadbands
    setOpenLoop(percentage);
  }

  public void setMotionMagicPosition(double position) {
    climberLeft.set(ControlMode.MotionMagic, position);
  }

  public boolean isMotionMagicDone() {
    // return Math.abs(climberLeft.getClosedLoopTarget() - this.getEncoderPos())
    // <= TOLERANCE;
    // motion magic is a little too much for this, let's focus on this later
    return true;

  }
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
    if(!isRampingDown) {
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