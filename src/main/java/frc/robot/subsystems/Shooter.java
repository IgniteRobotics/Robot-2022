// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.igniterobotics.robotbase.preferences.DoublePreference;
import com.igniterobotics.robotbase.reporting.ReportingBoolean;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class Shooter extends SubsystemBase {
  private final DoublePreference shooterFeedEffort = new DoublePreference("Shooter Feed Effort");
  private final ReportingBoolean isSetpointMet = new ReportingBoolean("Setpoint Met", ReportingLevel.COMPETITON);
  private final ReportingNumber shooterVelocityReporter = new ReportingNumber("Shooter Actual Velocity",
      ReportingLevel.COMPETITON);
  private final ReportingNumber shooterCurrent1 = new ReportingNumber("Shooter Current 1", ReportingLevel.COMPETITON);
  private final ReportingNumber shooterCurrent2 = new ReportingNumber("Shooter Current 2", ReportingLevel.COMPETITON);

  private WPI_TalonFX leaderMotor = new WPI_TalonFX(PortConstants.shooterLeaderPort); // shooter
  private WPI_TalonFX followerMotor = new WPI_TalonFX(PortConstants.shooterFollowerPort);

  private CANSparkMax feedMotor = new CANSparkMax(PortConstants.shooterFeedPort, MotorType.kBrushless);

  private double setpointVelocity;
  private static final double SETPOINT_TOLERANCE = 100;

  public static final int CURRENT_LIMIT = 20;
  public static final int CURRENT_LIMIT_THRESHOLD = 20;
  public static final int CURRENT_LIMIT_TIME = 1;

  public static final double kF = 0.05568;
  public static final double kP = 0.2;
  public static final double kD = 0.03;

  private final DoublePreference kFPref = new DoublePreference("Shooter kF", kF);
  private final DoublePreference kPPref = new DoublePreference("Shooter kP", kP);
  private final DoublePreference kDPref = new DoublePreference("Shooter kD", kD);

  private final int setpointFramesRequirement = 20;
  private int setpointFrames = 0;

  public Shooter() {
    leaderMotor.setNeutralMode(NeutralMode.Coast);
    followerMotor.setNeutralMode(NeutralMode.Coast);

    leaderMotor.config_kF(0, kF);
    leaderMotor.config_kP(0, kP);
    leaderMotor.config_kD(0, kD);
    followerMotor.config_kF(0, kF);
    followerMotor.config_kP(0, kP);
    followerMotor.config_kD(0, kD);

    feedMotor.setInverted(true);

    leaderMotor.setInverted(false);
    followerMotor.setInverted(true);

    leaderMotor.enableVoltageCompensation(true);
    leaderMotor.configVoltageCompSaturation(11);
    followerMotor.enableVoltageCompensation(true);
    followerMotor.configVoltageCompSaturation(11);

    feedMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    feedMotor.burnFlash();
  }

  public void runVelocity(double velocity) {
    this.setpointVelocity = velocity;
    leaderMotor.set(ControlMode.Velocity, Math.abs(velocity));
    followerMotor.set(ControlMode.Velocity, Math.abs(velocity));
  }

  public void runFeed() {
    feedMotor.set(shooterFeedEffort.getValue());
  }

  public void stop() {
    leaderMotor.set(ControlMode.PercentOutput, 0);
    followerMotor.set(ControlMode.PercentOutput, 0);
    feedMotor.set(0);
    setpointVelocity = 0;
  }

  @Override
  public void periodic() {
    shooterVelocityReporter.set(leaderMotor.getSelectedSensorVelocity());
    shooterCurrent1.set(leaderMotor.getStatorCurrent());
    shooterCurrent2.set(followerMotor.getStatorCurrent());
    isSetpointMet.set(isSetpointMet());

    if(isVelocityMet() && setpointVelocity > 0) {
      setpointFrames++;
    } else {
      setpointFrames = 0;
    }

    // leaderMotor.config_kF(0, kFPref.getValue());
    // leaderMotor.config_kP(0, kPPref.getValue());
    // leaderMotor.config_kD(0, kDPref.getValue());
    // followerMotor.config_kF(0, kFPref.getValue());
    // followerMotor.config_kP(0, kPPref.getValue());
    // followerMotor.config_kD(0, kDPref.getValue());
  }

  private boolean isVelocityMet() {
    return setpointVelocity - SETPOINT_TOLERANCE < leaderMotor.getSelectedSensorVelocity(0)
        && leaderMotor.getSelectedSensorVelocity(0) < setpointVelocity + SETPOINT_TOLERANCE && setpointVelocity > 0;
  }

  public boolean isSetpointMet() {
    return setpointFrames >= setpointFramesRequirement;
  }
}
