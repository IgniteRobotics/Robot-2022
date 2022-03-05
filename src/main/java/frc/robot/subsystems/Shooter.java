// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.igniterobotics.robotbase.preferences.DoublePreference;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class Shooter extends SubsystemBase {
  private final DoublePreference shooterFeedEffort = new DoublePreference("Shooter Feed Effort");
  private final ReportingNumber shooterVelocityReporter = new ReportingNumber("Shooter Actual Velocity", ReportingLevel.COMPETITON);
  private final ReportingNumber shooterCurrent = new ReportingNumber("Shooter Current", ReportingLevel.COMPETITON);

  private WPI_TalonFX leaderMotor = new WPI_TalonFX(PortConstants.shooterLeaderPort); //shooter
  private WPI_TalonFX followerMotor = new WPI_TalonFX(PortConstants.shooterFollowerPort);

  private CANSparkMax feedMotor = new CANSparkMax(PortConstants.shooterFeedPort, MotorType.kBrushless);

  public Shooter() {
    leaderMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();

    leaderMotor.setNeutralMode(NeutralMode.Coast);
    followerMotor.setNeutralMode(NeutralMode.Coast);

    leaderMotor.config_kF(0, 0.047);
    leaderMotor.config_kP(0, 0.30);

    feedMotor.setInverted(true);

    leaderMotor.setInverted(false);
    followerMotor.setInverted(true);

    followerMotor.follow(leaderMotor);
  }

  public void runVelocity(double velocity) {
    leaderMotor.set(ControlMode.Velocity, Math.abs(velocity));
  }

  public void runFeed() {
    feedMotor.set(shooterFeedEffort.getValue());
  }

  public void stop() {
    leaderMotor.set(ControlMode.PercentOutput, 0);
    followerMotor.set(ControlMode.PercentOutput, 0);
    feedMotor.set(0);
  }

  @Override
  public void periodic() {
    shooterVelocityReporter.set(leaderMotor.getSelectedSensorVelocity());
    shooterCurrent.set(leaderMotor.getSupplyCurrent());
  }
}
