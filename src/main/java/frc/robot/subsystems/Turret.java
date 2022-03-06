// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class Turret extends SubsystemBase {
  private ReportingNumber turretPosition = new ReportingNumber("Turret Position", ReportingLevel.COMPETITON);

  private CANSparkMax turretMotor = new CANSparkMax(PortConstants.shooterTurretPort, MotorType.kBrushless);
  private RelativeEncoder turretEncoder = turretMotor.getEncoder();

  /** Creates a new Turret. */
  public Turret() {
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, 11.35f);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -11.35f);
    turretMotor.burnFlash();
  }

  public void runTurret(double speed) {
    turretMotor.set(speed);
  }

  public void stop() {
    turretMotor.set(0);
  }

  public void resetEncoder() {
    turretEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    turretPosition.set(turretEncoder.getPosition());
  }
}
