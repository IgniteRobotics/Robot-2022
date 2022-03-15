// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class Turret extends SubsystemBase {
  private ReportingNumber turretPosition = new ReportingNumber("Turret Position", ReportingLevel.COMPETITON);

  private CANSparkMax turretMotor = new CANSparkMax(PortConstants.shooterTurretPort, MotorType.kBrushless);
  private RelativeEncoder turretEncoder = turretMotor.getEncoder();
  private SparkMaxPIDController turretPidController = turretMotor.getPIDController();

  public static final int CURRENT_LIMIT = 20;
  public static double DEFAULT_KP = 0.1;
  public static double DEFAULT_KI = 0.0;
  public static double DEFAULT_KD = 0;
  public static double DEFAULT_RPM = 100;
  public static float FORWARD_LIMIT = 19f;
  public static float REVERSE_LIMIT = -19f;

  public static double POSITION_ACCURACY = 0.1;

  public double currentTargetTicks = 0;


  /** Creates a new Turret. */
  public Turret() {
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, FORWARD_LIMIT);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, REVERSE_LIMIT);
    turretMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    turretMotor.setIdleMode(IdleMode.kBrake);

    turretPidController.setP(DEFAULT_KP);
    turretPidController.setI(DEFAULT_KI);
    turretPidController.setD(DEFAULT_KD);
    turretPidController.setIZone(0);
    turretPidController.setOutputRange(-0.5, 0.5);

    turretMotor.burnFlash();
  }

  public void runTurret(double speed) {
    turretMotor.set(speed);
  }

  public void gotoPostion(double positionTicks){
    this.currentTargetTicks = positionTicks;
    turretPidController.setReference(positionTicks, CANSparkMax.ControlType.kPosition); 
  }

  public double getPositionTicks() {
    return turretEncoder.getPosition();
  }

  public boolean isAtPosition(){
    return Math.abs(getPositionTicks() - currentTargetTicks) < POSITION_ACCURACY;
  }

  public void stop() {
    turretMotor.stopMotor();
  }

  public void resetEncoder() {
    turretEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    turretPosition.set(turretEncoder.getPosition());
  }
}
