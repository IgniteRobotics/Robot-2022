// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LinearServo;

public class Hood extends SubsystemBase {
  private ReportingNumber hoodAngleReporter = new ReportingNumber("Hood Set Angle", ReportingLevel.COMPETITON);

  private LinearServo hoodServo = new LinearServo(3, 50, 32);

  private double targetPosition;

  /** Creates a new Hood. */
  public Hood() {

  }

  public void setPosition(double position) {
    this.targetPosition = position;
    hoodServo.set(position);
  }

  public void setAngle(double degrees) {
    setPosition((degrees / 180) * 50);
  }

  public boolean isInPosition(){
    return hoodServo.getPosition() >= targetPosition - 0.1 && hoodServo.getPosition() <= targetPosition + 0.1;
  }

  @Override
  public void periodic() {
    hoodServo.updateCurPos();
    hoodAngleReporter.set(hoodServo.getAngle());
  }
}
