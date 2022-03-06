// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private ReportingNumber hoodAngleReporter = new ReportingNumber("Hood Set Angle", ReportingLevel.COMPETITON);

  private Servo hoodServo = new Servo(3);

  /** Creates a new Hood. */
  public Hood() {
    hoodServo.setBounds(2, 1.5, 1.5, 1.5, 1);
  }

  public void setPosition(double position) {

  }

  public void setAngle(double degrees) {
    degrees = MathUtil.clamp(degrees, 0, 360);
    hoodServo.setAngle(degrees);
  }

  @Override
  public void periodic() {
    hoodAngleReporter.set(hoodServo.getAngle());
  }
}
