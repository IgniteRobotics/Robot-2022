// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private ReportingNumber hoodAngleReporter = new ReportingNumber("Hood Set Angle", ReportingLevel.COMPETITON);

  private LinearServo hoodServo = new LinearServo(3, 50, 32);
  private double targetPosition = 0;

  /** Creates a new Hood. */
  public Hood() {
  
  }

  public void setPosition(double position) {
    hoodServo.setPosition(position);
    targetPosition = position;
 }

  @Override
  public void periodic() {
    hoodAngleReporter.set(getPosition());
    hoodServo.updateCurPos();
  }

  public double getPosition() {
    return hoodServo.getPosition();
  }

  public boolean isInPosition(){
    return hoodServo.getPosition() == targetPosition;
  }

  class LinearServo extends Servo
  {
      double m_speed;
      double m_length;
      double setPos;
      double curPos;

      /**
       * Parameters for L16-R Actuonix Linear Actuators
       *
       * @param channel PWM channel used to control the servo
       * @param length  max length of the servo [mm]
       * @param speed   max speed of the servo [mm/second]
       */
      public LinearServo(int channel, int length, int speed)
      {
          super(channel);
          setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
          m_length = length;
          m_speed = speed;
          
      }

      /**
       * Run this method in any periodic function to update the position estimation of
       * your servo
       *
       * @param setpoint the target position of the servo [mm]
       */
      public void setPosition(double setpoint)
      {
          setPos = MathUtil.clamp(setpoint, 0, m_length);
          setSpeed((setPos / m_length * 2) - 1);
      }

      double lastTime = 0;

      /**
       * Run this method in any periodic function to update the position estimation of
       * your servo
       */
      public void updateCurPos()
      {
          double dt = Timer.getFPGATimestamp() - lastTime;
          if (curPos > setPos + m_speed * dt) {
              curPos -= m_speed * dt;
          } else if (curPos < setPos - m_speed * dt) {
              curPos += m_speed * dt;
          } else {
              curPos = setPos;
          }
      }

      /**
       * Current position of the servo, must be calling {@link #updateCurPos()
       * updateCurPos()} periodically
       *
       * @return Servo Position [mm]
       */
      public double getPosition()
      {
          return curPos;
      }
  }


}
