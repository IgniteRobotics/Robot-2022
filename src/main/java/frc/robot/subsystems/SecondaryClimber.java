// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class SecondaryClimber extends SubsystemBase {
  private DoubleSolenoid solenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, PortConstants.secondaryClimberForward, PortConstants.secondaryClimberReverse);

  /** Creates a new SecondaryClimber. */
  public SecondaryClimber() {}

  public void forward() {
    solenoid.set(Value.kForward);
  }

  public void reverse() {
    solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {

  }
}
