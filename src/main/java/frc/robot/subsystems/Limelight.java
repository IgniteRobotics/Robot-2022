// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.ldap.LdapContext;

import com.igniterobotics.robotbase.reporting.ReportingBoolean;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTableInstance nTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable netTable = nTableInstance.getTable("limelight");
  private NetworkTableEntry ty = netTable.getEntry("ty");
  private NetworkTableEntry tx = netTable.getEntry("tx");
  private NetworkTableEntry tv = netTable.getEntry("tv");
  private NetworkTableEntry camMode = netTable.getEntry("camMode");
  private NetworkTableEntry ledMode = netTable.getEntry("ledMode");

  private ReportingNumber tyReporter = new ReportingNumber("Limelight ty", ReportingLevel.TEST);
  private ReportingNumber txReporter = new ReportingNumber("Limelight tx", ReportingLevel.TEST);
  private ReportingBoolean tvReporter = new ReportingBoolean("Limelight tv", ReportingLevel.TEST);

  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    tyReporter.set(getTy());
    txReporter.set(getTx());
    tvReporter.set(getTv());
  }

  public double getTy() {
    return ty.getDouble(-1);
  }

  public double getTx() {
    return tx.getDouble(-1);
  }

  public boolean getTv() {
    return tv.getDouble(-1) > 0;
  }

  public void setCamMode(boolean isCamMode) {
    camMode.setNumber(isCamMode ? 1 : 0);
  }

  public void setLed(boolean on) {
    ledMode.setNumber(on ? 3 : 1);
  }
}
