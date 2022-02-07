
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.PortConstants;

public class Indexer extends SubsystemBase {

  ////////// Constants //////////
  private static final double INDEXER_BELT_FWD_SPEED = 0.5;
  private static final double INDEXER_BELT_REV_SPEED = -0.5;
  private static final double INDEXER_KICKUP_FWD_SPEED = 0.5;
  private static final double INDEXER_KICKUP_REV_SPEED = -0.5;


  ////////// instance variables /////////
  private CANSparkMax indexerMotor;
  private CANSparkMax kickupMotor;

  private DigitalInput initialIndexerBeamBreak;
  private DigitalInput kickupIndexerBeamBreak;


  public Indexer() {

    indexerMotor = new CANSparkMax(PortConstants.indexerMotorPort, MotorType.kBrushless);
    kickupMotor = new CANSparkMax(PortConstants.indexerKickupMotorPort, MotorType.kBrushless);
    initialIndexerBeamBreak = new DigitalInput(PortConstants.initialIndexerBeamBreakPort);
    kickupIndexerBeamBreak = new DigitalInput(PortConstants.kickupIndexerBeamBreakPort);

    //TODO:  verify inversion
    indexerMotor.setInverted(false);
    indexerMotor.burnFlash();

    kickupMotor.setInverted(false);
    kickupMotor.burnFlash();
  }

  public void advanceBelt() {
    indexerMotor.set(INDEXER_BELT_FWD_SPEED);
  }

  public void retreatBelt() {
    indexerMotor.set(INDEXER_BELT_REV_SPEED);
  }

  public void stopBelt() {
    indexerMotor.set(0.0);
  }

  public void advanceKickUp() {
    kickupMotor.set(INDEXER_KICKUP_FWD_SPEED);
  }

  public void retreatKickup() {
    kickupMotor.set(INDEXER_KICKUP_REV_SPEED);
  }

  public void stopKickup() {
    kickupMotor.set(0.0);
  }

  public void stopAll() {
    this.stopBelt();
    this.stopKickup();
  }

  public boolean getInitialIndexerBeamBreak() {
    return !initialIndexerBeamBreak.get();
  }

  public boolean getKickupIndexerBeamBreak() {
    return !kickupIndexerBeamBreak.get();
  }


  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Initial Indexer Sensor", this.getInitialIndexerBeamBreak());
    SmartDashboard.putBoolean("Kickup Indexer Sensor", this.getKickupIndexerBeamBreak());
  }
}
