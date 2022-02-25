
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;
//Color sensor stuff. We borrowed from willtoth on Github


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;


public class Indexer extends SubsystemBase {

  ////////// Constants //////////
  private static final double INDEXER_BELT_FWD_SPEED = -0.5;
  private static final double INDEXER_BELT_REV_SPEED = 0.5;
  private static final double INDEXER_KICKUP_FWD_SPEED = -0.5;
  private static final double INDEXER_KICKUP_REV_SPEED = 0.5;


  ////////// instance variables /////////
  private CANSparkMax indexerMotor;
  private CANSparkMax kickupMotor;

  private DigitalInput initialIndexerBeamBreak;
  private DigitalInput kickupIndexerBeamBreak;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public Indexer() {

    indexerMotor = new CANSparkMax(PortConstants.indexerMotorPort, MotorType.kBrushless);
    kickupMotor = new CANSparkMax(PortConstants.indexerKickupMotorPort, MotorType.kBrushless);
    initialIndexerBeamBreak = new DigitalInput(PortConstants.initialIndexerBeamBreakPort);
    kickupIndexerBeamBreak = new DigitalInput(PortConstants.kickupIndexerBeamBreakPort);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    //TODO:  invert the motors instead of negative power!
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

  // Gets the value of the digital input.  Normally returns true if the circuit is open, but we negate it.
  public boolean getInitialIndexerBeamBreak() {
    return !initialIndexerBeamBreak.get();
  }

  public boolean getKickupIndexerBeamBreak() {
    return !kickupIndexerBeamBreak.get();
  }

  public void colorsensorstuff()
  {
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    }  else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Initial Indexer Sensor", this.getInitialIndexerBeamBreak());
    SmartDashboard.putBoolean("Kickup Indexer Sensor", this.getKickupIndexerBeamBreak());
  }
}
