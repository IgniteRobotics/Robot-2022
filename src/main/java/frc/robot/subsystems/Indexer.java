
package frc.robot.subsystems;

import com.igniterobotics.robotbase.preferences.DoublePreference;
import com.igniterobotics.robotbase.reporting.ReportingBoolean;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;
import com.igniterobotics.robotbase.reporting.ReportingString;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateController;
import frc.robot.constants.PortConstants;
//Color sensor stuff. We borrowed from willtoth on Github

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

public class Indexer extends SubsystemBase {
  private final DoublePreference indexerBeltSpeed = new DoublePreference("Indexer/Belt Speed");
  private final DoublePreference indexerKickupSpeed = new DoublePreference("Indexer/Kickup Speed");

  private CANSparkMax indexerMotor;
  private CANSparkMax kickupMotor;

  private DigitalInput initialIndexerBeamBreak;
  private DigitalInput kickupIndexerBeamBreak;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private BallColor lastColor;
  private RobotStateController stateController;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a
   * parameter. The device will be automatically initialized with default
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final ReportingBoolean initalBeamBreakReporting = new ReportingBoolean("Indexer/Initial Beam (2nd pos)",
      ReportingLevel.COMPETITON);
  private final ReportingBoolean kickupBeamBreakReporting = new ReportingBoolean("Indexer/Kickup Beam (1st pos)",
      ReportingLevel.COMPETITON);
  private final ReportingNumber proximityReporting = new ReportingNumber("Indexer/Color Proximity",
      ReportingLevel.TEST);
  private final ReportingString ballColorReporting = new ReportingString("Indexer/Ball Color",
      ReportingLevel.COMPETITON);
  private final ReportingString position1ColorReporting = new ReportingString("Indexer/Pos 1 Color",
      ReportingLevel.COMPETITON);
  private final ReportingString position2ColorReporting = new ReportingString("Indexer/Pos 2 Color",
      ReportingLevel.COMPETITON);

  public enum BallColor {
    BLUE, RED, UNKNOWN
  }

  public Indexer() {

    stateController = RobotStateController.getInstance();

    indexerMotor = new CANSparkMax(PortConstants.indexerMotorPort, MotorType.kBrushless);
    kickupMotor = new CANSparkMax(PortConstants.indexerKickupMotorPort, MotorType.kBrushless);
    initialIndexerBeamBreak = new DigitalInput(PortConstants.initialIndexerBeamBreakPort);
    kickupIndexerBeamBreak = new DigitalInput(PortConstants.kickupIndexerBeamBreakPort);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    // TODO: invert the motors instead of negative power!
    indexerMotor.setInverted(false);
    indexerMotor.burnFlash();

    kickupMotor.setInverted(false);
    kickupMotor.burnFlash();
    position1ColorReporting.set(stateController.getFirstPositionColor().toString());
    position2ColorReporting.set(stateController.getSecondPositionColor().toString());
  }

  public void indexBall() {
    
    if (!getKickupIndexerBeamBreak()) {
      advanceKickUp();
      advanceBelt();
    } else {
      stopKickup();
      if (!getInitialIndexerBeamBreak()) {
        advanceBelt();
      } else {
        stopBelt();
      }
    }

    // if (!getInitialIndexerBeamBreak() && !getKickupIndexerBeamBreak()) {
    //   advanceBelt();
    // } else {
    //   stopBelt();
    // }
  }

  public void advanceBelt() {
    indexerMotor.set(-Math.abs(indexerBeltSpeed.getValue()));
  }

  public void retreatBelt() {
    indexerMotor.set(Math.abs(indexerBeltSpeed.getValue()));
  }

  public void stopBelt() {
    indexerMotor.set(0.0);
  }

  public void advanceKickUp() {
    kickupMotor.set(-Math.abs(indexerKickupSpeed.getValue()));
  }

  public void retreatKickup() {
    kickupMotor.set(Math.abs(indexerKickupSpeed.getValue()));
  }

  public void stopKickup() {
    kickupMotor.set(0.0);
  }

  public void stopAll() {
    this.stopBelt();
    this.stopKickup();
  }

  // Gets the value of the digital input. Normally returns true if the circuit is
  // open, but we negate it.
  public boolean getInitialIndexerBeamBreak() {
    return !initialIndexerBeamBreak.get();
  }

  public boolean getKickupIndexerBeamBreak() {
    return !kickupIndexerBeamBreak.get();
  }

  public BallColor getDetectedColor() {

    BallColor ballColor = BallColor.UNKNOWN;

    if (m_colorSensor.getProximity() >= 300) {
      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      if (match.color == kBlueTarget) {
        ballColor = BallColor.BLUE;
      } else if (match.color == kRedTarget) {
        ballColor = BallColor.RED;
      }
    }

    return ballColor;
  }

  public void updateColorInfo() {
    BallColor color = getDetectedColor();
    ballColorReporting.set(color.toString());

    proximityReporting.set((double) m_colorSensor.getProximity());

    if(color != BallColor.UNKNOWN && color != lastColor) {
      //stateController.addBall(getDetectedColor());
      stateController.addCargo(color);
    }

    lastColor = color;
    position1ColorReporting.set(stateController.getFirstPositionColor().toString());
    position2ColorReporting.set(stateController.getSecondPositionColor().toString());
  }

  @Override
  public void periodic() {
    RobotStateController stateController = RobotStateController.getInstance();
    stateController.setFirstPositionBreak(getKickupIndexerBeamBreak());
    stateController.setSecondPositionBreak(getInitialIndexerBeamBreak());

    initalBeamBreakReporting.set(getInitialIndexerBeamBreak());
    kickupBeamBreakReporting.set(getKickupIndexerBeamBreak());
    
  }
}
