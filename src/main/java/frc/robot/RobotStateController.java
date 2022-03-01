package frc.robot;

import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingString;

import org.opencv.ml.EM;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.BallColor;

public class RobotStateController {
    public static final char EMPTY = 1 << 0;
    public static final char POS1 = 1 << 1;
    public static final char POS2 = 1 << 2;

    private boolean firstPositionBreak = false;
    private boolean secondPositionBreak = false;

    // private Indexer.BallColor[] ballColors = { BallColor.UNKNOWN, BallColor.UNKNOWN };

    private final ReportingString stateReporting = new ReportingString("State Info", ReportingLevel.TEST);


    // HeldCargo, a FIFO queue implemented in an array.  
    //    Index = 0 is "First Position", first ball to shoot
    //    Index = 1 is "Second Position", the second ball to shoot
    // 
    //  If only one cargo is in the indexer, it will be held in First Position (idx = 0)
    //  Since second position is full only after first position is full, if second position
    //  is empty we can intake more cargo.  IF second position is full, we can't intake more
    //  cargo.

    private Indexer.BallColor[] heldCargo = { BallColor.UNKNOWN, BallColor.UNKNOWN };

    // Color of alliance to know if the cargo in First Position can be shoot when we try to score.
    private Indexer.BallColor allianceColor = BallColor.BLUE;

    private static RobotStateController _instance = new RobotStateController();

    private RobotStateController() {

    }

    public void Reset() {
        heldCargo[0] = BallColor.UNKNOWN;
        heldCargo[1] = BallColor.UNKNOWN;
    }

    public boolean IsIndexerFull() {
        return firstPositionBreak && secondPositionBreak;
    }

    public boolean isEmpty() {
        return heldCargo[1] == BallColor.UNKNOWN;
    }

    //When indexer gets a new cargo, in first position if possible, otherwise add to second position
    public void AddCargo(BallColor ballColor)
    {
        if (heldCargo[0] == BallColor.UNKNOWN) {
            stateReporting.set("Setting 0");
            heldCargo[0] = ballColor;
        }
        else {
            stateReporting.set("Setting 1");
            heldCargo[1] = ballColor;
        }
    }

    //When shooting, advancing cargo will remove first positition and move the second potion into the first position
    public void AdvanceCargo() {
        heldCargo[0] = heldCargo[1];
        heldCargo[1] = BallColor.UNKNOWN;
        
    }
    
    public BallColor FirstPositionColor() {
        return heldCargo[0];
    }

    public BallColor SecondPositionColor() {
        return heldCargo[1];
    }

    // If Shooter 'Ready to shoot' when the shooter is "On Target" and "At Velocity".
    // Indexer won't advance cargo in to the shooter until it's ready to shoot.
    public boolean isReadyToShoot;






    // public boolean runFirstPosition() {
    //     return !firstPositionBreak;
    // }

    // public boolean runSecondPosition() {
    //     return !firstPositionBreak || !secondPositionBreak;
    // }

    public void setFirstPositionBreak(boolean broken) {
        this.firstPositionBreak = broken;
    }

    public void setSecondPositionBreak(boolean broken) {
        this.secondPositionBreak = broken;
    }

    public boolean isBreaksClear() {
        return !firstPositionBreak && !secondPositionBreak;
    }

    // public void addBall(BallColor ballColor) {
    //     if (ballColors[1] != BallColor.UNKNOWN && ballColors[0] != BallColor.UNKNOWN) {
    //         ballColors[0] = ballColors[1];
    //     }
    //     ballColors[1] = ballColor;
    // }

    // public String getBallColorsString() {
    //     return "[" + ballColors[0] + ", " + ballColors[1] + "]";
    // }

    // public BallColor[] getBallColors() {
    //     return ballColors;
    // }

    public static RobotStateController getInstance() {
        return _instance;
    }
}