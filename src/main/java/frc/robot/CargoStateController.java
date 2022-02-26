package frc.robot;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.BallColor;

public class CargoStateController {
    private boolean firstPositionBreak = false;
    private boolean secondPositionBreak = false;

    private Indexer.BallColor[] ballColors = {BallColor.UNKNOWN, BallColor.UNKNOWN};

    private static CargoStateController _instance = new CargoStateController();

    private CargoStateController() {

    }

    public boolean runFirstPosition() {
        return !firstPositionBreak;
    }

    public boolean runSecondPosition() {
        return !firstPositionBreak || !secondPositionBreak; 
    }

    public void setFirstPositionBreak(boolean broken) {
        this.firstPositionBreak = broken;
    }

    public void setSecondPositionBreak(boolean broken) {
        this.secondPositionBreak = broken;
    }

    public void addBall(BallColor ballColor) {
        ballColors[0] = ballColors[1];
        ballColors[1] = ballColor;
    }

    public String getBallColorsString() {
        return "[" + ballColors[0] + ", " + ballColors[1] + "]";
    }

    public static CargoStateController getInstance() {
        return _instance;
    }
}