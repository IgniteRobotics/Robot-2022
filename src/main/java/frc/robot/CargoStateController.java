package frc.robot;

public class CargoStateController {
    private boolean firstPositionBreak = false;
    private boolean secondPositionBreak = false;

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

    public static CargoStateController getInstance() {
        return _instance;
    } 
}