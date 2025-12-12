package frc.robot.leds;

/**
 * Represents the state of the leds of the robot
 */
public enum LedState {
    RED("red"),
    NONE("none");
    private final String NTKey;

    LedState(String NTKey){
        this.NTKey = NTKey;
    }

    public String getNTKey() {
        return NTKey;
    }
}
