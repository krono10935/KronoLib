package frc.robot.leds;

/**
 * Location of the leds including Line id, start led, end led
 */
public enum LedLocation{
    BASE(0,0,1),
    ARM(1,1,2),
    END_EFFECTOR(2,2,3);

    /**
     * Ledline to control
     */
    public final int ledLineID;

    /**
     * Starting led for the pattern on the ledline
     */
    public final int start;

    /**
     * Last led in the pattern for the ledline
     */
    public final int end;

    /**
     * Construct a Ledlocation enum.
     * @param ledLineID Ledline id to access
     * @param start Start position on the ledline
     * @param end End position on the ledline
     */
    LedLocation(int ledLineID,int start, int end){
        this.ledLineID = ledLineID;
        this.start = start;
        this.end = end;
    }
}
