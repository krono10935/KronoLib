package frc.robot.leds;

/**
 * Location of the leds including Line id, start led, end led
 */
public enum LedLocation{
    BASE(0,10),
    ARM(11,20),
    END_EFFECTOR(2,3),
    ALL(0, 20);

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
     * @param start Start position on the ledline
     * @param end End position on the ledline
     */
    LedLocation(int start, int end){
        this.start = start;
        this.end = end;
    }
}
