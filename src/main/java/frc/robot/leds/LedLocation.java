package frc.robot.leds;

/**
 * Location of the leds including Line id, start led, end led
 */
public enum LedLocation{
    BASE(0,0,1),
    ARM(1,1,2),
    END_EFFECTOR(2,2,3);

    public final int ledLineID;
    public final int start;
    public final int end;

    /**
     * Construct a Ledlocation enum.
     * @param ledLineID
     * @param start
     * @param end
     */
    LedLocation(int ledLineID,int start, int end){
        this.ledLineID = ledLineID;
        this.start = start;
        this.end = end;
    }
}
