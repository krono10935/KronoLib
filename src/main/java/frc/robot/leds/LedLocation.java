package frc.robot.leds;

public enum LedLocation{
    BASE(0,0,1),
    ARM(1,1,2),
    END_EFFECTOR(2,2,3);

    public final int ledLineID;
    public final int start;
    public final int end;

    LedLocation(int ledLineID,int start, int end){
        this.ledLineID = ledLineID;
        this.start = start;
        this.end = end;
    }
}
