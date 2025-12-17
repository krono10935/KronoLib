package frc.robot.leds;

public enum LedLocation{
    BASE(0,1),
    ARM(1,2),
    END_EFFECTOR(2,3);

    public final int start;
    public final int end;

    LedLocation(int start, int end){
        this.start = start;
        this.end = end;
    }
}
