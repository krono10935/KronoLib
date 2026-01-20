package frc.robot.leds;

/**
 * Possible patterns for the leds, how they look visually is determined on the server side
 */
public enum LedPattern {
    RAINBOW,
    SOLID,
    BLINK;


    @Override
    public String toString(){
        return this.name().toLowerCase();
    }
}
