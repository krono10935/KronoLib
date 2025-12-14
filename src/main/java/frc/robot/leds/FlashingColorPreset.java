package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;

/**
 * preset for solid color flashing in constant intervals
 */
public class FlashingColorPreset implements LedPreset{

    private Color color;
    private  double PHASE_TIME_SECONDS;
    private final int AMOUNT_OF_LEDS;

    /**
     *
     * @param t time elapsed in robot
     * @return the state for the strip, either single chosen color or turned off
     */
    @Override
    public ArrayList<Color> apply(double t) {
        ArrayList<Color> colors = new ArrayList<>();


        int phase = (int) (t / PHASE_TIME_SECONDS);

        for (int i = 0; i < AMOUNT_OF_LEDS; i++) {
            colors.add((phase % 2 == 0) ? Color.kBlack : color);
        }

        return colors;
    }

    /**
     *
     * @param color color to apply
     * @param phaseTimeSeconds the length of phases, turns on/off for that time in a constant cycle
     * @param amountOfLeds Amount of leds in the strip
     */
    public FlashingColorPreset(Color color, double phaseTimeSeconds, int amountOfLeds) {
        this.color = color;
        this.PHASE_TIME_SECONDS = phaseTimeSeconds;
        this.AMOUNT_OF_LEDS = amountOfLeds;
    }
}
