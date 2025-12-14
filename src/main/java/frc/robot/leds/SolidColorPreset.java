package frc.robot.leds;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;


/**
 * preset for single solid color
 */
public class SolidColorPreset implements LedPreset{
    private Color color;
    private final int AMOUNT_OF_LEDS;


    /**
     *
     * @param t time elapsed in robot
     * @return solid color across all the strip
     */
    @Override
    public ArrayList<Color> apply(double t) {
        ArrayList<Color> result = new ArrayList<>();
        for (int i = 0; i <AMOUNT_OF_LEDS ; i++) {
            result.add(color);
        }
        return result;

    }


    /**
     *
     * @param color single color to apply across the strip
     * @param amountOfLeds amount of leds in the strip
     */
    public SolidColorPreset(Color color, int amountOfLeds) {
        this.color = color;
        AMOUNT_OF_LEDS = amountOfLeds;
    }
}
