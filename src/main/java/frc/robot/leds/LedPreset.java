package frc.robot.leds;


import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;

/**
 * generic interface for led preset  types
 */
public interface LedPreset{

    /**
     *
     * @param t time elapsed in robot
     * @return colors to apply to led strip
     */
    ArrayList<Color> apply(double t);
}
