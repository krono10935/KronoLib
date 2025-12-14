package frc.robot.leds;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;

/**
 * Manages the robot LED state by publishing it to NetworkTables.
 * <p>
 * {@link NetworkTableInstance} and publishes to the topic {@code Leds/LedState}.
 */
public class LedManager {

    //TODO: Set actual number of leds
    private final int AMOUNT_OF_LEDS = 2;
    private final NetworkTableInstance nt;

    private LedPreset preset;
    private final LedPreset NONE = new SolidColorPreset(Color.kBlack,1);
    private final ArrayList<DoubleArrayEntry> ledColorsEntries;
    private ArrayList<Color> ledColors;







    public LedManager() {
        nt = NetworkTableInstance.getDefault();
        preset = NONE;
        ledColorsEntries = new ArrayList<>();
        for (int i = 0; i <AMOUNT_OF_LEDS ; i++) {
            ledColorsEntries.add(nt.getDoubleArrayTopic("Leds/led" + i).getEntry(new double[3]));
        }



    }

    /**
     * publishes the colors to networkTable
     */
    public void publishColors(){
        ledColors=preset.apply(Timer.getFPGATimestamp());
        for (int i = 0; i < AMOUNT_OF_LEDS; i++) {
            ledColorsEntries.get(i).set(
                    convertColorToDoubleArr(ledColors.get(i)));
        }
    }


    /**
     * util to convert a color to an array to publish to network tables
     * @param color chosen color
     * @return the color in {r,g,b}
     */
    private static double[] convertColorToDoubleArr(Color color){
        return new double[]{color.red,color.green,color.blue};
    }

    /**
     * set the preset for the leds
     * @param preset desired preset
     */
    public void setPreset(LedPreset preset){
        this.preset = preset;
    }







}
