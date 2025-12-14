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






    /**
     * Creates a new {@code LedManager} that publishes to {@code Leds/LedState}
     * using the default {@link NetworkTableInstance}.
     */
    public LedManager() {
        nt = NetworkTableInstance.getDefault();
        preset = NONE;
        ledColorsEntries = new ArrayList<>();
        for (int i = 0; i <AMOUNT_OF_LEDS ; i++) {
            ledColorsEntries.add(nt.getDoubleArrayTopic("Leds/led" + i).getEntry(new double[3]));
        }



    }
    public void publishColors(){
        ledColors=preset.apply(Timer.getFPGATimestamp());
        for (int i = 0; i < AMOUNT_OF_LEDS; i++) {
            ledColorsEntries.get(i).set(
                    convertColorToDoubleArr(ledColors.get(i)));
        }
    }

    private static double[] convertColorToDoubleArr(Color color){
        return new double[]{color.red,color.green,color.blue};
    }

    public void setPreset(LedPreset preset){
        this.preset = preset;
    }







}
