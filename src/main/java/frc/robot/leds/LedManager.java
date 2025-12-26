package frc.robot.leds;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.function.Supplier;

/**
 * Manages the robot LED state by publishing it to NetworkTables.
 * <p>
 */
public class LedManager {

    /**
     * Represents a given ledstate which the leds should have (position, colors, frequency, and pattern)
     * @param pattern
     * @param mainColor
     * @param secondaryColor
     * @param hz
     * @param ledLineID
     * @param start
     * @param end
     */
    public record LedState(String pattern, Color mainColor, Color secondaryColor, double hz, int ledLineID, int start, int end){
        public LedState(LedPattern pattern, Color mainColor, Color secondaryColor, double hz, LedLocation location){
            this(pattern.toString(), mainColor, secondaryColor, hz, location.ledLineID,  location.start, location.end);
        }
    }
    private final NetworkTableEntry ledLineIDEntry;
    private final NetworkTableEntry patternEntry;

    private final NetworkTableEntry mainColorEntry;

    private final NetworkTableEntry secondaryColorEntry;

    private final NetworkTableEntry hzEntry;

    private final NetworkTableEntry rangeEntry;

    private final NetworkTableEntry hasChangeEntry;

    public LedManager() {
        var nt = NetworkTableInstance.getDefault();

        var table = nt.getTable("Led");


        ledLineIDEntry = table.getEntry("id");

        patternEntry = table.getEntry("pattern");

        mainColorEntry = table.getEntry("mainColor");

        secondaryColorEntry = table.getEntry("secondaryColor");

        hzEntry = table.getEntry("hz");

        rangeEntry = table.getEntry("range");

        hasChangeEntry = table.getEntry("hasChange");

    }

    /**
     * publishes the colors to networkTable
     * @param state the led state to activate for the robot
     */
    public void setColors(LedState state){
        ledLineIDEntry.setInteger(state.ledLineID);
        patternEntry.setString(state.pattern);
        mainColorEntry.setDoubleArray(convertColorToDoubleArr(state.mainColor));
        secondaryColorEntry.setDoubleArray(convertColorToDoubleArr(state.secondaryColor));
        hzEntry.setDouble(state.hz);
        rangeEntry.setDoubleArray(new double[]{state.start, state.end});
        hasChangeEntry.setBoolean(true);
    }


    /**
     * util to convert a color to an array to publish to network tables
     * @param color chosen color
     * @return the color in {r,g,b}
     */
    private static double[] convertColorToDoubleArr(Color color){
        return new double[]{color.red,color.green,color.blue};
    }









}
