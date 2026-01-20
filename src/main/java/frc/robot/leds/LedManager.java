package frc.robot.leds;

import java.util.ArrayList;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.util.Color;


/**
 * Manages the robot LED state by publishing it to NetworkTables.
 * <p>
 */
public class LedManager {
    private final StructArrayPublisher<LedState> publisher;

    private ArrayList<LedState> list = new ArrayList<>();

    public LedManager() {
        var nt = NetworkTableInstance.getDefault();

        publisher = nt.getTable("Led").getStructArrayTopic("states", LedState.struct).publish();
    
    }

    /**
     * publishes the colors to networkTable
     * including the location of the leds to set, the pattern chosen, the colors to use for the pattern
     * and updates the entry to say that a new command for the leds has been chosen.
     * @param state the led state to activate for the robot
     */
    public void setColors(LedState state){
        list.add(state);
    }

    public void periodic(){
        if(list.isEmpty()) return;
        
        LedState[] arr = list.toArray(new LedState[0]);

        list = new ArrayList<>();

        publisher.set(arr);
    }

}
