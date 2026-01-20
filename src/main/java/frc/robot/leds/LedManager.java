package frc.robot.leds;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


/**
 * Manages the robot LED state by publishing it to NetworkTables.
 * <p>
 */
public class LedManager extends SubsystemBase {
    /**
     * The publisher for the LED states
     */
    private final StructArrayPublisher<LedState> publisher;

    /**
     * The array holding the next states of the LED
     */
    private ArrayList<LedState> list = new ArrayList<>();

    /**
     * The status of the rsl light, used to sync the leds with the rsl.
     */
    private final NetworkTableEntry rslStatus;

    /**
     * Creates a new LED manager
     */
    public LedManager() {
        var ntTable = NetworkTableInstance.getDefault().getTable("Led");

        publisher = ntTable.getStructArrayTopic("states", LedState.struct).publish();

        rslStatus = ntTable.getEntry("RslStatus");
    }

    /**
     * publishes the colors to networkTable
     * including the location of the leds to set, the pattern chosen, the colors to use for the pattern
     * and updates the entry to say that a new command for the leds has been chosen.
     * @param state the LED state to activate for the robot
     */
    public void setColors(LedState state){
        list.add(state);
    }

    @Override
    public void periodic(){
        if(Robot.isReal()) rslStatus.setBoolean(RobotController.getRSLState());
        else Logger.runEveryN(25, () -> rslStatus.setBoolean(!rslStatus.getBoolean(false)));
        
        if(list.isEmpty()) return;
        
        LedState[] arr = list.toArray(new LedState[0]);

        list = new ArrayList<>();

        publisher.set(arr);
    }

}
