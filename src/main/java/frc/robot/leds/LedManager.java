package frc.robot.leds;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;

/**
 * Manages the robot LED state by publishing it to NetworkTables.
 * <p>
 * {@link NetworkTableInstance} and publishes to the topic {@code Leds/LedState}.
 */
public class LedManager {

    private final NetworkTableInstance nt;
    private LedState state;
    private final StringEntry ledStateEntry;

    /**
     * Creates a new {@code LedManager} that publishes to {@code Leds/LedState}
     * using the default {@link NetworkTableInstance}.
     */
    public LedManager() {
        nt = NetworkTableInstance.getDefault();
        StringTopic topic = nt.getStringTopic("Leds/LedState");
        state = LedState.NONE;
        ledStateEntry = topic.getEntry(LedState.NONE.getNTKey());
        publishStateToNT();
    }

    /**
     * Publishes the current {@link LedState} to NetworkTables.
     */
    private void publishStateToNT() {
        ledStateEntry.set(state.getNTKey());
    }

    /**
     * Updates the LED state and publishes the change to NetworkTables.
     *
     * @param state the new LED state to apply
     */
    public void setState(LedState state) {
        this.state = state;
        publishStateToNT();
    }
}
