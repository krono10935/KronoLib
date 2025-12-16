package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNav {

    // NetworkTables instance and table for Quest communication
    NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    NetworkTable nt4Table = nt4Instance.getTable("questnav");

    // Integer subscriber to receive messages from the Quest (MISO)
    private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);

    // Integer publisher to send messages to the Quest (MOSI)
    private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    // Timestamp of last Quest update
    private DoubleSubscriber questTimeStamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0);

    // Quest battery percentage
    private DoubleSubscriber questBattery = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0);

    // Quest position [x, y, z]
    private FloatArraySubscriber questPos = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});

    // Quest orientation quaternion [w, x, y, z]
    private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});

    // Euler angles [roll, yaw, pitch] from the Quest
    private FloatArraySubscriber questEulerAngle = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});

    // Offset for yaw to allow resetting/zeroing of heading
    private float yaw_ofsset = 0.0f;

    // Stores the reset pose for zeroing position
    private Pose2d resetPosition = new Pose2d();

    /**
     * Converts Quest position to field coordinates (Translation2d).
     * Swaps axes to match robot field coordinate convention.
     */
    private Translation2d getQuestNavTranslation() {
        float[] pos = questPos.get();
        
        if (pos == null || pos.length < 3) {
            throw new IllegalStateException("QuestNav position array is invalid");
        }
        
        double x = pos[2];  // Forward
        double y = -pos[0]; // Coordinate system adjustment
        
        return new Translation2d(x, y);
    }
    
    /**
     * Returns the Quest yaw (heading) in degrees, corrected by offset.
     * Result is always in [0, 360) degrees.
     */
    private float getOculusYaw() {
        float[] eulerAngles = questEulerAngle.get();
    
        if (eulerAngles == null || eulerAngles.length < 2) {
            throw new IllegalStateException("Invalid Euler angles array");
        }
    
        float yaw = eulerAngles[1] - yaw_ofsset;
        yaw = (yaw % 360 + 360) % 360;  // Wrap to 0–360
    
        return yaw;
    }
    
    // Offset to account for the Quest mounting height
    private static final double OCULUS_Y_OFFSET = 0.1651; // meters

    /**
     * Returns the current Pose2d of the Quest, adjusted for headset offset.
     */
    private Pose2d getQuestNavPose() {
        Translation2d oculusPosition = getQuestNavTranslation();
        Translation2d compensatedPosition = oculusPosition.minus(new Translation2d(0, OCULUS_Y_OFFSET));
        Rotation2d oculusRotation = Rotation2d.fromDegrees(getOculusYaw());

        return new Pose2d(compensatedPosition, oculusRotation);
    }

    /**
     * Returns the robot Pose2d relative to the zeroed position.
     */
    public Pose2d getPose2d() {
        Translation2d poseXYZ = getQuestNavPose().minus(resetPosition).getTranslation();
        Rotation2d poseRotation = Rotation2d.fromDegrees(getOculusYaw());
        return new Pose2d(poseXYZ, poseRotation);
    }

    /**
     * Returns the battery percentage of the Quest headset.
     */
    public double getBatteryPercent() {
        return questBattery.get();
    }

    /**
     * Checks if the Quest headset is connected.
     * Uses the timestamp of the last battery update to determine connectivity.
     */
    private static final double CONNECTION_TIMEOUT_MS = 250.0;

    public boolean connected() {
        double fpgaTimeUs = RobotController.getFPGATime();
        double lastBatteryUpdateUs = questBattery.getLastChange();

        double elapsedMs = (fpgaTimeUs - lastBatteryUpdateUs) / 1000.0;
        return elapsedMs < CONNECTION_TIMEOUT_MS;
    }

    /**
     * Returns the current quaternion of the Quest headset.
     */
    public Quaternion getQuaternion() {
        float[] qValues = questQuaternion.get();
    
        if (qValues == null || qValues.length < 4) {
            throw new IllegalStateException("Invalid quaternion array from Quest");
        }
    
        return new Quaternion(qValues[0], qValues[1], qValues[2], qValues[3]);
    }
    
    /**
     * Returns the timestamp of the last Quest update.
     */
    public double timestamp() {
        return questTimeStamp.get();
    }

    /**
     * Zeroes the robot heading by updating the yaw offset.
     */
    public void zeroHeading() {
        float[] eulerAngles = questEulerAngle.get();
    
        if (eulerAngles == null || eulerAngles.length < 2) {
            throw new IllegalStateException("Invalid Euler angles array from Quest");
        }
    
        yaw_ofsset = eulerAngles[1];
    }
    
    /**
     * Zeroes the robot's absolute 3D position.
     * Signals the Quest headset to reset internal tracking if necessary.
     */
    public void zeroPosition() {
        resetPosition = getPose2d();
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
    }

    /**
     * Cleans up QuestNav handshake messages.
     * Ensures MISO/MOSI handshake is cleared after processing.
     */
    public void cleanUpQuestNavMessages() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }
}


