package frc.robot.subsystems.lib.chasisConfigs;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.lib.moduleConfig.ModuleConstantsGeneric;
import frc.robot.subsystems.lib.moduleConfig.SwerveModuleConstantsRecord;

/**
 * Chassis-level container for swerve drivetrain configuration.
 * <p>
 * This class gathers together:
 * <ul>
 *   <li>Gyro I/O implementation used for robot heading</li>
 *   <li>Per-module constants (IDs, offsets, translations, motor configs)</li>
 *   <li>Chassis speed limits and tuning values</li>
 *   <li>A PathPlanner {@link RobotConfig} derived from the above</li>
 * </ul>
 * The {@link RobotConfig} is constructed from the module translations and generic
 * module configuration so PathPlanner can perform proper kinematics and trajectory
 * constraints for the robot.
 * <p>
 * Example usage:
 * <pre>
 *   ChasisConstants constants = new ChasisConstants(gyro, modules, speedCfg, generic, ppCfg);
 * </pre>
 */
public class ChasisConstants {

    public static double LOOP_TIME_SECONDS = 0.02;

    private GyroIO gyro;

    private final SwerveModuleConstantsRecord[] modules;


    private final ChasisSpeedConfig speedConfig;

    private final RobotConfig robotConfig;

    private final PPChasisConfig ppConfig;

    private final double MAX_ANGULAR_SPEED; //rad/s

    private final double MIN_ANGULAR_SPEED; //rad/s

    /**
     * Create chassis constants and derive the PathPlanner {@link RobotConfig} from the
     * provided module and chassis parameters.
     *
     * @param gyro         gyro implementation used to read robot heading
     * @param modules      array of swerve module constants
     * @param speedConfig  chassis-level speed limits and tuning parameters
     * @param generic      generic module configuration for all the modules
     * @param ppConfig     chassis parameters for PathPlanner (mass, MOI, etc.)
     */
    public ChasisConstants(GyroIO gyro, SwerveModuleConstantsRecord[] modules,
                           ChasisSpeedConfig speedConfig, ModuleConstantsGeneric generic, PPChasisConfig ppConfig) {
        this.gyro = gyro;
        this.modules = modules;
        this.speedConfig = speedConfig;
        this.ppConfig = ppConfig;

        Translation2d[] moduleTranslations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleTranslations[i] = modules[i].translation();
        }

       robotConfig = new RobotConfig(
                ppConfig.massKG(),
                ppConfig.MOI(),
                generic.moduleConfig(),
                moduleTranslations
        );

        MAX_ANGULAR_SPEED = speedConfig.maxLinearSpeed()/modules[0].translation().getNorm();
        MIN_ANGULAR_SPEED = speedConfig.minLinearSpeed()/modules[0].translation().getNorm();
    }

    public GyroIO getGyro() {
        return gyro; // returns the gyro I/O used by the drivetrain
    }

    public SwerveModuleConstantsRecord[] getModules() {
        return modules; // returns an array of per-module constants
    }

    public ChasisSpeedConfig getSpeedConfig() {
        return speedConfig; // returns chassis speed/tuning configuration
    }

    public RobotConfig getRobotConfig() {
        return robotConfig; // PathPlanner RobotConfig derived in the constructor
    }

    public PPChasisConfig getPpConfig() {
        return ppConfig; // returns PathPlanner-specific chassis parameters
    }

    public void setGyroSpeedsSupplier(){

    }

    /**
     * Determine whether autonomous paths should be mirrored for the current alliance.
     * <p>
     * PathPlanner expects paths to be authored from the Blue alliance perspective. This
     * helper returns true for Red so the path can be flipped at runtime. If the alliance
     * is unknown (e.g., in simulation or before FMS connection), this method returns false
     * and paths are not flipped.
     *
     * @return true when the robot is on Red alliance; false otherwise
     */
    public static boolean shouldFlipPath(){
        var currentAlliance = DriverStation.getAlliance();

        //         If no alliance is set, do not flip the path.
        return currentAlliance.filter(alliance -> alliance == DriverStation.Alliance.Red).isPresent();

    }

    public double getMAX_ANGULAR_SPEED() {
        return MAX_ANGULAR_SPEED;
    }

    public double getMIN_ANGULAR_SPEED() {
        return MIN_ANGULAR_SPEED;
    }

    public void setGyro(GyroIO gyro){
        this.gyro=gyro;
    }
}
