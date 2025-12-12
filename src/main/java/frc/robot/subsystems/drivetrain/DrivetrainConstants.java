package frc.robot.subsystems.drivetrain;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.module.SwerveModuleConstants;


public class DrivetrainConstants {


    public static final Pose2d startPose2d = new Pose2d(2,7, new Rotation2d());

    public static final double MAX_LINEAR_SPEED = 4; // TODO find max linear speed (m/s)
    public static final double MIN_LINEAR_SPEED = 0.5; // m/s
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / SwerveModuleConstants.FRONT_LEFT.TRANSLATION.getNorm(); // rad/s
    public static final double MIN_ANGULAR_SPEED = MIN_LINEAR_SPEED / SwerveModuleConstants.FRONT_LEFT.TRANSLATION.getNorm(); // rad/s

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
            60,
            6.8,
            SwerveModuleConstants.getModuleConfig(),
            SwerveModuleConstants.getModuleTranslations()
    );


    public static final double LOOP_TIME_SECONDS = 0.02;

    public static final PIDConstants PID_CONSTANTS = new PIDConstants(0);

    public static final PIDConstants ANGULAR_PID_CONSTANTS = new PIDConstants(0);

    public static final int PIGEON_ID = 1;


    public static boolean shouldFlipPath(){
        var currentAlliance = DriverStation.getAlliance();

        //         If no alliance is set, do not flip the path.
        return currentAlliance.filter(alliance -> alliance == DriverStation.Alliance.Red).isPresent();

    }

}
