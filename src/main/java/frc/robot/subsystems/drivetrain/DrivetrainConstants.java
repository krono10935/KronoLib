package frc.robot.subsystems.drivetrain;
import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleConstants;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

public class DrivetrainConstants {
    public enum HolonomicType{
        SWERVE,
        MECANUM
    }

    public static final HolonomicType HOLONOMIC_TYPE = HolonomicType.SWERVE;

    public static final Pose2d startPose2d = new Pose2d(2,7, new Rotation2d());

    public static final double MAX_LINEAR_SPEED = 4; // TODO find max linear speed (m/s)
    public static final double MIN_LINEAR_SPEED = 0.5; // m/s
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / SwerveModuleConstants.FRONT_LEFT.TRANSLATION.getNorm(); // rad/s
    public static final double MIN_ANGULAR_SPEED = MIN_LINEAR_SPEED / SwerveModuleConstants.FRONT_LEFT.TRANSLATION.getNorm(); // rad/s
    
    public static final PIDGains LINEAR_PID_GAINS = new PIDGains();
    public static final PIDGains ANGULAR_PID_GAINS = new PIDGains();

    // DriveToPose constants
    static {
        DriveToPoseConstants.ANGULAR_PID_GAINS = new ProfiledPIDController(0,0,0,null);
        DriveToPoseConstants.DISTANCE_TO_STOP_PP = 0.751; // meters
    }

    public static boolean shouldFlipPath(){
        var currentAlliance = DriverStation.getAlliance();

        // if (currentAlliance.isPresent()) {
        //     return currentAlliance.get() == DriverStation.Alliance.Red;
        // }
        // If no alliance is set, do not flip the path.
        return false;
    }

}
