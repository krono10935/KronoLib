package frc.robot.subsystems.drivetrain;
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
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / SwerveModuleConstants.FRONT_LEFT.TRANSLATION.getNorm(); // degrees/s
    public static final double MIN_ANGULAR_SPEED = MIN_LINEAR_SPEED / SwerveModuleConstants.FRONT_LEFT.TRANSLATION.getNorm(); // degrees/s

    public static class DriveToPose{
        /**
         * the linear Trapezoid Profile constraints for autonomous
         */
        public static final TrapezoidProfile.Constraints LINEAR_AUTO_CONSTRAINTS =
                new TrapezoidProfile.Constraints(0, 0);
        /**
         * the linear Trapezoid Profile constraints for tele-op
         */
        public static final TrapezoidProfile.Constraints LINEAR_TELE_CONSTRAINTS =
                new TrapezoidProfile.Constraints(0, 0);
        /**
         * the angle Trapezoid Profile constraints for autonomous
         */
        public static final TrapezoidProfile.Constraints ANGLE_AUTO_CONSTRAINTS =
                new TrapezoidProfile.Constraints(Units.degreesToRadians(0), Units.degreesToRadians(0));
        /**
         * the angle Trapezoid Profile constraints for tele-op
         */
        public static final TrapezoidProfile.Constraints ANGLE_TELE_CONSTRAINTS =
                new TrapezoidProfile.Constraints(Units.degreesToRadians(0), Units.degreesToRadians(0));
        /**
         * linear PID constants
         */
        public static final PIDGains LINEAR_PID_GAINS = new PIDGains();
        /**
         * angular PID constants
         */
        public static final PIDGains ANGULAR_PID_GAINS = new PIDGains();
        /**
         * the minimum distance from the current pose to the goal where we will not scale the velocity
         */
        public static final double MIN_DISTANCE_VELOCITY_CORRECTION = 0.02;
        /**
         * the minimum velocity the feed forward can get
         */
        public static final double MIN_SET_POINT_VELOCITY = 0.5;
        /**
         * the distance which above the FF is at 100% and below is 100%-0%.
         */
        public static final double FF_MAX_DISTANCE = 0.03;
        /**
         * the distance which above the FF is at 0%-100% and below is 0%.
         */
        public static final double FF_MIN_DISTANCE = 0.01;
        /**
         * the angle which above the FF is at 100% and below is 100%-0%.
         */
        public static final double FF_MAX_ANGLE = Units.degreesToRadians(15);
        /**
         * the angle which above the FF is at 0%-100% and below is 0%.
         */
        public static final double FF_MIN_ANGLE = Units.degreesToRadians(5);
        /**
         * the distance within the robot is considered at the goal
         */
        public static final double poseTolerance = 5;
        /**
         * the angle within the robot is considered at the goal
         */
        public static final double angleTolerance = 10;
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
