package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleConstants;

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
        public static final TrapezoidProfile.Constraints AUTO_CONSTRAINTS =
                new TrapezoidProfile.Constraints(0, 0);
        public static final TrapezoidProfile.Constraints TELE_CONSTRAINTS =
                new TrapezoidProfile.Constraints(0, 0);
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
