package frc.robot.commands.auto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
    /**
     * the goal state in the Trapezoid Profile
     */
    private final static TrapezoidProfile.State GOAL_STATE = new State(0,0);
    /**
     * the drivetrain of the robot
     */
    private final Drivetrain drivetrain;
    /**
     * the position of the goal
     */
    private final Supplier<Pose2d> goalPose;
    /**
     * the position of the robot
     */
    private final Supplier<Pose2d> robotPose;
    /**
     * the linear Trapezoid Profile
     */
    private TrapezoidProfile driveProfile;
    /**
     * the chassis speed
     */
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    /**
     * the position in the last run
     */
    private Translation2d lastSetPoint;
    /**
     * the velocity in the last run
     */
    private Vector<N2> lastSetpointVelocity;
    /**
     * the linear PID controller
     */
    private final PIDController drivePIDController;
    /**
     * the angular PID controller and profile
     */
    private final ProfiledPIDController angularPIDController;
    /**
     * absolute value of the angle error
     */
    private double absAngleError;
    /**
     * absolute value of the linear error
     */
    private double absPoseError;

    private Pose2d goalPose_;


    public DriveToPoseCommand(Drivetrain drivetrain, Supplier<Pose2d> goalPose) {
        robotPose = drivetrain::getEstimatedPosition;
        this.chassisSpeedSupplier = drivetrain::getChassisSpeeds;
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;

        var linerGains = DrivetrainConstants.DriveToPose.LINEAR_PID_GAINS;
        drivePIDController = new PIDController(
                linerGains.getK_P(),
                linerGains.getK_I(),
                linerGains.getK_D()
        );

        var angularGains = DrivetrainConstants.DriveToPose.ANGULAR_PID_GAINS;
        angularPIDController = new ProfiledPIDController(
                angularGains.getK_P(),
                angularGains.getK_I(),
                angularGains.getK_D(),
                DrivetrainConstants.DriveToPose.ANGLE_AUTO_CONSTRAINTS);

        angularPIDController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    /**
     * rests all the variables before starting
     */
    private void resetState(){
        driveProfile = new TrapezoidProfile(RobotState.isAutonomous() ?
                DrivetrainConstants.DriveToPose.LINEAR_AUTO_CONSTRAINTS :
                DrivetrainConstants.DriveToPose.LINEAR_TELE_CONSTRAINTS);

        drivePIDController.reset();

        angularPIDController.setConstraints(RobotState.isAutonomous() ?
                DrivetrainConstants.DriveToPose.ANGLE_AUTO_CONSTRAINTS:
                DrivetrainConstants.DriveToPose.ANGLE_TELE_CONSTRAINTS);

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeedSupplier.get(), robotPose.get().getRotation());

        angularPIDController.reset(new State(robotPose.get().getRotation().getRadians(), speeds.omegaRadiansPerSecond));

        lastSetPoint = robotPose.get().getTranslation();

        lastSetpointVelocity = VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        var error = robotPose.get().relativeTo(goalPose.get());

        absAngleError = Math.abs(error.getRotation().getRadians());

        absPoseError = error.getTranslation().getNorm();
    }
    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        resetState();
        Logger.recordOutput("driveToPose/goalPose", goalPose.get());
        goalPose_ = goalPose.get();
    }

    /**
     * calculates the next state in the profile.
     * @param goal final goal
     * @param pose current pose
     * @param error the distance between the current pose and the goal.
     * @return the next state of the profile
     */
    public TrapezoidProfile.State calcProfile(Pose2d goal, Pose2d pose, double error) {
        var profileDirection2d = goal.getTranslation().minus(lastSetPoint);
        var profileDirection = profileDirection2d.toVector();

        //if the distance from the current pose to the goal is less then a constant then we will not scale the velocity
        //vector according to the direction vector.
        double velocity = profileDirection.norm()
                <= DrivetrainConstants.DriveToPose.MIN_DISTANCE_VELOCITY_CORRECTION
                ? lastSetpointVelocity.norm()
                : lastSetpointVelocity.dot(profileDirection) / profileDirection.norm();

        velocity = Math.max(velocity, DrivetrainConstants.DriveToPose.MIN_SET_POINT_VELOCITY);

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(profileDirection.norm(), -velocity);

        var nextState = driveProfile.calculate(0.02, currentState, GOAL_STATE);

        double scalar = nextState.position / error;

        //let v be current pose vector
        //let u be goal pose vector
        //vectoric function:
        //w = v * t + u * (1 - t)
        //if you are reading this use Translation2D.interpolate instead
        lastSetPoint = pose.getTranslation().times(scalar).plus(goal.getTranslation().times(1 - scalar));
        return nextState;
    }

    /**
     * calc the FFScalar for the velocity
     * @param error the distance between the current pose and the goal.
     * @return how much should the velocity FF affect the output.
     */
    public double FFScalar(double error, double minError, double maxError) {
        return MathUtil.clamp((error - minError)/ (maxError - minError),
                0.0, 1.0);
    }

    @Override
    public void execute() {
        Pose2d pose = robotPose.get();
        Pose2d goal = goalPose_;
        var poseError = pose.relativeTo(goal);
        absPoseError = poseError.getTranslation().getNorm();
        absAngleError = Math.abs(poseError.getRotation().getRadians());
        var nextState = calcProfile(goal, pose, absPoseError);

        double targetVelocityPID = drivePIDController.calculate(absPoseError, nextState.position);

        double targetVelocityFF = nextState.velocity * FFScalar(absPoseError, DrivetrainConstants.DriveToPose.FF_MIN_DISTANCE,
                DrivetrainConstants.DriveToPose.FF_MAX_DISTANCE);

        var errorAngle = pose.getTranslation().minus(goal.getTranslation()).getAngle();

        lastSetpointVelocity = new Translation2d(targetVelocityFF, errorAngle).toVector();

        double targetVelocity = targetVelocityPID + targetVelocityFF;


        if(absPoseError <= DrivetrainConstants.DriveToPose.POSE_TOLERANCE)
            targetVelocity = 0;

        targetVelocity = Math.signum(targetVelocity) * Math.min(Math.abs(targetVelocity),
                DrivetrainConstants.MAX_LINEAR_SPEED);

        var driveVelocity = new Translation2d(targetVelocity, errorAngle);

        double angularVelocityPID = angularPIDController.calculate(pose.getRotation().getRadians(),
                goal.getRotation().getRadians());

        double angularVelocityFF = angularPIDController.getSetpoint().velocity * FFScalar(absAngleError,
                DrivetrainConstants.DriveToPose.FF_MIN_ANGLE, DrivetrainConstants.DriveToPose.FF_MAX_ANGLE);

        double angularVelocity = angularVelocityPID + angularVelocityFF;


        if(absAngleError <= DrivetrainConstants.DriveToPose.ANGLE_TOLERANCE)
            angularVelocity = 0;

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(), driveVelocity.getY(), angularVelocity,
                pose.getRotation());

        drivetrain.drive(speeds);

        //Logging
        Logger.recordOutput("driveToPose/lastSetPoint", new Pose2d(lastSetPoint,
                Rotation2d.fromRadians(angularPIDController.getSetpoint().position)));

        Logger.recordOutput("driveToPose/nextState/position", nextState.position);
        Logger.recordOutput("driveToPose/nextState/velocity", nextState.velocity);

        Logger.recordOutput("driveToPose/angularError", absAngleError);
        Logger.recordOutput("driveToPose/poseError", absPoseError);

        Logger.recordOutput("driveToPose/velocity/total", targetVelocity);
        Logger.recordOutput("driveToPose/velocity/PID", targetVelocityPID);
        Logger.recordOutput("driveToPose/velocity/feedForward", targetVelocityFF);

        Logger.recordOutput("driveToPose/angularVelocity/total", angularVelocity);
        Logger.recordOutput("driveToPose/angularVelocity/PID", angularVelocityPID);
        Logger.recordOutput("driveToPose/angularVelocity/feedForward", angularVelocityFF);



    }

    @Override
    public boolean isFinished() {
        var speeds = chassisSpeedSupplier.get();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
        return absAngleError <= DrivetrainConstants.DriveToPose.ANGLE_TOLERANCE
                && absPoseError <= DrivetrainConstants.DriveToPose.POSE_TOLERANCE
                && linearSpeed <= DrivetrainConstants.DriveToPose.VELOCITY_TOLERANCE
                && angularSpeed <= DrivetrainConstants.DriveToPose.ANGULAR_VELOCITY_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds());
    }
}
