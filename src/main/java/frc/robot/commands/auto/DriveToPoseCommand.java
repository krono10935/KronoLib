package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import io.github.captainsoccer.basicmotor.LogFrame;

import java.util.function.Supplier;


public class DriveToPoseCommand extends Command {
    private Drivetrain drivetrain;
    private Supplier<Pose2d> goalPose;
    private final Supplier<Pose2d> robotPose;
    private TrapezoidProfile driveProfile;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private TrapezoidProfile.State lastState;
    private Translation2d lastSetPoint;
    private Vector<N2> lastSetpointVelocity;
    private final static TrapezoidProfile.State GOAL_STATE = new State(0,0);
    private final PIDController drivePIDController;


    public DriveToPoseCommand(Drivetrain drivetrain, Supplier<Pose2d> targetPose, Supplier<ChassisSpeeds> chassisSpeedSupplier, PIDController drivePIDController) {
        robotPose = drivetrain::getEstimatedPosition;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.drivePIDController = drivePIDController;
        addRequirements(drivetrain);
    }

    public ChassisSpeeds getChassisSpeeds() {

    }


    public LogFrame.PIDOutput getPIDOutput() {

    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        driveProfile = new TrapezoidProfile(RobotState.isAutonomous() ?
                        DrivetrainConstants.DriveToPose.AUTO_CONSTRAINTS :
                        DrivetrainConstants.DriveToPose.TELE_CONSTRAINTS);

        var errorVector = robotPose.get().getTranslation().minus(goalPose.get().getTranslation()).toVector();
        //velocityVector*errorVector/velocityVector
        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeedSupplier.get(), robotPose.get().getRotation());
        double startVelocityScalar = errorVector.dot(
                VecBuilder.fill(speeds.vxMetersPerSecond,  speeds.vyMetersPerSecond))
                /errorVector.norm();
        lastState = new  TrapezoidProfile.State(errorVector.norm(), -startVelocityScalar);
    }

    /**
     * calculates the next state in the profile.
     * @param goal final goal
     * @param pose current pose
     * @param error the distance between the current pose and the goal.
     * @return the next state of the profile
     */
    public TrapezoidProfile.State calcProfile(Pose2d goal, Pose2d pose, double error) {
        var profileDirection = goal.getTranslation().minus(lastSetPoint).toVector();

        //if the distance from the current pose to the goal is less then a content then we will not scale the velocity
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
        lastSetPoint = pose.getTranslation().times(scalar).plus(goal.getTranslation().times(1 - scalar));

        return nextState;
    }

    private final static double FF_MIN_DISTANCE = DrivetrainConstants.DriveToPose.FF_MIN_DISTANCE;
    private final static double FF_MAX_DISTANCE = DrivetrainConstants.DriveToPose.FF_MAX_DISTANCE;

    /**
     * calc the FFScalar for the velocity
     * @param error the distance between the current pose and the goal.
     * @return how much should the velocity FF affect the output.
     */
    public double FFScalar(double error){
        return MathUtil.clamp((error - FF_MIN_DISTANCE)/ (FF_MAX_DISTANCE - FF_MIN_DISTANCE),
                0.0, 1.0);
    }

    @Override
    public void execute() {
        Pose2d pose = robotPose.get();
        Pose2d goal = goalPose.get();
        var poseError = pose.relativeTo(goal);
        double absError = poseError.getTranslation().getNorm();
        var nextState = calcProfile(goal, pose, absError);

        //
        double targetVelocity = drivePIDController.calculate(absError, nextState.position)
                + nextState.velocity * FFScalar(absError);




    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     *
     */
    @Override
    public boolean isFinished() {

        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
