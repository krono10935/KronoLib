package frc.robot.commands.auto;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.swerve.Swerve;
import io.github.captainsoccer.basicmotor.LogFrame;

import java.util.function.Supplier;


public class DriveToPoseCommand extends Command {
    private Drivetrain drivetrain;
    private Supplier<Pose2d> targetPose;
    private final Supplier<Pose2d> robotPose;
    private TrapezoidProfile driveProfile;
    private Translation2d lastSetpointVelocity = Translation2d.kZero;


    public DriveToPoseCommand(Drivetrain drivetrain, Supplier<Pose2d> targetPose) {
        robotPose = drivetrain::getEstimatedPosition;
        addRequirements(drivetrain);
    }

    public ChassisSpeeds getChassisSpeeds() {

    }


    public LogFrame.PIDOutput getPIDOutput() {

    }


    private TrapezoidProfile.State lastState;
    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        driveProfile = new TrapezoidProfile(RobotState.isAutonomous() ?
                        DrivetrainConstants.DriveToPose.AUTO_CONSTRAINTS :
                        DrivetrainConstants.DriveToPose.TELE_CONSTRAINTS);

        var errorVector = robotPose.get().getTranslation().minus(targetPose.get().getTranslation())
        //velocityVector*errorVector/velocityVector
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {

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
