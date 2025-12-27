package frc.robot.subsystems.drivetrain;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PPController;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.GyroIONavx;
import frc.robot.subsystems.drivetrain.gyro.GyroIOSim;
import frc.robot.subsystems.drivetrain.DrivetrainInputsAutoLogged;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

public abstract class Drivetrain extends SubsystemBase {

    @AutoLog
    public static class DrivetrainInputs {
        public Rotation2d gyroAngle = Rotation2d.kZero;
        public ChassisSpeeds speeds = new ChassisSpeeds();
    }

    private final GyroIO gyroIO;

    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    private Pose2d pathPlannerTargetPose = Pose2d.kZero;
    RobotConfig config;


    protected Drivetrain(BooleanSupplier isRedAlliance) {
        if(RobotBase.isReal()) {
            gyroIO = new GyroIONavx(isRedAlliance);
        }
        else {
            gyroIO = new GyroIOSim(this::getChassisSpeeds, isRedAlliance);
        }
        
        
        
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getEstimatedPosition, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(DrivetrainConstants.LINEAR_PID_GAINS.getK_P(),
                                DrivetrainConstants.LINEAR_PID_GAINS.getK_I() ,
                                DrivetrainConstants.LINEAR_PID_GAINS.getK_D()), // Translation PID constants
                        new PIDConstants(DrivetrainConstants.ANGULAR_PID_GAINS.getK_P(),
                                DrivetrainConstants.ANGULAR_PID_GAINS.getK_I() ,
                                DrivetrainConstants.ANGULAR_PID_GAINS.getK_D()) // Rotation PID constants
                ),
                config, // The robot configuration
                DrivetrainConstants::shouldFlipPath,
                this // Reference to this subsystem to set requirements
    );

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> {
                    var posesArr = poses.toArray(new Pose2d[0]);

                    Logger.recordOutput("DriveTrain/PathPlanner/active path", posesArr);
                }
        );

        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    Logger.recordOutput("DriveTrain/PathPlanner/target pose", pose);
                    this.pathPlannerTargetPose = pose;
                }
        );
    }

    /**
     * Drives the robot at relative speed
     *
     * @param speeds the target speed of the robot
     */
    public void drive(ChassisSpeeds speeds) {
        var discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        setChassisSpeed(discreteSpeeds);
        Logger.recordOutput("drivetrain/target speed", discreteSpeeds);
    }

    /**
     * Forwards the speeds to the implementation
     *
     * @param speeds the target speed of the robot
     */
    protected abstract void setChassisSpeed(ChassisSpeeds speeds);

    /**
     * Return the latest gyro angle
     * (counter clockwise positive)
     *
     * @return the gyro angle
     */
    public Rotation2d getGyroAngle() {
        return inputs.gyroAngle;
    }

    /**
     * Return the latest speeds of the robot
     *
     * @return speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return inputs.speeds;
    }

    /**
     * Adds the vision measurement
     *
     * @param pose      the position where the vision think the robot is there
     * @param timestamp the time when the pose was taken
     * @param stdDevs   A Vector with 3 parameters in the following order:
     *                  X standard deviation (in meters).
     *                  Y standard deviation (in meters).
     *                  Theta standard deviation (in radians).
     */
    // @Override
    public abstract void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);

    /**
     * Return the latest position of the robot
     *
     * @return the latest pose
     */
    public abstract Pose2d getEstimatedPosition();

    @Override
    public void periodic() {
        inputs.gyroAngle = gyroIO.update();
        updateInputs(inputs);
        Logger.processInputs("drivetrain", inputs);
        Logger.recordOutput("drivetrain/estimated pose", getEstimatedPosition());

        String currentCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();

        Logger.recordOutput("drivetrain/current command", currentCommand);
    }

    /**
     * Update the inputs from the sensors
     * @param inputs the inputs that stores the data
     */
    protected abstract void updateInputs(DrivetrainInputs inputs);

    /**
     * Reset the gyro and the pose estimator states
     * @param newPose
     */
    public void reset(Pose2d newPose){
        gyroIO.reset(newPose.getRotation());
        resetPose(newPose);
    }

    public Pose2d getPathFinalPose(){
        return pathPlannerTargetPose;
    }

    /**
     * Resets the pose of the pose estimator
     * @param newPose
     */
    protected abstract void resetPose(Pose2d newPose);

    public Command driveToPosCommand(Pose2d targetPose){
        // System.out.println(targetPose);
         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(getEstimatedPosition(), targetPose);
         PathConstraints constraints = new PathConstraints(1.0, 1.0, 1.0, 1.0); //dummy
         PathPlannerPath path = new PathPlannerPath(waypoints, constraints,null, new GoalEndState(0.0, targetPose.getRotation()));
         return AutoBuilder.followPath(path);

    }
    
    public Command runBackCommand(){
        return new RunCommand(() -> 
        drive(
            new ChassisSpeeds(
                0.1* DrivetrainConstants.MAX_LINEAR_SPEED, 0.1* DrivetrainConstants.MAX_LINEAR_SPEED, 0)) , this).
        withTimeout(0.5);
    }
}

