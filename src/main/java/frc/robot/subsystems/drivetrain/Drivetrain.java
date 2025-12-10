package frc.robot.subsystems.drivetrain;


import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.PPController;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.GyroIOSim;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.drivetrain.module.SwerveModuleBasic;
import frc.robot.subsystems.drivetrain.module.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIO;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;

public class Drivetrain extends SubsystemBase {

    @AutoLog
    public static class DrivetrainInputs {
        public Rotation2d gyroAngle = Rotation2d.kZero;
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    }

    private final GyroIO gyroIO;

    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    private final SwerveModuleIO[] io = new SwerveModuleIO[4];

    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Supplier<Double> batteryVoltageSupplier;

    private final SwerveSetpointGenerator setpointGenerator;

    private SwerveSetpoint previousSetpoint;



    public Drivetrain(BooleanSupplier isRedAlliance, Supplier<Double> batteryVoltageSupplier) {
        if(RobotBase.isReal()) {
            gyroIO = new GyroIOSim(this::getChassisSpeeds, isRedAlliance); //TODO: create real gyro
        }
        else {
            gyroIO = new GyroIOSim(this::getChassisSpeeds, isRedAlliance);
        }

        for(int i=0;i<4;i++){
            io[i] = new SwerveModuleBasic(SwerveModuleConstants.values()[i]);
            inputs.moduleStates[i] = io[i].getState();
            modulePositions[i] = io[i].getPosition();
        }

        this.batteryVoltageSupplier = batteryVoltageSupplier;

        setpointGenerator = new SwerveSetpointGenerator(
                DrivetrainConstants.ROBOT_CONFIG,
                SwerveModuleConstants.STEER_MAX_SPEED
        );

        previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), inputs.moduleStates,
                DriveFeedforwards.zeros(inputs.moduleStates.length));

        kinematics = new SwerveDriveKinematics(SwerveModuleConstants.getModuleTranslations());

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), modulePositions,
                DrivetrainConstants.startPose2d);

        var setBrake = new InstantCommand(() -> setBrakeMode(true))
                .ignoringDisable(true);


        var setCoast = new InstantCommand(() -> setBrakeMode(false))
                .ignoringDisable(true);

        new Trigger(RobotState::isEnabled)
                .onTrue(setBrake)
                .onFalse(setCoast);

        setCoast.schedule();

    }


    private void configPathPlanner(){

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getEstimatedPosition, // Robot pose supplier
                this::reset, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(DrivetrainConstants.DriveToPose.LINEAR_PID_GAINS.getK_P(),
                                DrivetrainConstants.DriveToPose.LINEAR_PID_GAINS.getK_I() ,
                                DrivetrainConstants.DriveToPose.LINEAR_PID_GAINS.getK_D()), // Translation PID constants
                        new PIDConstants(DrivetrainConstants.DriveToPose.ANGULAR_PID_GAINS.getK_P(),
                                DrivetrainConstants.DriveToPose.ANGULAR_PID_GAINS.getK_I() ,
                                DrivetrainConstants.DriveToPose.ANGULAR_PID_GAINS.getK_D()) // Rotation PID constants
                ),
                DrivetrainConstants.ROBOT_CONFIG, // The robot configuration
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
                }
        );
    }

    /**
     * Drives the robot at relative speed
     *
     * @param speeds the target speed of the robot
     */
    public void drive(ChassisSpeeds speeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, null,
                DrivetrainConstants.LOOP_TIME_SECONDS, batteryVoltageSupplier.get());

        for (int i = 0; i < 4; i++){
            var targetSpeed = previousSetpoint.moduleStates()[i];
            io[i].setTargetState(targetSpeed);
        }
        Logger.recordOutput("drivetrain/swerve/target states", previousSetpoint.moduleStates());
    }


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
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }
    /**
     * Return the latest position of the robot
     *
     * @return the latest pose
     */
    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        inputs.gyroAngle = gyroIO.update();

        for (int i=0;i<4;i++){
            io[i].update();
            this.inputs.moduleStates[i] = io[i].getState();
            modulePositions[i] = io[i].getPosition();
        }

        inputs.speeds = kinematics.toChassisSpeeds(this.inputs.moduleStates);

        poseEstimator.update(getGyroAngle(), modulePositions);

        gyroIO.getEstimatedPosition().ifPresent((
                pose -> poseEstimator.addVisionMeasurement(pose.pose(), Timer.getTimestamp(), pose.stdDevs())));

        Logger.processInputs("drivetrain", inputs);
        Logger.recordOutput("drivetrain/estimated pose", getEstimatedPosition());

        String currentCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();

        Logger.recordOutput("drivetrain/current command", currentCommand);
    }


    /**
     * Reset the gyro and the pose estimator states
     * @param newPose new pose of the robot
     */
    public void reset(Pose2d newPose){
        gyroIO.reset(newPose);
        poseEstimator.resetPosition(newPose.getRotation(), modulePositions, newPose);
    }

    public void setBrakeMode(boolean isBrake){
        for (SwerveModuleIO module : io){
            module.setBrakeMode(isBrake);
        }
    }

}

