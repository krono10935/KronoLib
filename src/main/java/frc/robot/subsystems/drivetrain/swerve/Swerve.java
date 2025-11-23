package frc.robot.subsystems.drivetrain.swerve;


import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleBasic;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.swerve.SwerveInputsAutoLogged;

public class Swerve extends Drivetrain {
    
    @AutoLog
    public static class SwerveInputs{
        public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    }

    private final SwerveModuleIO[] io = new SwerveModuleIO[4];

    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    public Swerve(BooleanSupplier isRedAlliance){
        super(isRedAlliance);
        for(int i=0;i<4;i++){
            io[i] = new SwerveModuleBasic(SwerveModuleConstants.values()[i]);
            inputs.moduleStates[i] = io[i].getState();
            modulePositions[i] = io[i].getPosition();
        }

        kinematics = new SwerveDriveKinematics(SwerveModuleConstants.getModuleTranslations());

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), modulePositions, DrivetrainConstants.startPose2d);

        var setBrake = new InstantCommand(() -> setBrakeMode(true))
                                .ignoringDisable(true);
        
        
        var setCoast = new InstantCommand(() -> setBrakeMode(false))
                                .ignoringDisable(true);

        new Trigger(RobotState::isEnabled)
                            .onTrue(setBrake)
                            .onFalse(setCoast);

        setCoast.schedule();
    }

    @Override
    protected void setChassisSpeed(ChassisSpeeds speeds) {
        var targetSpeeds = kinematics.toWheelSpeeds(speeds);
        for (int i = 0; i < 4; i++){
            //
            targetSpeeds[i].optimize(io[i].getState().angle);
            //
            targetSpeeds[i].cosineScale(io[i].getState().angle);
            io[i].setTargetState(targetSpeeds[i]);
        }
        Logger.recordOutput("drivetrain/swerve/target states", targetSpeeds);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    @Override
    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    protected void updateInputs(DrivetrainInputs inputs) {
        for (int i=0;i<4;i++){
            io[i].update();
            this.inputs.moduleStates[i] = io[i].getState();
            modulePositions[i] = io[i].getPosition();
        }
        Logger.recordOutput("swerve real states "  , this.inputs.moduleStates);
        
        inputs.speeds = kinematics.toChassisSpeeds(this.inputs.moduleStates);

        poseEstimator.update(getGyroAngle(), modulePositions);


        Logger.processInputs("drivetrain/swerve", this.inputs);
        
    }

    public void setBrakeMode(boolean isBrake){
        for (SwerveModuleIO module : io){
            module.setBrakeMode(isBrake);
        }
    }

    @Override
    protected void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(newPose.getRotation(), modulePositions, newPose);
    }

    
}
