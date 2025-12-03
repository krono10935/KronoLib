package frc.robot.subsystems.drivetrain.swerve;


import java.lang.invoke.VolatileCallSite;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleBasic;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.swerve.module.SwerveModuleIO;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;

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

    public Swerve(BooleanSupplier isRedAlliance) {
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
    public void setSteerVoltage(double voltage){
        for (SwerveModuleIO module : io){
            module.setSteerVoltage(voltage);
        }
    }

    @Override
    public void setDriveVoltage(double voltage){
        for (SwerveModuleIO module : io){
            module.setDriveVoltage(voltage);
        }
    }

    @Override
    protected void setChassisSpeed(ChassisSpeeds speeds) {
        var targetSpeeds = kinematics.toWheelSpeeds(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(targetSpeeds, DrivetrainConstants.MAX_LINEAR_SPEED);
        
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

    @Override
    public void usePowerAndAngle(double voltage, Rotation2d angle) {
        var targetSpeeds = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            targetSpeeds[i] = new SwerveModuleState(
                voltage,
                angle
            );
        }
        
        for (int i = 0; i < 4; i++){
            targetSpeeds[i].cosineScale(io[i].getState().angle);
            io[i].setTargetStateVoltages(targetSpeeds[i]);
        }
        Logger.recordOutput("drivetrain/swerve/target states sysid with angle", targetSpeeds);
    }

    @Override
    public void spinWithPower(double voltage) {
        var targetSpeeds = new SwerveModuleState[4];
        for (int i = 0; i < 2; i++){
            targetSpeeds[i] = new SwerveModuleState(
                voltage,
                Rotation2d.fromDegrees(-45 - 90 * i)
            );
        }

        for (int i = 0; i < 2; i++){
            targetSpeeds[i+2] = new SwerveModuleState(
                voltage,
                Rotation2d.fromDegrees(45 + 90 * i)
            );
        }
        
        for (int i = 0; i < 4; i++){
            targetSpeeds[i].cosineScale(io[i].getState().angle);
            io[i].setTargetStateVoltages(targetSpeeds[i]);
        }
        Logger.recordOutput("drivetrain/swerve/target states sysid", targetSpeeds);
    }
}
