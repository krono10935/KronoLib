package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.ModuleConstants;

public abstract class SwerveModuleIO {

    public final ModuleConstants constants;

    private final SwerveModuleState currentState = new SwerveModuleState();

    private final SwerveModulePosition position = new SwerveModulePosition();

    protected SwerveModuleIO(ModuleConstants constants){
        this.constants = constants;
    }

    /**
     * Set the motors to the target state of the system
     * @param targetState
     */
    public abstract void setTargetState(SwerveModuleState targetState);


    public SwerveModuleState getState(){
        return currentState;
    }

    public SwerveModulePosition getPosition(){
        return position;
    }

    /**
     * Update the module's attributes
     */
    public void update(){
        currentState.angle = Rotation2d.fromRotations(getSteerAngle());
        position.angle = currentState.angle;
        currentState.speedMetersPerSecond = getDriveVelocity();
        position.distanceMeters = getDriveDistance();

    }

    /**
     * @return The drive motor speed in meters per second
     */
    protected abstract double getDriveVelocity();

    /**
     * @return The drive motor position in meters
     */
    protected abstract double getDriveDistance();

    /**
     * @return The steer motor position in rotations
     */
    protected abstract double getSteerAngle();

    /**
     * Set if the module is Brake or Coast
     * @param isBrake
     */
    public abstract void setBrakeMode(boolean isBrake);

    public abstract void setDriveVoltageAndSteerAngle(double voltage, Rotation2d angle);

    public abstract void setSteerVoltage(double voltage);



}
