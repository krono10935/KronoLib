package frc.robot.subsystems.drivetrain.swerve.module;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleFake extends SwerveModuleIO {
    public SwerveModuleFake(SwerveModuleConstants constants){
        super(constants);
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        
    }

    @Override
    public void setTargetStateVoltages(SwerveModuleState targetState) {
        
    }

    @Override
    protected double getDriveVelocity() {
        return 0;
    }

    @Override
    protected double getDriveDistance() {
        return 0;
    }

    @Override
    protected double getSteerAngle() {
        return 0;
    }

    @Override
    public void setBrakeMode(boolean isBrake) {

    }


    @Override
    public void update(){
        super.update();
    }

    public void setDriveVoltage(double voltage){

    }

    public void setSteerVoltage(double voltage){

    }

    @Override
    public void usePowerAndAngle(double voltage, double angle) {

    }
}
