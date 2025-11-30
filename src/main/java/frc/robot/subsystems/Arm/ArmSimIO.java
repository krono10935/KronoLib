package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.arm.BasicArmSim;

public class ArmSimIO implements ArmIO{


    BasicMotor motor;

    
    public ArmSimIO() {
        motor = new BasicArmSim(ArmConstants.config);
        
    }
//  @Override
//     public void resetEncoder() {
//         motor.resetEncoder(0);
//     }

    @Override
    public void update(ArmInputs inputs) {
        inputs.atSetPoint = motor.atSetpoint();
        inputs.currentAngle = Rotation2d.fromRotations(motor.getPosition());
    }

    @Override
    public void setMotorAngle(Rotation2d Angle) {
        motor.setControl(Angle.getRotations(), ControlMode.POSITION);
    }

    @Override 
    public double getMotorPos() {
        // TODO Auto-generated method stub
        return 0;
    }
    @Override
    public void stop(){
        motor.stop();
    }

    @Override
    public Rotation2d getVelocity(){
        return Rotation2d.fromRotations(motor.getVelocity());
    }

    @Override
    public void setArmMotorDutyCycle(double duty ) {
        motor.setControl(duty, ControlMode.VELOCITY);
    }

    @Override
    public void setBrake(){
        motor.setIdleMode(io.github.captainsoccer.basicmotor.BasicMotor.IdleMode.BRAKE);
    }

    @Override
    public void setCoast(){
        motor.setIdleMode(io.github.captainsoccer.basicmotor.BasicMotor.IdleMode.COAST);
    }

    @Override
    public void setVoltage(double volt){
        motor.setVoltage(volt);
    }
}