package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.measurements.Measurements;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager.ControllerLocation;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;
import io.github.captainsoccer.basicmotor.rev.encoders.RevAbsoluteEncoder;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;

public class ArmRealMotorIO implements ArmIO{

    private final BasicMotor motor;
    final DutyCycleEncoder armDutyCycleEncoder;

    public static double ff = 0.4;

    public static double calcFF(double rotation){
        return  Math.sin(Units.rotationsToRadians(rotation)) * ff;
    }


    public ArmRealMotorIO() {
        motor = new BasicSparkMAX(ArmConstants.config);
        // motor.setControllerLocation(ControllerLocation.RIO); 
        
        armDutyCycleEncoder = new DutyCycleEncoder(ArmConstants.DUTY_CYCLE_ENCODER_PORT);
        armDutyCycleEncoder.setInverted(ArmConstants.IS_ABS_ENCODER_INVERTED);

        // SmartDashboard.putData(motor.getController());

        motor.resetEncoder(armDutyCycleEncoder.get() - ArmConstants.DUTY_CYCLE_ENCODER_ZERO_OFFSET);

        // SmartDashboard.putNumber("feedforward", 0.4);
    }

    // @Override
    // public void resetEncoder() {
    //     motor.resetEncoder(armDutyCycleEncoder.get());
    // }

    @Override
    public void update(ArmInputs inputs) {
        inputs.atSetPoint = motor.atGoal();
        inputs.currentAngle = Rotation2d.fromRotations(motor.getPosition());
        Logger.recordOutput("Arm/absolute encoder offset", armDutyCycleEncoder.get());
        ff = SmartDashboard.getNumber("feedForward", ff);
        
    }

    public double getMotorPos(){
        return motor.getPosition();
    }

    @Override
    public void setMotorAngle(Rotation2d Angle) {
        // motor.getController().reset(getMotorPos(), motor.getVelocity());
        motor.setControl(Angle.getRotations(), ControlMode.PROFILED_POSITION);
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
    public void setArmMotorDutyCycle(double duty){
        motor.setControl(duty, ControlMode.VELOCITY);
    }

    @Override
    public void setBrake(){
        motor.setIdleMode(IdleMode.BRAKE);
    }

    @Override
    public void setCoast(){
        motor.setIdleMode(IdleMode.COAST);
    }
    
    @Override
    public void setVoltage(double volt){
        motor.setVoltage(volt);
    }
    
}