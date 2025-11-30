package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmInputs {
        public boolean atSetPoint;
        public Rotation2d currentAngle; //absolute
    }

    // /*
    //  * reset the encoder of the arm motor
    //  */
    // void resetEncoder();

    /*
     * update the auto logged inputs
     */
    void update(ArmInputs inputs);

    /*
     * set the armMotor position (absolute)
     */
    void setMotorAngle(Rotation2d Angle);

    double getMotorPos();

    void stop();
    
    Rotation2d getVelocity();

    void setArmMotorDutyCycle(double duty);

    void setCoast();
    void setBrake();

    void setVoltage(double volt);
    

    
}