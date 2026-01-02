package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFX;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;

public class SwerveModuleBasic extends SwerveModuleIO {

    private final BasicMotor drivingMotor;
    private final BasicMotor steeringMotor;

    private final CANcoder canCoder;

    public SwerveModuleBasic(SwerveModuleConstants constants){
        super(constants);

        if (RobotBase.isReal()) {
            drivingMotor = new BasicTalonFX(constants.DRIVING_CONFIG);
            steeringMotor = new BasicTalonFX(constants.STEERING_CONFIG);

            canCoder = createCANcoder(constants);
            steeringMotor.resetEncoder(canCoder.getPosition().getValueAsDouble());

            ((BasicTalonFX) steeringMotor).useRemoteCanCoder(canCoder);

            canCoder.getMagnetHealth().setUpdateFrequency(4);
            canCoder.getPosition().setUpdateFrequency(50);
            canCoder.optimizeBusUtilization();

        }
        else{
            drivingMotor = new BasicMotorSim(constants.DRIVING_CONFIG);
            steeringMotor = new BasicMotorSim(constants.STEERING_CONFIG);

            canCoder = null;
        }
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        drivingMotor.setControl(targetState.speedMetersPerSecond,ControlMode.VELOCITY);
        steeringMotor.setControl(targetState.angle.getRotations(),ControlMode.POSITION);
    }

    @Override
    public void setTargetStateVoltages(SwerveModuleState targetState) {
        drivingMotor.setVoltage(targetState.speedMetersPerSecond);
        steeringMotor.setControl(targetState.angle.getRotations(),ControlMode.POSITION);
    }

    @Override
    protected double getDriveVelocity() {
        return drivingMotor.getVelocity();
    }

    @Override
    protected double getDriveDistance() {
        return drivingMotor.getPosition();
    }

    @Override
    protected double getSteerAngle() {
        return steeringMotor.getPosition();
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        IdleMode idleMode = isBrake ? IdleMode.BRAKE : IdleMode.COAST;

        drivingMotor.setIdleMode(idleMode);
        steeringMotor.setIdleMode(idleMode);
    }

    /**
     * Auto configs a CANCoder for the module
     * @param constants
     * @return A configured CANCoder for the module
     */
    private static CANcoder createCANcoder(SwerveModuleConstants constants){
        CANcoder encoder = new CANcoder(constants.CAN_CODER_ID);
        var config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -constants.ZERO_OFFSET;

        encoder.getConfigurator().apply(config);
        return encoder;
    }

    @Override
    public void update(){
        super.update();
        if (canCoder != null){
            Logger.recordOutput("basic module/" + constants.name() + "/magnet health",
                    canCoder.getMagnetHealth().toString());
            Logger.recordOutput("basic module/" + constants.name() + "/cancoder pos", canCoder.getPosition().getValueAsDouble()  );
        }
        SmartDashboard.putData(drivingMotor.getController());
    }

    public void setDriveVoltage(double voltage){
        drivingMotor.setVoltage(voltage);
    }

    public void setSteerVoltage(double voltage){
        steeringMotor.setVoltage(voltage);
    }

    @Override
    public void usePowerAndAngle(double voltage, double angle) {
        setDriveVoltage(voltage);
        steeringMotor.setControl(angle, ControlMode.POSITION);
    }
}
