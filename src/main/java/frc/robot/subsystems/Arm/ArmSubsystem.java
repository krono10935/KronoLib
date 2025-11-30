package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm.ArmConstants.ArmLevel;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO io;
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();
    private ArmConstants.ArmLevel targetLevel;
    private static final Alert unknownArmLevelAlert = new Alert("Attempted to set arm level to unknown angle",AlertType.kWarning);

    private final SysIdRoutine routine;

    public ArmSubsystem() {
        io = RobotBase.isReal() ? new ArmRealMotorIO() : new ArmSimIO();

        routine = new SysIdRoutine(
        new SysIdRoutine.Config(null, Units.Volts.of(2), null, 
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volt) -> io.setVoltage(volt.baseUnitMagnitude()),
            null,
            this
        )
    );
    }

    public ArmLevel getCurrentLevel(){
        Rotation2d angle = inputs.currentAngle;
        if (angle.getDegrees() == ArmLevel.HOME.angle.getDegrees()){
            return ArmLevel.HOME;
        }

        if (angle.getDegrees() == ArmLevel.L1.angle.getDegrees()){
            return ArmLevel.L1;
        }
        
        if(angle.getDegrees() == ArmLevel.L2.angle.getDegrees()){
            return ArmLevel.L2;
        }

        if (angle.getDegrees() == ArmLevel.L3.angle.getDegrees()){
            return ArmLevel.L3;
        }
        
        return ArmLevel.UNKNOWN;   
    }


    @Override
    public void periodic() {
        io.update(inputs);
        Logger.processInputs(getName(), inputs);

        String currentCommandName = (getCurrentCommand() == null) ? "Null" : getCurrentCommand().getName();
        Logger.recordOutput("Arm/CurrentCommand", currentCommandName);
        
        Logger.recordOutput("Arm/currentLevel", getCurrentLevel().name());
        Logger.recordOutput("Arm/current angle", inputs.currentAngle.getDegrees());
        
    }


    public void setAngleByLevel(ArmConstants.ArmLevel level) {
        if(level == ArmConstants.ArmLevel.UNKNOWN){
            unknownArmLevelAlert.set(true);
            return;
        }
        unknownArmLevelAlert.set(false);
        io.setMotorAngle(level.angle);
        targetLevel = level;
        Logger.recordOutput("Arm/Target Level", level.name());
    }

    public Rotation2d getCurrentAngle() {
        return inputs.currentAngle;
    }

    public boolean isAtSetPoint() {
        return inputs.atSetPoint;
    }

    public ArmConstants.ArmLevel getTargetLevel(){
        return targetLevel;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }


}

