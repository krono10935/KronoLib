package frc.robot.subsystems.drivetrain.constants;

import com.pathplanner.lib.config.PIDConstants;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants.ChassisSpeedConfig;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants.PPChassisConfig;
import frc.robot.subsystems.drivetrain.gyro.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.gyro.GyroType;
import frc.robot.subsystems.drivetrain.module.constants.SwerveModulesMK4;

public enum ChassisType {
    DEVBOT(new ChassisConstants(
            SwerveModulesMK4.getConstants(),
            new ChassisSpeedConfig(1,4), SwerveModulesMK4.getGenericConf(),
            new PPChassisConfig(new PIDConstants(0),new PIDConstants(0), 30, 6 ), GyroType.NAVX,2)),
    COMPBOT(null);

    public final ChassisConstants constants;


    ChassisType(ChassisConstants constants){
        this.constants = constants;
    }

}
