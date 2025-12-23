package frc.robot.subsystems.drivetrain.module.constants;

import edu.wpi.first.math.geometry.Translation2d;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;
import io.github.captainsoccer.basicmotor.gains.FeedForwardsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

public enum SwerveModulesMK5 {

    //TODO: set actual modules
    FRONT_LEFT(1,1,1,null,null,1,1,null,null,1,1,null);


    SwerveModulesMK5(int canCoderID,
                     double zeroOffset,
                     int driveMotorID,
                     PIDGains drivePIDGains,
                     FeedForwardsGains driveFeedForwards,
                     double driveKA,
                     int steerMotorID,
                     PIDGains steerPIDGains,
                     FeedForwardsGains steerFeedForwards,
                     double steerKV,
                     double steerKA,
                     Translation2d location) {
        BasicTalonFXConfig driveConfig = getGenericConf().driveConfig().copy();
        BasicTalonFXConfig steerConfig = getGenericConf().steerConfig().copy();

        driveConfig.motorConfig.id = driveMotorID;
        steerConfig.motorConfig.id = steerMotorID;

        driveConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(drivePIDGains);
        steerConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(steerPIDGains);

        driveConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(driveFeedForwards);
        steerConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(steerFeedForwards);

        driveConfig.simulationConfig.kA = driveKA;

        steerConfig.simulationConfig.kV = steerKV;
        steerConfig.simulationConfig.kA = steerKA;

        constants = new SwerveModuleConstantsRecord(canCoderID, zeroOffset, driveConfig, steerConfig, location,this.name());



    }



    public final SwerveModuleConstantsRecord constants;
    public ModuleConstantsGeneric getGenericConf(){
        if(genericConf==null) genericConf = new ModuleConstantsGeneric(null,null,1,1);
        return genericConf;
    }
    public static  ModuleConstantsGeneric genericConf;
}
