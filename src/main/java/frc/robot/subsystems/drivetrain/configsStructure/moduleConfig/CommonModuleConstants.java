package frc.robot.subsystems.drivetrain.configsStructure.moduleConfig;

import com.pathplanner.lib.config.ModuleConfig;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;

/**
 * Generic per-chassis configuration for swerve modules.
 * <p>
 * This record is intended to be shared by all swerve modules on the same chassis.
 * It bundles together the drive and steer motor config baselines
 * {@link ModuleConfig} used for PP
 */
public record CommonModuleConstants(BasicTalonFXConfig DRIVE_CONFIG, BasicTalonFXConfig STEER_CONFIG, ModuleConfig MODULE_CONFIG, double STEER_SPEED_REDUCTION) {


    /**
     *
     * @return the module config for Path Planner.
     */
    private static ModuleConfig getModuleConfig(
            BasicTalonFXConfig driveConfig, BasicTalonFXConfig steerConfig, double wheelCOF, double maxLinearSpeed){

        return new ModuleConfig(
                //goofy derivation from reversing the unitConversion
                driveConfig.motorConfig.unitConversion/(2 * Math.PI ),
                maxLinearSpeed, wheelCOF,
                driveConfig.motorConfig.motorType.withReduction(driveConfig.motorConfig.gearRatio),
                driveConfig.currentLimitConfig.statorCurrentLimit, 1);
    }


    /**
     * @param driveConfig config for drive motor
     * @param steerConfig config for steer motor
     * @param wheelCOF coefficient of friction of wheel
     * @param maxLinearSpeed max linear speed of the module
     * @param steerSpeedReduction reduction of the steer motor speed, in range of 0-1, multiplies the max steer speed
     */
    public CommonModuleConstants(
            BasicTalonFXConfig driveConfig, BasicTalonFXConfig steerConfig, double wheelCOF, double maxLinearSpeed ,double steerSpeedReduction){
                this(driveConfig, steerConfig,getModuleConfig(driveConfig, steerConfig, wheelCOF, maxLinearSpeed),steerSpeedReduction);
    }

    /**
     *
     * @return max speed of the steer motor in radians per second
     */
    public double maxSteerSpeed(){
        return STEER_CONFIG.motorConfig.motorType.freeSpeedRadPerSec/
                STEER_CONFIG.motorConfig.gearRatio * STEER_SPEED_REDUCTION;
    }


}
