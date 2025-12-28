package frc.robot.subsystems.lib.moduleConfig;

import com.pathplanner.lib.config.ModuleConfig;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;

/**
 * Generic per-chassis configuration for swerve modules.
 * <p>
 * This record is intended to be shared by all swerve modules on the same chassis.
 * It bundles together the drive and steer motor config baselines
 * {@link ModuleConfig} used for PP
 */
public record ModuleConstantsGeneric(BasicTalonFXConfig driveConfig, BasicTalonFXConfig steerConfig, ModuleConfig moduleConfig) {

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




    public ModuleConstantsGeneric (
            BasicTalonFXConfig driveConfig, BasicTalonFXConfig steerConfig, double wheelCOF, double maxLinearSpeed){
        this(driveConfig, steerConfig,getModuleConfig(driveConfig, steerConfig, wheelCOF, maxLinearSpeed));
    }


}
