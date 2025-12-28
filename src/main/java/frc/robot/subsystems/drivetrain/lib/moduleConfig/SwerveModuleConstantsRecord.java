package frc.robot.subsystems.drivetrain.lib.moduleConfig;

import edu.wpi.first.math.geometry.Translation2d;

import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;

/**
 * Simple container record holding all constants required to construct a single
 * swerve module instance.
 *
 * <p>It groups the CANcoder ID and absolute zero offset with the motor configs
 * for driving and steering, along with the module's translation from the robot
 * center and a human-readable name.</p>
 *
 * @param canCoderID      CAN ID of the module's absolute encoder (CANcoder)
 * @param zeroOffset      absolute angle offset in rotations from the zero position
 * @param drivingConfig   configuration for the drive TalonFX motor
 * @param steeringConfig  configuration for the steer TalonFX motor
 * @param translation     module position relative to the robot center
 * @param name            descriptive name of the module
 */
public record SwerveModuleConstantsRecord(
        int canCoderID,
        double zeroOffset, BasicTalonFXConfig drivingConfig, BasicTalonFXConfig steeringConfig,
        Translation2d translation, String name) {
}



