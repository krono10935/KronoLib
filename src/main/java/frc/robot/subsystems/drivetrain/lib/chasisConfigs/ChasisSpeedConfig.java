package frc.robot.subsystems.drivetrain.lib.chasisConfigs;

/**
 * Chassis-level linear speed limits for the drivetrain.
 *
 * @param minLinearSpeed minimum commanded linear speed in m/s
 * @param maxLinearSpeed maximum commanded linear speed in m/s
 */
public record ChasisSpeedConfig(double minLinearSpeed, double maxLinearSpeed) {

}
