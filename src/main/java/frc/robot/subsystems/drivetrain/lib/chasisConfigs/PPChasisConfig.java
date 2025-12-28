package frc.robot.subsystems.drivetrain.lib.chasisConfigs;

import com.pathplanner.lib.config.PIDConstants;


/**
 * record containing the information relevant to the chasis relative to PP
 * @param LOOP_TIME_SECONDS
 * @param PID_CONSTANTS
 * @param ANGULAR_PID_CONSTANTS
 * @param massKG
 * @param MOI
 */
public record PPChasisConfig(double LOOP_TIME_SECONDS, PIDConstants PID_CONSTANTS,
                             PIDConstants ANGULAR_PID_CONSTANTS, double massKG, double MOI) {

}
