package frc.robot.subsystems.Arm;

/*
 * ArmConstants
 * ------------------------------------------------------------
 * Centralized configuration and constants for the Arm subsystem.
 * This file focuses on readability and organization only; no logic
 * or values were altered. Existing commented-out lines were preserved.
 *
 * Sections:
 *  - Imports
 *  - Top-level config objects and encoder constants
 *  - Helper data structures (ArmFeedForwardInputs)
 *  - Static configuration block (motor, PID, feedforward, simulation)
 *  - Arm levels enum
 */

// ============================ Imports ============================
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.MotorConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains.ConstraintType;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

// ============================ Class =============================
public class ArmConstants {

    /*
     * Select the correct motor config implementation based on runtime:
     *  - Real robot: use BasicSparkConfig (REV Spark MAX specific)
     *  - Simulation: fall back to the generic BasicMotorConfig
     */
    public static final BasicMotorConfig config = RobotBase.isReal() ? new BasicSparkConfig() : new BasicMotorConfig();

    public static double kg = 0.64313;

    // -----------------------------------------------------------------
    // Encoder configuration
    // -----------------------------------------------------------------
    /* Port for the absolute encoder which resets position at the start */
    public static Integer DUTY_CYCLE_ENCODER_PORT = 0;

    /* Mechanical zero offset for the absolute encoder (in rotations) */
    public static Double DUTY_CYCLE_ENCODER_ZERO_OFFSET = 0.47;
    public static final boolean IS_ABS_ENCODER_INVERTED = true;

    // ----------------------------------------------------------------- 
    // Helper data structures
    // -----------------------------------------------------------------
    /**
     * Simple container for arm feedforward inputs.
     * Units are represented with WPILib Rotation2d for clarity.
     */
    public static class ArmFeedForwardInputs {
        Rotation2d AngularVelocity;     // rotational velocity
        Rotation2d AngularPositon;      // current position
        Rotation2d AngularAccelaration; // rotational acceleration

        public ArmFeedForwardInputs(
                Rotation2d AngularVelocity,
                Rotation2d AngularPosition,
                Rotation2d AngularAccelaration
        ) {
            this.AngularVelocity = AngularVelocity;
            this.AngularPositon = AngularPosition;
            this.AngularAccelaration = AngularAccelaration;
        }
    }

    // -----------------------------------------------------------------
    // Static configuration block
    // -----------------------------------------------------------------
    static {
        // port for AbsEncoder which resets pos in the beginning of the game

        // ======================= Motor Info ========================
        // Basic motor configuration details
        config.motorConfig.name = "Arm Motor";
        config.motorConfig.id = 20;
        config.motorConfig.inverted = false;
        config.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        config.motorConfig.gearRatio = 45;
        config.motorConfig.unitConversion = 1;
        config.motorConfig.motorType = DCMotor.getNEO(1);

        // ==================== Current Limiting =====================
        ((BasicSparkConfig) config).currentLimitConfig.freeSpeedCurrentLimit = 20;
        ((BasicSparkConfig) config).currentLimitConfig.stallCurrentLimit = 45;
        ((BasicSparkConfig) config).currentLimitConfig.secondaryCurrentLimit = 55;

        // ===================== Closed-loop PID =====================
        // dummy values for pid
        config.slot0Config.pidConfig.kP = 25;
        config.slot0Config.pidConfig.kI = 0.054;
        config.slot0Config.pidConfig.kD = 0;
        config.slot0Config.pidConfig.tolerance = 0.012;
        config.slot0Config.pidConfig.iZone = 0.02;
        config.slot0Config.pidConfig.iMaxAccum = 0.5;

        // Gravity/feedforward helper (unused directly, kept as a note)

        // Output constraints
        config.constraintsConfig.constraintType = ConstraintType.LIMITED;
        config.constraintsConfig.minValue = -0.05;
        config.constraintsConfig.maxValue = 0.5;
        // config.constraintsConfig.voltageDeadband = 0.05;

        // Measurement filters/profile constraints
        config.slot0Config.profileConfig.maximumMeasurementVelocity = 3;
        config.slot0Config.profileConfig.maximumMeasurementAcceleration = 5;
       // config.constraintsConfig.rampRate = 0.05;
        //config.constraintsConfig.maxOutput = 6;
        // config.constraintsConfig.minOutput=-6;

    
        // ======================== Feedforward =======================
        // config.slot0Config.feedForwardConfig.customFeedForward = new ArmFeedForward(0.84);
        config.slot0Config.feedForwardConfig.customFeedForward = (angle) -> Math.sin(Units.rotationsToRadians(angle)) * kg;
        // config.slot0Config.feedForwardConfig.frictionFeedForward = 0.3;

        // ========================= Simulation ======================
        // dummy values for sim
        config.simulationConfig.kA = 0.65532;
        config.simulationConfig.kV = 3.7916;
        config.simulationConfig.armSimConfig.simulateGravity = false;
        config.simulationConfig.armSimConfig.armlengthMeters = 0.55;
        config.simulationConfig.armSimConfig.startingAngle = 0;
        config.simulationConfig.velocityStandardDeviation = 0.01;
        config.simulationConfig.positionStandardDeviation = 0.01;
        config.simulationConfig.momentOfInertia = 0.01;
    }

    // -----------------------------------------------------------------
    // Arm levels (preset targets)
    // -----------------------------------------------------------------
    public enum ArmLevel {

        HOME(0.05),

        L1(0.15),
        L2(0.21),
        L3(0.37),
        UNKNOWN(0.144),
        CoralIntakeLevel(0.2)
        ;
        //L1(17.88 * Constants.INCH_TO_CM),
        // L2(31.12 * Constants.INCH_TO_CM),
        // L3(38.95 *  Constants.INCH_TO_CM),
        // CoralIntakeLevel(0),
        // UNKNOWN(67); //tuff

        public final double height;
        public final Rotation2d angle;
        // public final Pose2d[][] panels;
        public static final double epsilon = 3; // Degre

        ArmLevel(double height) { // Untested
            this.height = height;
            this.angle = Rotation2d.fromRotations(height);
            // this.panels = new Pose2d[6][2];

            // for(int i = 0; i < 6; i++){
            //     System.out.println(this.name());
            //     this.panels[i] = ArmCalculator.coordinateTranslation2d(height, i);
            // }
        }
    }
}