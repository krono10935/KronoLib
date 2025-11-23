package frc.robot.subsystems.drivetrain.swerve.module;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.FeedForwardConfig;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.PIDConfig;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains.ConstraintType;
import io.github.captainsoccer.basicmotor.gains.FeedForwardsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

public enum SwerveModuleConstants {
    FRONT_LEFT(
            12,  0.23, 10,
            new PIDGains(5, 0, 0, 0, 0, 0),
            new FeedForwardsGains(3),
            1.2,
            11,
            new PIDGains(150, 0, 0, 0, 0, 0.001),
            new FeedForwardsGains(0),
            0,0,
            new Translation2d(0.29, 0.29)),


    FRONT_RIGHT(
            6, 0.015, 4,
            new PIDGains(5, 0, 0, 0, 0, 0),
            new FeedForwardsGains(3),
            1.2,
            14,
            new PIDGains(150, 0, 0, 0, 0, 0.001),
            new FeedForwardsGains(0),
            0,0,
            new Translation2d(0.29, -0.29)),


    BACK_LEFT(
            3, 0.124, 1,
            new PIDGains(5, 0, 0, 0, 0, 0),
            new FeedForwardsGains(3),
            1.2,
            2,
            new PIDGains(150, 0, 0, 0, 0, 0.001),
            new FeedForwardsGains(0),
            0,0,
            new Translation2d(-0.29, 0.29)),


    BACK_RIGHT(
            15, 0.254, 13,
            new PIDGains(5, 0, 0, 0, 0, 0),
            new FeedForwardsGains(3),
            1.2,
            7,
            new PIDGains(150, 0, 0, 0, 0, 0.001),
            new FeedForwardsGains(0),
            0,0,
            new Translation2d(-0.29, -0.29));


    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double DRIVE_GEAR_RATIO = 5.14;
    public static final int DRIVE_CURRENT_LIMIT = 80;

    /**
     * @return Common drive motor config
     */
    private static BasicMotorConfig getDriveMotorCommonConfig() {
        var config = new BasicTalonFXConfig();

        config.motorConfig.gearRatio = DRIVE_GEAR_RATIO;
        config.motorConfig.unitConversion = 2 * Math.PI * WHEEL_RADIUS_METERS;
        config.motorConfig.idleMode = IdleMode.COAST;
        config.motorConfig.motorType = DCMotor.getKrakenX60(1);

        config.currentLimitConfig.statorCurrentLimit = DRIVE_CURRENT_LIMIT;
        config.currentLimitConfig.supplyCurrentLimit = 0;

        return config;
    }

    public static final double STEER_GEAR_RATIO = 12.8;
    public static final int STEER_CURRENT_LIMIT = 35;

    /**
     * @return Common steer motor config
     */
    private static BasicMotorConfig getSteerMotorCommonConfig() {
        var config = new BasicTalonFXConfig();

        config.motorConfig.gearRatio = STEER_GEAR_RATIO;
        config.motorConfig.idleMode = IdleMode.COAST;
        config.motorConfig.motorType = DCMotor.getFalcon500(1);

        config.currentLimitConfig.statorCurrentLimit = STEER_CURRENT_LIMIT;

        config.constraintsConfig.constraintType = ConstraintType.CONTINUOUS;
        config.constraintsConfig.maxValue = 0.5;
        config.constraintsConfig.minValue = -0.5;

        return config;
    }

    public final BasicMotorConfig STEERING_CONFIG;
    public final BasicMotorConfig DRIVING_CONFIG;

    public final Translation2d TRANSLATION;

    public final int CAN_CODER_ID;
    public final double ZERO_OFFSET;

    public final String NAME;

    SwerveModuleConstants(
            int canCoderID,
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

        this.NAME = this.name();
        this.TRANSLATION = location;

        CAN_CODER_ID = canCoderID;
        ZERO_OFFSET = zeroOffset;

        DRIVING_CONFIG = getDriveMotorCommonConfig();

        DRIVING_CONFIG.motorConfig.name = NAME + " Drive Motor";
        DRIVING_CONFIG.motorConfig.id = driveMotorID;

        DRIVING_CONFIG.slot0Config.pidConfig = PIDConfig.fromGains(drivePIDGains);
        DRIVING_CONFIG.slot0Config.feedForwardConfig = FeedForwardConfig.fromFeedForwards(driveFeedForwards);

        DRIVING_CONFIG.simulationConfig.kA = driveKA;
        DRIVING_CONFIG.simulationConfig.kV = driveFeedForwards.getSetpointFeedForward();

        STEERING_CONFIG = getSteerMotorCommonConfig();

        STEERING_CONFIG.motorConfig.name = NAME + " Steer Motor";
        STEERING_CONFIG.motorConfig.id = steerMotorID;
        STEERING_CONFIG.slot0Config.pidConfig = PIDConfig.fromGains(steerPIDGains);
        STEERING_CONFIG.slot0Config.feedForwardConfig = FeedForwardConfig.fromFeedForwards(steerFeedForwards);

        STEERING_CONFIG.simulationConfig.kV = steerKV;
        STEERING_CONFIG.simulationConfig.kA = steerKA;

    }


    /**
     * @return Array of all the translations from the robot to the module;
     */
    public static Translation2d[] getModuleTranslations() {
        ArrayList<Translation2d> translations = new ArrayList<>();

        for (SwerveModuleConstants constants : SwerveModuleConstants.values()) {
            translations.add(constants.TRANSLATION);
        }

        return translations.toArray(new Translation2d[4]);
    }
}
