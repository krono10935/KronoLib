package frc.robot.subsystems.drivetrain.gyro;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class GyroIONavx implements GyroIO {

    private final AHRS navx;
    private Rotation2d gyroOffset = Rotation2d.kZero;

    public GyroIONavx() {
        navx = new AHRS(NavXComType.kMXP_SPI);
    }

    @Override
    public Optional<GyroPoseOutput> getEstimatedPosition() {
        return Optional.empty();
    }

    @Override
    public Rotation2d update() {
        return Rotation2d.fromDegrees(-navx.getAngle() + gyroOffset.getDegrees());
    }

    @Override
    public void reset(Pose2d pose) {
        navx.reset();
        gyroOffset = pose.getRotation();
    }
}