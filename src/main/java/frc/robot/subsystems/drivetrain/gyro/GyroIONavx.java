package frc.robot.subsystems.drivetrain.gyro;

import java.util.function.BooleanSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavx implements GyroIO {

    private final AHRS navx;
    private Rotation2d gyroOffset = Rotation2d.kZero;

    public GyroIONavx(BooleanSupplier isRedAlliance) {
        navx = new AHRS(NavXComType.kMXP_SPI);
        
        
    }

    @Override
    public Rotation2d update() {
        return Rotation2d.fromDegrees(-navx.getAngle() + gyroOffset.getDegrees());
    }

    @Override
    public void reset(Rotation2d angle) {
        navx.reset();
        gyroOffset = angle;
    }
}