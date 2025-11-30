package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class GyroIOSim implements GyroIO{

    private final Supplier<ChassisSpeeds> speedsSupplier;
    
    private Rotation2d angle = Rotation2d.kZero;

    public GyroIOSim(Supplier<ChassisSpeeds> speedsSupplier, BooleanSupplier isRedAlliance){
        this.speedsSupplier = speedsSupplier;
    }

    @Override
    public Rotation2d update() {
        double omega = speedsSupplier.get().omegaRadiansPerSecond;
        angle = Rotation2d.fromRadians(angle.getRadians() + omega * 0.02);
        return angle;
    }

    @Override
    public void reset(Rotation2d angle) {
        this.angle = angle;
    }

}
