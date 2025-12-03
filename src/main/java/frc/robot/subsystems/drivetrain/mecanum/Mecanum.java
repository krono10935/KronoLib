package frc.robot.subsystems.drivetrain.mecanum;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Mecanum extends Drivetrain{

    public Mecanum(BooleanSupplier isRedAlliance){
        super(isRedAlliance);
    }

    @Override
    protected void setChassisSpeed(ChassisSpeeds speeds) {

    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {

    }

    @Override
    public Pose2d getEstimatedPosition() {
        return null;
    }

    @Override
    protected void updateInputs(DrivetrainInputs inputs) {

    }

    @Override
    protected void resetPose(Pose2d newPose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPose'");
    }

    @Override
    public void usePowerAndAngle(double power, edu.wpi.first.math.geometry.Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'usePowerAndAngle'");
    }

    @Override
    public void setDriveVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveVoltage'");
    }

    @Override
    public void setSteerVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSteerVoltage'");
    }

    @Override
    public void spinWithPower(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'spinWithPower'");
    }
}
