// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    
    /**
     * @return Get the new gyro angle
     */
    Rotation2d update();

    /**
     * Reset the gyro angle to another angle
     */
    void reset(Rotation2d angle);
}
