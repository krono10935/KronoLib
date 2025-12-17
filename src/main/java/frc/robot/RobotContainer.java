// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.Vision.VisionConsumer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.swerve.Swerve;



public class RobotContainer
{
    private static Vision vision;
    private static Drivetrain drivetrain;
    public RobotContainer()
    {
        drivetrain = new Swerve(() -> false);
        vision = new Vision(drivetrain::addVisionMeasurement, () -> new Pose2d());
        configureBindings();

    }
    
    
    private void configureBindings() {}
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
