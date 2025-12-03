// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.auto.DriveToPoseCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.swerve.Swerve;


public class RobotContainer
{
    private final Drivetrain drivetrain = new Swerve(() -> false);
    public RobotContainer()
    {
        configureBindings();
    }
    
    
    private void configureBindings() {
//        drivetrain.setDefaultCommand(new DriveToPoseCommand(drivetrain, () -> new Pose2d()));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("Hi");
    }


}
