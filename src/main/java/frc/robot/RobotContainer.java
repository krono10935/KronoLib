// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.DriveToPose;
import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.swerve.Swerve;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer
{

    private final LoggedDashboardChooser<Command> chooser;
    private final Drivetrain drivetrain;
    public RobotContainer()
    {
        drivetrain  = new Swerve(() -> false);
        DriveToPose.configure(
                new DriveToPoseConstants(
                        drivetrain::getEstimatedPosition,
                        drivetrain::getChassisSpeeds,
                        drivetrain::drive,
                        true
                )
        );
        configureBindings();
        chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());
    }
    
    
    private void configureBindings(){
//        drivetrain.setDefaultCommand(new DriveToPoseCommand(drivetrain, () -> new Pose2d()));





    }


    public Command getAutonomousCommand()
    {
        return chooser.get();
    }


}
