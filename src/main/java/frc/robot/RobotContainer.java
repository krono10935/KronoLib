// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.conduit.ConduitApi;



public class RobotContainer
{

    private CommandXboxController cmd;

    private static RobotContainer instance;


    private RobotContainer()
    {
        configureBindings();
    }



    public final Drivetrain drivetrain;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }



    private void configureBindings() {
        //        controller.a().whileTrue(
//            swerveSysID.sysIdQuasistaticDrive(Direction.kForward)
//        );
//
//        controller.b().whileTrue(
//            swerveSysID.sysIdQuasistaticDrive(Direction.kReverse)
//        );
//
//        controller.y().whileTrue(
//            swerveSysID.sysIdDynamicDrive(Direction.kForward)
//        );
//
//        controller.x().whileTrue(
//            swerveSysID.sysIdDynamicDrive(Direction.kReverse)
//        );
        cmd.a().onTrue(new InstantCommand(() -> drivetrain.reset(new Pose2d())));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
