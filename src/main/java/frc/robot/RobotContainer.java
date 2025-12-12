// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    CommandXboxController cmd;

    private static RobotContainer instance;
    public final Drivetrain drivetrain;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {
        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage);
        cmd = new CommandXboxController(0);
        cmd.b().whileTrue(new RunCommand(drivetrain::stop));

        drivetrain.setDefaultCommand(new DriveCommand(drivetrain,cmd));

        configureBindings();
    }
    
    
    private void configureBindings() {}
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
