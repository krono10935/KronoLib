// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.leds.FlashingColorPreset;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedPreset;

import java.util.function.Supplier;


public class RobotContainer
{
    private static  RobotContainer instance;
    private final LedManager ledManager;

    private RobotContainer()
    {
        Supplier<LedPreset> sup = ()-> new FlashingColorPreset(new Color(1,1,1),1,2);

        ledManager = new LedManager(sup);

        configureBindings();
    }

    public static RobotContainer getInstance() {
        if(instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private void configureBindings() {}

    public LedManager getLedManager(){
        return this.ledManager;
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
