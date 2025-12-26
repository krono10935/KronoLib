// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    public Robot()
    {



//        Logger.recordMetadata("ProjectName", "*GENERIC_ROBOT_PROJECT*"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("u/logs/logging")); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
<<<<<<< HEAD
        }else{
=======
        } else {
<<<<<<< HEAD
            // Save outputs to a new log
>>>>>>> origin/main
=======
>>>>>>> parent of 2576df1 (Merge pull request #6 from krono10935/ErrorHandling)
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        RobotContainer.getInstance();
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        MotorManager.getInstance().periodic(); // must run AFTER CommandScheduler
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();
        
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {

            autonomousCommand.cancel();
        }

    }
    
    
    @Override
    public void teleopPeriodic() {
//        RobotContainer.getInstance().drivetrain.drive(new ChassisSpeeds(2,0,2));
    }
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
