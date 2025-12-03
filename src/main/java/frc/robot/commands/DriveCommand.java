// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

public class DriveCommand extends Command {
  private final frc.robot.subsystems.drivetrain.Drivetrain drivetrain;
  private final CommandXboxController controller;

  private static final double MAX_LINEAR_SPEED = DrivetrainConstants.MAX_LINEAR_SPEED;
  private static final double MIN_LINEAR_SPEED = DrivetrainConstants.MIN_LINEAR_SPEED;
  private static final double MAX_ANGULAR_SPEED = DrivetrainConstants.MAX_ANGULAR_SPEED;
  private static final double MIN_ANGULAR_SPEED = DrivetrainConstants.MIN_ANGULAR_SPEED;

  private static final BooleanSupplier isRedAlliance = () -> false;
  

  private static final double DEADBAND = 0.15;

  public DriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
  }


  @Override
  public void execute() {
    double speed = lerp((1 - controller.getRightTriggerAxis()));
    double angularSpeed = angularLerp(1 - controller.getRightTriggerAxis());

    double xSpeed = deadband(-controller.getLeftY()) * speed * 0.6;
    double ySpeed = deadband(-controller.getLeftX()) * speed * 0.6 ;
    double thetaSpeed = deadband(-controller.getRightX()) * angularSpeed * 0.6;

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
     DriverStation.getAlliance().get() == Alliance.Red? 
      drivetrain.getGyroAngle():
       Rotation2d.fromDegrees(drivetrain.getGyroAngle().getDegrees() + 180)));
  }

  /**
   * Calculate the linear lerp of a {@code value}
   * @param value
   * @return the linear lerp of a {@code value}
   */
  private static double lerp(double value){
    return MIN_LINEAR_SPEED + (MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) * value;
  }

  /**
   * Calculate the angular lerp of a {@code value}
   * @param value
   * @return the angular lerp of a {@code value}
   */
  private static double angularLerp(double value){
    return MIN_ANGULAR_SPEED + (MAX_ANGULAR_SPEED - MIN_ANGULAR_SPEED) * value;
  }

  /**
   * Calculate if the value is passed the deadband value
   * @param value
   * @return 0 if the absolute {@code value} is less than deadband, otherwise {@code value}
   */
  private static double deadband(double value){
    if (Math.abs(value) < DEADBAND){
      return 0;
    }

    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds());
  }
}
