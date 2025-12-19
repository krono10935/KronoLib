// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.auto.DriveToPoseCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.swerve.Swerve;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class RobotContainer
{

    private final Drivetrain drivetrain = new Swerve(() -> false);
    public RobotContainer()
    {
        configureBindings();
    }
    
    
    private void configureBindings(){
//        drivetrain.setDefaultCommand(new DriveToPoseCommand(drivetrain, () -> new Pose2d()));






    }

    public PathPlannerAuto createAutoFromPaths(PathPlannerPath[] paths){
        return new PathPlannerAuto(createSequenceWithDriveCommand(paths));
    }

    public Command createSequenceWithDriveCommand(PathPlannerPath[] paths) {

        SequentialCommandGroup auto = new SequentialCommandGroup();

        for (PathPlannerPath p: paths){
            Rotation2d finalRotation = p.getAllPathPoints().getLast().rotationTarget.rotation();
            Translation2d finalTranslation = p.getAllPathPoints().getLast().position;
            Pose2d finalPose = new Pose2d(finalTranslation.getX(), finalTranslation.getY(), finalRotation);



            Logger.recordOutput("auto/isCloseEnough" + p.name, (BooleanSupplier) () -> drivetrain.getEstimatedPosition().getTranslation().getDistance(finalTranslation)<
                    DrivetrainConstants.DISTANCE_TO_STOP_PP);
            Logger.recordOutput("auto/isCloseEnoughDis" + p.name, (DoubleSupplier) ()-> drivetrain.getEstimatedPosition().getTranslation().getDistance(finalTranslation));
            Logger.recordOutput("auto/finalTranslation" + p.name, finalPose);


            auto.addCommands(AutoBuilder.followPath(p).until(() -> drivetrain.getEstimatedPosition().getTranslation().getDistance(finalTranslation)<
                    DrivetrainConstants.DISTANCE_TO_STOP_PP));
            auto.addCommands((new DriveToPoseCommand(drivetrain, ()-> finalPose)));


        }
        return auto;

    }


    public Command getAutonomousCommand()
    {
        Command auto = null;
        try {
            PathPlannerPath a = PathPlannerPath.fromPathFile("a");
            PathPlannerPath b = PathPlannerPath.fromPathFile("b");
            PathPlannerPath[] paths = {a,b};
            auto = createSequenceWithDriveCommand(paths);

        } catch (IOException | ParseException e){
            e.getStackTrace();
        }

        return auto;
    }


}
