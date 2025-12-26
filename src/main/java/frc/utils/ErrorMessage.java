// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represent an Error message and the relevant action connected to it
 */
public class ErrorMessage{
    private final Alert alert;
    /**
     * @param subsystem the subsystem which is sending the error
     * @param code the error code from the subsystem
     * @param message a message connected to the error
     * @param shouldDisplayError boolean supplier trigger for the error
     * @param onTrue a runnable to run once the supplier returns true
     * @param onFalse a runnable to run once the supplier returns true
     */
    public ErrorMessage(Subsystem subsystem, int code, String message,
     BooleanSupplier shouldDisplayError, Runnable onTrue, Runnable onFalse) {

        this.alert = new Alert(subsystem.getName(),
                "Error in " + subsystem.getName() + " | Code: " + code + " | Message: " + message,
                Alert.AlertType.kError
        );

        new Trigger(shouldDisplayError).onTrue(
                new InstantCommand(() -> alert.set(true))
                        .andThen(new InstantCommand(onTrue)))
                .onFalse(
                new InstantCommand(() -> alert.set(false))
                        .andThen(new InstantCommand(onFalse)));
    }
}

