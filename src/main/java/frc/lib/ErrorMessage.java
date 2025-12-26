// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Error message class.
 * @param subsystem the subsystem which is sending the error
 * @param code the error code from the subsystem
 * @param message a message connected to the error
 * @param shouldDisplayError boolean supplier trigger for the error
 * @param onDisplayAlert a runnable to run once the supplier returns true
 * @param alert an Alert object which is turned on once the trigger activates
 */
public record ErrorMessage(
    Subsystem subsystem, 
    int code, 
    String message, 
    BooleanSupplier shouldDisplayError,
    Runnable onTrue,
    Runnable onFalse,
    Alert alert
    ) {
    /**
     * @param subsystem the subsystem which is sending the error
     * @param code the error code from the subsystem
     * @param message a message connected to the error
     * @param shouldDisplayError boolean supplier trigger for the error
     * @param onDisplayAlert a runnable to run once the supplier returns true
     */
    public ErrorMessage(Subsystem subsystem, int code, String message,
     BooleanSupplier shouldDisplayError, Runnable onTrue, Runnable onFalse) {

        this(subsystem, code, message, shouldDisplayError, onTrue, onFalse, new Alert(
            "Error in " + subsystem.getName() + " | Code: " + code + " | Message: " + message,
            Alert.AlertType.kError
        ));

        new Trigger(shouldDisplayError).onChange(new InstantCommand(() -> {
            
            alert.set(shouldDisplayError.getAsBoolean());

            if (shouldDisplayError.getAsBoolean()){
                onTrue.run();
            } else {
                onFalse.run();
            }
        }));
    }
}

