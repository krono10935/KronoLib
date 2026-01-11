// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represent an Error message and the relevant action connected to it
 */
public class ErrorMessage{

    /**
     * Store errors
     */
    private static record ErrorMessageRecord(Alert alert, BooleanSupplier shouldDisplayError, Consumer<BooleanSupplier> callback) {
        /**
         * Run a specfic error record
         */
        public void Run(){
            if (shouldDisplayError.getAsBoolean()){
                if (!alert.get()){
                    alert.set(true);
                    callback.accept(shouldDisplayError);
                }
            }
            if (!shouldDisplayError.getAsBoolean()){
                if (alert.get()){
                    alert.set(false);
                    callback.accept(shouldDisplayError);
                }
            }
        }  
    }

    /**
     * A list of error storage
     */
    private static List<ErrorMessageRecord> errors;

    /**
     * @param subsystem the subsystem which is sending the error
     * @param message a message connected to the error
     * @param shouldDisplayError boolean supplier trigger for the error
     * @param onTrue a runnable to run once the supplier returns true
     * @param onFalse a runnable to run once the supplier returns true
     */
    public static void create(Subsystem subsystem, String message,
     BooleanSupplier shouldDisplayError, Runnable onTrue, Runnable onFalse) {

        @SuppressWarnings("resource")
        Alert alert = new Alert(subsystem.getName(),
                "Message: " + message,
                Alert.AlertType.kError
        );

        Consumer<BooleanSupplier> callback = (BooleanSupplier shouldDisplayErrorCallback) -> {
            if (shouldDisplayErrorCallback.getAsBoolean()){
                onTrue.run();
            }
            if (!shouldDisplayErrorCallback.getAsBoolean()){
                onFalse.run();
            }
        };

        errors.add(new ErrorMessageRecord(alert, shouldDisplayError, callback));
    }

    /**
     * Create a error message without a runnable for onTrue and onFalse
     * @param subsystem The subsystem on which the error is being run on
     * @param message The message that will be displayed
     * @param shouldDisplayError BooleanSupplier for if to display error
     */
    public static void create(Subsystem subsystem, String message,
     BooleanSupplier shouldDisplayError) {
        create(subsystem, message, shouldDisplayError, () -> {}, () -> {});
    }

    /**
     * Run each error
     */
    public static void UpdateErrors(){
        errors.forEach((error) -> error.Run());
    }
}

