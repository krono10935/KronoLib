// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public record ErrorMessage(
    Subsystem subsystem, 
    int code, 
    String message, 
    BooleanSupplier hasError, 
    Alert alert
    ) {
    public ErrorMessage(Subsystem subsystem, int code, String message, BooleanSupplier hasError) {
        this(subsystem, code, message, hasError, new Alert(
            "Error in " + subsystem.getName() + " | Code: " + code + " | Message: " + message,
            Alert.AlertType.kError
        ));
    }

    public void update(){
        alert.set(hasError.getAsBoolean());
    }
}

