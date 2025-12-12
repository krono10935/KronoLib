// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.ErrorMessage;
import frc.lib.ReportError;

public class skebob extends SubsystemBase {
  /** Creates a new skebob. */
  CommandXboxController controller = new CommandXboxController(0);
  public skebob() {
    ReportError.getInstance().addError(new ErrorMessage(this, 0,
     "skebob", controller.a(),
      () -> System.out.println("skebob error triggered")));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
