package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SwerveSysID {
   private SysIdRoutine routineDrive;
   private SysIdRoutine routineSteer;
   private SysIdRoutine routineSpin;
   private CommandXboxController controller;
   private Drivetrain drivetrain;

   public static final double VOLT = 2;

   public static final double VOLT_RAMP_RATE = 0.5;

   public static final double TIMEOUT = 10;
   public SwerveSysID(Drivetrain drivetrain, CommandXboxController controller) {
      this.controller = controller;
      this.drivetrain = drivetrain;

      routineSteer = new SysIdRoutine(
              new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(VOLT_RAMP_RATE),
                      Units.Volts.of(VOLT),
                      Units.Seconds.of(TIMEOUT),
                      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
              ),

              new SysIdRoutine.Mechanism(
                      (volt) -> drivetrain.setSteerVoltage(volt.baseUnitMagnitude()),
                      null,
                      drivetrain
              )
      );

      routineDrive = new SysIdRoutine(
              new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(VOLT_RAMP_RATE),
                      Units.Volts.of(VOLT),
                      Units.Seconds.of(TIMEOUT),
                      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
              ),

              new SysIdRoutine.Mechanism(
                      (volt) -> this.driveWithController(volt.baseUnitMagnitude()),
                      null,
                      drivetrain
              )
      );

      routineSpin = new SysIdRoutine(
              new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(VOLT_RAMP_RATE),
                      Units.Volts.of(VOLT),
                      Units.Seconds.of(TIMEOUT),
                      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
              ),
              new SysIdRoutine.Mechanism(
                      (volt) -> this.spin(volt.baseUnitMagnitude()),
                      null,
                      drivetrain
              )
      );
   }

   public void driveWithController(double voltage) {

      Rotation2d angle = new Translation2d(-controller.getLeftX(), -controller.getLeftY()).getAngle().rotateBy(drivetrain.getGyroAngle().unaryMinus());

      drivetrain.usePowerAndAngle(voltage, angle);
   }

   public void spin(double voltage) {
      drivetrain.spinWithPower(voltage);
   }


   public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
      return routineDrive.quasistatic(direction);
   }


   public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
      return routineDrive.dynamic(direction);
   }


   public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
      return routineSteer.quasistatic(direction);
   }


   public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
      return routineSteer.dynamic(direction);
   }

   public Command sysIdQuasistaticSpin(SysIdRoutine.Direction direction) {
      return routineSpin.quasistatic(direction);
   }


   public Command sysIdDynamicSpin(SysIdRoutine.Direction direction) {
      return routineSpin.dynamic(direction);
   }
}
