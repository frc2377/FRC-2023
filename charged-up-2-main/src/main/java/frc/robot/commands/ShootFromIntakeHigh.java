// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFromIntakeHigh extends SequentialCommandGroup {

  /** Creates a new ShootFromIntake. */
  public ShootFromIntakeHigh(IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intake);
    addCommands(
      new WaitCommand(.75),
     intake.runOnce(() -> intake.ShootFromIntakeHigh()),
     new WaitCommand(.25),
     intake.runOnce(() -> intake.setWheelsOff()));
   }
}