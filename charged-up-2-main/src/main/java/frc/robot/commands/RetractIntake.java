// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractIntake extends SequentialCommandGroup {
  /** Creates a new RetractIntake. */
  public RetractIntake(IntakeSubsystem s_IntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(s_IntakeSubsystem);
    addCommands(
      Commands.runOnce(() -> s_IntakeSubsystem.CloseIntake()), 
      new WaitCommand(.6),
      Commands.runOnce(() -> s_IntakeSubsystem.setWheelsMostlyOff())
    );
  }
}
