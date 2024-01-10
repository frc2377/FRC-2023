// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PovMoveLeft extends SequentialCommandGroup {
  /** Creates a new PovMoveLeft. */
  public PovMoveLeft(SwerveSubsystem m_SwerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_SwerveSubsystem.runOnce(() -> m_SwerveSubsystem.drive(new Translation2d(0,0).
      times(0),.45,false,false)),
      //new WaitCommand(.01),
      m_SwerveSubsystem.runOnce(() -> m_SwerveSubsystem.drive(new Translation2d(0, Swerve.kPovMovementDistance).
      times(Constants.Swerve.kPovMovementSpeed ),0,true,false)),
      new WaitCommand(100)
    );
  }
}