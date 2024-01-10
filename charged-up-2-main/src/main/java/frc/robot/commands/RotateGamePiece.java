// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateGamePiece extends SequentialCommandGroup {
  private final ClawSubsystem m_claw;
  private final ArmSubsystem3 m_ArmSubsystem3;

  /** Creates a new RotateGamePiece. */
  public RotateGamePiece(ClawSubsystem s_claw, ArmSubsystem3 s_arm3) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_claw = s_claw;
    m_ArmSubsystem3 = s_arm3;
    addCommands(
      new RotateShoulder(m_ArmSubsystem3, ShoulderConstants.kShoulderHybridPosition),  // rotate to hybrid positio        
      Commands.waitSeconds(.2),
      new TurnWrist(m_claw),
      Commands.waitSeconds(1.2),
      new RotateShoulder(m_ArmSubsystem3, ShoulderConstants.kShoulderStowPosition)  // rotate to stow       
    );
   
  }
}
