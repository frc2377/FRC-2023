// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Retract extends SequentialCommandGroup {
  /** Creates a new Retract. */
  ArmSubsystem3 m_arm3;
  ExtenderSubsystem m_extender;
  ClawSubsystem m_claw;
  public Retract(ArmSubsystem3 s_arm3, ExtenderSubsystem s_extender, ClawSubsystem s_claw) {
    m_arm3 = s_arm3;
    m_extender = s_extender;
    m_claw = s_claw;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_extender.runOnce(() -> m_extender.setPositionInches(ExtenderConstants.kExtenderRetractedPosition)),
      new WaitCommand(0.75),
      new RotateShoulder(m_arm3, ShoulderConstants.kShoulderStowPosition),
      new WaitCommand(0.75),
      new ClawOperate(m_claw, true)
    );
  }
}
