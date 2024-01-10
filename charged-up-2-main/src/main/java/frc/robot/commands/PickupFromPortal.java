// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ArmSubsystem3;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupFromPortal extends SequentialCommandGroup {
  private ArmSubsystem3 m_arm3;
  private ClawSubsystem m_claw;
  private ExtenderSubsystem m_extender;
  /** Creates a new PickUpFromPortal. */
  public PickupFromPortal(ArmSubsystem3 s_arm3, ClawSubsystem s_claw, ExtenderSubsystem s_extender) {
    m_arm3 = s_arm3;
    m_claw = s_claw;
    m_extender = s_extender;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawOperate(m_claw, false), // open claw while in robot
      new RotateShoulder(m_arm3,ShoulderConstants.kShoulderPortalPosition), // rotate to portal position
      new WaitCommand(1),
      m_extender.runOnce(() -> m_extender.setPositionInches(ExtenderConstants.kExtenderMiddlePosition)),
      new ClawOperate(m_claw, true),
      new WaitCommand(.2),
      new RotateShoulder(m_arm3, ShoulderConstants.kShoulderStowPosition), // put in robot
      m_extender.runOnce(() -> m_extender.setPositionInches(ExtenderConstants.kExtenderRetractedPosition)),
      new TurnWrist(m_claw)
    );
  }
}
