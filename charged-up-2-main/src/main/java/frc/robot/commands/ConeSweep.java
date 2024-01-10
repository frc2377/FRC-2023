// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//  Command to sweep a loose cone or cube out of the robot
public class ConeSweep extends SequentialCommandGroup {

  private final ArmSubsystem3 m_arm;
  private final ClawSubsystem m_claw;

  /** Creates a new ConeSweep. */
  public ConeSweep(ArmSubsystem3 s_arm, ClawSubsystem s_claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_arm = s_arm;
    m_claw = s_claw;

    addCommands(
      new ClawOperate(m_claw, false),
      new WaitCommand(.2),
      new RotateShoulder(m_arm, ShoulderConstants.kShoulderSweepPosition),
      new WaitCommand(.4),
      new ClawOperate(m_claw, true),
      new WaitCommand(.2),
      new RotateShoulder(m_arm, ShoulderConstants.kShoulderHybridPosition),
      new WaitCommand(0.4),
      new ClawOperate(m_claw, false),
      new WaitCommand(0.4),
      new RotateShoulder(m_arm, ShoulderConstants.kShoulderStowPosition)
    );
  }
}
