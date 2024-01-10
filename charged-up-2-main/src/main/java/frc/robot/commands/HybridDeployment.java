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
public class HybridDeployment extends SequentialCommandGroup {
  /** Creates a new HybridDeployment. */
  private ArmSubsystem3 m_arm3;
  private ClawSubsystem m_claw;
  public HybridDeployment(ArmSubsystem3 s_arm3, ClawSubsystem s_claw) {
    m_arm3 = s_arm3;
    m_claw = s_claw;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateShoulder(m_arm3, ShoulderConstants.kShoulderHybridPosition),
      new WaitCommand(0.4),
      new ClawOperate(m_claw, false),
      new WaitCommand(0.4),
      new RotateShoulder(m_arm3, ShoulderConstants.kShoulderStowPosition)
    );
  }
}
