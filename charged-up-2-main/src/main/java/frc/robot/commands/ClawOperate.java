// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
  



public class ClawOperate extends CommandBase {
  /** Creates a new ClawOperate. */
  private final ClawSubsystem m_claw;
  private final boolean m_claw_state; 

  public ClawOperate(ClawSubsystem s_claw, boolean s_open_close) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_claw = s_claw;
    m_claw_state = s_open_close; 
    addRequirements(m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (m_claw_state)
    m_claw.OpenClaw();
   else
    m_claw.CloseClaw();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("we are in the is finished of operate claw");
    return true;
  }
}
