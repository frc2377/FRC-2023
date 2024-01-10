// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoDisable extends CommandBase {
  /** Creates a new IntakeAutoDisable. */
IntakeSubsystem m_intake;
int intakeCycle;

  public IntakeAutoDisable(IntakeSubsystem s_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = s_intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intake.getOutputCurrentUpperWheels() > Intake.kCurrentOutputMax){
      intakeCycle++;
    }
    if(intakeCycle > 3){
      m_intake.CloseIntake();
      m_intake.setWheelsOff();
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
