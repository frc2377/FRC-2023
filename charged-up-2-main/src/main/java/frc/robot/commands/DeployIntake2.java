// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake2 extends CommandBase {
  /** Creates a new DeployIntake2. */
  int intakeCycle;
  boolean cycleFinished;
  private IntakeSubsystem m_intake;
  public DeployIntake2(IntakeSubsystem s_intake) {
    m_intake = s_intake;
    intakeCycle = 0;
    cycleFinished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_intake);   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_intake.OpenIntake();
      m_intake.setWheelsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //new WaitCommand(0.1);
    if(m_intake.getOutputCurrentUpperWheels() > Intake.kCurrentOutputMax){
      intakeCycle++;
      System.out.println(intakeCycle);
    }
    if(intakeCycle > (Intake.kCycleAmountInSeconds*Intake.kCyclePerSecond)){
      cycleFinished = true;
      intakeCycle = 0; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.CloseIntake();
    m_intake.setWheelsMostlyOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_intake.getIntakeHasCube(); uses sensor
    if (cycleFinished == true){
        cycleFinished = false;
        return true;
    }
    return false;
  }
}
