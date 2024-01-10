// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//comand to rotate shoulder

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem3;


public class RotateShoulder extends CommandBase {
  private final ArmSubsystem3 m_shoulder;
  private final double m_SetPosition_In_Degrees; // degrees

  /** Creates a new RotateShoulder. */
  public RotateShoulder(ArmSubsystem3 shoulder, double SetPositionIndegrees) {
    
    m_shoulder = shoulder;
    m_SetPosition_In_Degrees = SetPositionIndegrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shoulder.setPositionDegrees(m_SetPosition_In_Degrees);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Requested Position Rotations", m_shoulder.degreesToRotations(m_SetPosition_In_Degrees));
    SmartDashboard.putNumber("Encoder position rotations", m_shoulder.getEncoderPosition());    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // convert set requested degrees into rotations
    // double setRotations = m_shoulder.degreesToRotations(m_SetPosition_In_Degrees);
    // System.out.println("we are in isfinished of rotate shoulder");
    // return Math.abs(setRotations - m_shoulder.getEncoderPosition()) < 0.5;    
    return true;
  }
}
