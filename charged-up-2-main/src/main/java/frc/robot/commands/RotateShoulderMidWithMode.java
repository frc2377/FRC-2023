// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.util.DeployMode;

public class RotateShoulderMidWithMode extends CommandBase {
  /** Creates a new RotateShoulderMidWithMode. */
  private final ArmSubsystem3 m_shoulder;
  private final DeployMode m_DeployMode;
  private final double m_SetPosition_Cube = Constants.ShoulderConstants.kShoulderCubeMiddlePlacePosition;
  private final double m_SetPosition_Cone = Constants.ShoulderConstants.kShoulderConeMiddlePosition;
  public RotateShoulderMidWithMode( ArmSubsystem3 shoulder,DeployMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = shoulder;
    m_DeployMode = mode; 
    addRequirements(m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_DeployMode.getMode())
        m_shoulder.setPositionDegrees(m_SetPosition_Cone);
    else  
        m_shoulder.setPositionDegrees(m_SetPosition_Cube);    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_DeployMode.getMode()){
      SmartDashboard.putNumber("Requested Position Rotations", m_shoulder.degreesToRotations(m_SetPosition_Cone));
      SmartDashboard.putNumber("Encoder position rotations", m_shoulder.getEncoderPosition());
    }
    else {
       SmartDashboard.putNumber("Requested Position Rotations", m_shoulder.degreesToRotations(m_SetPosition_Cube));
       SmartDashboard.putNumber("Encoder position rotations", m_shoulder.getEncoderPosition());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double setRotations = 0.0;
    if (m_DeployMode.getMode()){
      setRotations = m_shoulder.degreesToRotations(m_SetPosition_Cone);
    }
    else{
      setRotations = m_shoulder.degreesToRotations(m_SetPosition_Cube);
    }
    return Math.abs(setRotations - m_shoulder.getEncoderPosition()) < 0.5;    
  }
}
