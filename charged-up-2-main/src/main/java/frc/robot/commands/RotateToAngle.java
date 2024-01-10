// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class RotateToAngle extends CommandBase {

  private double currentAngle;
  private SwerveSubsystem m_swerve;
  private double m_angle;
  private boolean doneSpinning;


  /** Creates a new Autobalance. */
  public RotateToAngle(SwerveSubsystem subsystem, double angle) {
    // Determines if the robot is balanced on the charging station
     m_swerve = subsystem;
     m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  // understand what yaw is relitive to orintation of robot 
  @Override
  public void initialize() {
    doneSpinning = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("DoneSpinning", doneSpinning);
    
    currentAngle = MathUtil.inputModulus(m_swerve.getYaw().getDegrees(), 0, 360);
    //SmartDashboard.putNumber("Yaw2", currentAngle);

    if((currentAngle < (m_angle - .5)) || (currentAngle > (m_angle + .5))){
      if ((currentAngle - 180) > m_angle){
        // Spin robot counterclockwise
        m_swerve.drive(new Translation2d(0,0).times(Constants.Swerve.maxSpeed),
          Constants.Swerve.rotateToAngleMaxSpeed * 
          Math.max((Math.abs(currentAngle - m_angle)), Constants.Swerve.rotateToAngleMinSpeed),
          false, false);
      }
      else {
        // Spin robot clockwise
        m_swerve.drive(new Translation2d(0,0).times(Constants.Swerve.maxSpeed),
          -Constants.Swerve.rotateToAngleMaxSpeed * 
          Math.max((Math.abs(currentAngle - m_angle)), Constants.Swerve.rotateToAngleMinSpeed),
          false, false);
      }
      }
    else{
      m_swerve.stopSwerve();
      doneSpinning = true;  
    } // end of yaw check
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doneSpinning;
  }
}

