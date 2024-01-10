// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// command to drive a distance

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveDistance extends CommandBase {

  private final SwerveSubsystem m_swerve;
  private final double m_TargetX;
  private final double m_TargetY;
  private boolean m_XFinished;
  private boolean m_YFinished;
  private boolean m_Finished;
  private double m_CurrentY;
  private double m_CurrentX;

  /** Creates a new DriveDistance. */
  public DriveDistance(SwerveSubsystem s_swerve, double s_X, double s_Y) {
    m_swerve = s_swerve;
    m_TargetX = s_X;
    m_TargetY = s_Y;   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_XFinished = false;
    m_YFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    boolean l_xForwards = false;
    boolean l_xBackwards = false;
    boolean l_yForwards = false;
    boolean l_yBackwards = false;
    
    //get current position of the robot
    m_CurrentY = m_swerve.getPose().getY();
    m_CurrentX = m_swerve.getPose().getX();
    

      if(m_CurrentX < m_TargetX) 
      {
        l_xForwards = true;
      }
      else if(m_CurrentX > m_TargetX)
        {
          l_xBackwards = true;
        }
      
        if(m_CurrentY < m_TargetY) 
        {
        l_yForwards = true;
        }
        else if(m_CurrentY > m_TargetY)
        {
          l_yBackwards = true;
        }

      if(m_CurrentX == m_TargetX)
      {
        m_XFinished = true;
      }
      if(m_CurrentY == m_TargetY)
      {
        m_YFinished = true;
      }

     
    // move both x and y
   if (!m_XFinished && !m_YFinished)
   {
      if (l_xForwards && l_yForwards)
      {
      m_swerve.drive(new Translation2d(0.5, 0.5).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
      true);
      } else if (l_xForwards && l_yBackwards) 
      {
        m_swerve.drive(new Translation2d(0.5, -0.5).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
      true);
      } else if (l_xBackwards && l_yForwards)
      {
        m_swerve.drive(new Translation2d(-0.5, 0.5).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
      true);
      } else if (l_xBackwards && l_yBackwards)
      {
        m_swerve.drive(new Translation2d(-0.5, -0.5).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
      true);
      }
   }   
   else if(!m_XFinished && m_YFinished) // move x but not y
   {
    if (l_xForwards)
    {
      m_swerve.drive(new Translation2d(0.5, 0).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
          true);
    } else if (l_xBackwards)
    {
      m_swerve.drive(new Translation2d(-0.5, 0).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
      true);
    }
   } 
   else if(m_XFinished && !m_YFinished) // move y but not x
   {
    if (l_yForwards)
    {
    m_swerve.drive(new Translation2d(0, 0.5).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
    true);
    } else if (l_yBackwards)
    {
      m_swerve.drive(new Translation2d(0, -0.5).times(Constants.Swerve.autoBalanceMaxSpeed), 0, false,
    true);
    }
   } else 
   {
    m_Finished = true;
   }
   
   
   // get current position and update variables for current position
   // compare current position to goal position and update m_Finished variables

  
      
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Finished;
  }

}







