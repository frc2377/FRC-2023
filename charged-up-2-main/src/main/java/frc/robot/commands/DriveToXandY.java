// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToXandY extends CommandBase {
  /** Creates a new DriveToX. */
  private Double m_targetX;
  private Double m_targetY;
  private SwerveSubsystem m_swerve;

  private DriveToXandY(SwerveSubsystem s_swerve) {
    m_swerve = s_swerve;
    addRequirements(m_swerve);
  }

  public DriveToXandY(double s_x, SwerveSubsystem s_swerve) {
    this(s_swerve);
    m_targetX = s_x;   
  }
  
  public DriveToXandY(SwerveSubsystem s_swerve, double s_y) {
    this(s_swerve);
    m_targetY = s_y;
  }

  public DriveToXandY(SwerveSubsystem s_swerve, double s_x, double s_y) {
    this(s_swerve);
    m_targetX = s_x;
    m_targetY = s_y;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_targetY == null) {
      m_swerve.drive(new Translation2d(-1 * (m_swerve.getPose().getX() - m_targetX), 0).
        times(Constants.Swerve.gridMaxSpeed),0,false, false);
    } else if (m_targetX == null) {
      m_swerve.drive(new Translation2d(0, -1 * (m_swerve.getPose().getY() - m_targetY)).
        times(Constants.Swerve.gridMaxSpeed),0,false, false);
    } else {
      m_swerve.drive(new Translation2d(-1 * (m_swerve.getPose().getX() - m_targetX), -1 * (m_swerve.getPose().getY() - m_targetY)).
        times(Constants.Swerve.gridMaxSpeed),0,false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(new Translation2d(0, 0).
      times(Constants.Swerve.gridMaxSpeed),0,false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_targetY == null) {
      return Math.abs(m_swerve.getPose().getX() - m_targetX) < 0.02;
    } else if (m_targetX == null) {
      return Math.abs(m_swerve.getPose().getY() - m_targetY) < 0.02;
    } else {
      return Math.abs(m_swerve.getPose().getX() - m_targetX) < 0.02 && Math.abs(m_swerve.getPose().getY() - m_targetY) < 0.02;
    }
  }
}