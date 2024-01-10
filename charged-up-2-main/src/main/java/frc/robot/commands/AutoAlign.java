// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.LimeLightConstants;
import edu.wpi.first.math.geometry.Translation2d;
// command to auto align
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  //private final LEDSubsystem m_led;
  private final SwerveSubsystem m_swerve;
  //private boolean m_check;
 
  public AutoAlign(SwerveSubsystem s_swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = s_swerve;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // m_check = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {    
 
    updateDashboard();

    //Square up robot
    new RotateToAngle(m_swerve, 0);

    //Align to target
    if (GridLimelightSubsystem.hasTarget()) {
      double offsetMeters = GridLimelightSubsystem.leftRightDistanceMeters();
      m_swerve.drive(new Translation2d(0, offsetMeters).
      times(Constants.Swerve.autoAlignMaxSpeed),0,false,
          false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stopSwerve();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Finish if the distance is less than the threshold
    return Math.abs(GridLimelightSubsystem.leftRightDistanceMeters()) < LimeLightConstants.kAlignThreshold;
  }

  private void updateDashboard() {
    // SmartDashboard.putNumber("Limelight tx", LimelightUtil.getTx());
    SmartDashboard.putBoolean("GridLimelight has target", GridLimelightSubsystem.hasTarget());
    // SmartDashboard.putNumber("Limelight ty", LimelightUtil.getTy());
    // SmartDashboard.putNumber("GridLimelight Distance", GridLimelightSubsystem.getDistanceMeters());
    SmartDashboard.putNumber("Left Right Offset", GridLimelightSubsystem.leftRightDistanceMeters());
  }

}
