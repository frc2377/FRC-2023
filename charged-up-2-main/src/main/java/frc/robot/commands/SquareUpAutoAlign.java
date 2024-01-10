// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;


// NOTE:  Consider using this command insoftware/commandbased/convenience-features.html
//command to SquareUpAutoAlignline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/
public class SquareUpAutoAlign extends SequentialCommandGroup {
  private SwerveSubsystem m_swerve;
  /** Creates a new SquareUpAutobalance. */
  public SquareUpAutoAlign(SwerveSubsystem s_swerve) {
    m_swerve = s_swerve;
    //SmartDashboard.putNumber("X", s_swerve.getPose().getX());
    //SmartDashboard.putNumber("Y", s_swerve.getPose().getY());
    addCommands(
      new RotateToAngle(m_swerve, 0),
      new WaitCommand(0.1),
      new AutoAlign(m_swerve),
      new WaitCommand(0.1),
      //Straighten wheels
      m_swerve.runOnce(() -> m_swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.rotateAtSpeed), 
      0.0,false,false)),
      new WaitCommand(0.1)//,
      //m_swerve.runOnce(() -> m_swerve.drive(new Translation2d(-1, 0).
      //  times(.5),0,false,false)),
      //new WaitCommand(5)
         
    
     
    );
  }
}
