// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DeployMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployePrePlaceMed extends SequentialCommandGroup {
  SwerveSubsystem m_swerve;
  ArmSubsystem3 m_arm3;
  ExtenderSubsystem m_extender;
  ClawSubsystem m_claw;
  DeployMode m_mode;

  /** Creates a new PlaceCargoTopPosition. */
  public DeployePrePlaceMed(SwerveSubsystem s_swerve, ArmSubsystem3 s_arm3, ExtenderSubsystem s_extender, ClawSubsystem s_claw, DeployMode s_mode) {
    m_swerve = s_swerve;
    m_arm3 = s_arm3;
    m_extender = s_extender;
    m_claw = s_claw;
    m_mode = s_mode;
    m_mode.toggleMode();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      m_arm3.runOnce(() -> m_arm3.setPositionDegrees(ShoulderConstants.kShoulderConeMiddlePosition)),
      new WaitCommand(1),
      m_swerve.runOnce(() -> m_swerve.drive(new Translation2d((-ShoulderConstants.kGridOffset), 0).
        times(Constants.Swerve.gridMaxSpeed),0,false,false)),
      //Adding .1 sec to forward motion to ensure we get to grid
      new WaitCommand(.7),
      m_swerve.runOnce(() -> m_swerve.drive(new Translation2d(0, 0).
        times(Constants.Swerve.gridMaxSpeed),0,false,false)),
      // conditionalCommand(m_extender.runOnce(() -> m_extender.setPositionInches(ExtenderConstants.kExtenderMiddlePosition))),
      // conditionalCommand(new WaitCommand(.5)),
      new ClawOperate(m_claw, false),
      // conditionalCommand(new WaitCommand(.5)),
      // m_extender.runOnce(() -> m_extender.setPositionInches(ExtenderConstants.kExtenderRetractedPosition)),
      new WaitCommand(.5),
      m_swerve.runOnce(() -> m_swerve.drive(new Translation2d(ShoulderConstants.kGridOffset, 0).
        times(Constants.Swerve.gridMaxSpeed),0,false,false)),
      new WaitCommand(.6),
      m_swerve.runOnce(() -> m_swerve.drive(new Translation2d(0, 0).
        times(Constants.Swerve.gridMaxSpeed),0,false,false)),
      new RotateShoulder(m_arm3, ShoulderConstants.kShoulderStowPosition)
    );
  }
}
