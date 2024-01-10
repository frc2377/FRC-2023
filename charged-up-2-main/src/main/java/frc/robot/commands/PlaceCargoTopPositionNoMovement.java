// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.ModeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DeployMode;


public class PlaceCargoTopPositionNoMovement extends SequentialCommandGroup {
  SwerveSubsystem m_swerve;
  ArmSubsystem3 m_arm3;
  ExtenderSubsystem m_extender;
  ClawSubsystem m_claw;

  /** Creates a new PlaceCargoTopPosition. */                                                                                      
  public PlaceCargoTopPositionNoMovement(SwerveSubsystem s_swerve, ArmSubsystem3 s_arm3, ExtenderSubsystem s_extender, ClawSubsystem s_claw) {
    m_swerve = s_swerve;
    m_arm3 = s_arm3;
    m_extender = s_extender;
    m_claw = s_claw;
    
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
 
    addCommands(
        m_extender.runOnce(() -> m_extender.setPositionInches(ExtenderConstants.kExtenderTopPosition)),
        new WaitCommand(1),
        new ClawOperate(m_claw, false),
        new WaitCommand(.7),
        m_extender.runOnce(() -> m_extender.setPositionInches(ExtenderConstants.kExtenderRetractedPosition)),
        new WaitCommand(.5),
        m_swerve.runOnce(() -> m_swerve.drive(new Translation2d(ShoulderConstants.kGridOffset, 0).
          times(Constants.Swerve.gridMaxSpeed),0,false,false)),
        new WaitCommand(0.6),
        m_swerve.runOnce(() -> m_swerve.drive(new Translation2d(0, 0).
          times(Constants.Swerve.gridMaxSpeed),0,false,false)),
        new WaitCommand(.3),
        new RotateShoulder(m_arm3, ShoulderConstants.kShoulderStowPosition)       
      );
  }
}


