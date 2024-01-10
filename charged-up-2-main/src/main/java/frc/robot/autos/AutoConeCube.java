// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.PlaceCargoTopPosition;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ModeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DeployMode;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ShootFromIntakeHigh;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoConeCube extends SequentialCommandGroup {
  Alliance m_alliance = DriverStation.getAlliance();

    
  /** Decides Distance based on alliance color */
  public AutoConeCube(SwerveSubsystem s_Swerve, ArmSubsystem3 s_arm3, ExtenderSubsystem s_Extender, ClawSubsystem s_claw,  ModeSubsystem s_mode_subsystem, IntakeSubsystem s_IntakeSubsystem) {
    System.out.println("we are in ths constctustor of shanes auto");
    double DistanceCube1Offset = 0;
    double DistanceCubeNode = 0;
    double Distance5 = 0;
    if (m_alliance == Alliance.Red) {
    DistanceCube1Offset = -0.254; //10 inches
    DistanceCubeNode = 0.4064; //16 inches
    Distance5 = -0.15; // 5.9 inches
    } else {
    DistanceCube1Offset = -0.254;
    DistanceCubeNode = -0.4064;
    Distance5 = 0.15;

    }
    
    TrajectoryConfig configslowforward =
    new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecondAutonomous,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredAutonomous)
        .setKinematics(Constants.Swerve.swerveKinematics);

 TrajectoryConfig configslowbackward =
    new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecondAutonomous,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredAutonomous)
         .setKinematics(Constants.Swerve.swerveKinematics);
    configslowbackward.setReversed(true);


Trajectory moveForward =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
          new Pose2d(Constants.ShoulderConstants.kGridOffset, 0, new Rotation2d(0)),
           // Move out of community and pick up cargo
          List.of(new Translation2d(2, Distance5),new Translation2d(4, Distance5),new Translation2d(4.8, DistanceCube1Offset)),
          new Pose2d(5.3, DistanceCube1Offset, new Rotation2d(Math.toRadians(0))),
          configslowforward);
//
    Trajectory moveBackward =
       TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.3, DistanceCube1Offset, new Rotation2d(0)),
            List.of(new Translation2d(4, Distance5),new Translation2d(2, Distance5)),
            new Pose2d(0.08, DistanceCubeNode, new Rotation2d(Math.toRadians(0))),
            configslowbackward);

        
            


         


var thetaController =
    new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0,
        Constants.AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


SwerveControllerCommand swerveControllerForward =
    new SwerveControllerCommand(
        moveForward,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

        SwerveControllerCommand swerveControllerBackward =
        new SwerveControllerCommand(
            moveBackward,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            () -> Rotation2d.fromDegrees(180),
            s_Swerve::setModuleStates,
            s_Swerve);


       
    
    addCommands(
      new PlaceCargoTopPosition(s_Swerve, s_arm3, s_Extender, s_claw, s_mode_subsystem),
      new DeployIntake(s_IntakeSubsystem),
      new InstantCommand(() -> s_Swerve.resetOdometry(moveForward.getInitialPose())),swerveControllerForward,
      new RetractIntake(s_IntakeSubsystem),
      new InstantCommand(() -> s_Swerve.resetOdometry(moveBackward.getInitialPose())),swerveControllerBackward,
      new ShootFromIntakeHigh(s_IntakeSubsystem)
      //new InstantCommand(() -> s_Swerve.resetOdometry(moveForward2.getInitialPose())),swerveControllerForward2
     // new DeployIntake(s_IntakeSubsystem)
     // new InstantCommand(() -> s_Swerve.resetOdometry(moveForward3.getInitialPose())),swerveControllerForward3,
     // new RetractIntake(s_IntakeSubsystem),
     // new DeployIntake2(s_IntakeSubsystem)

    );
  }
}
