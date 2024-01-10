// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForward extends ParallelCommandGroup {
  /** Creates a new DriveForward. */
  public DriveForward(IntakeSubsystem s_IntakeSubsystem, SwerveSubsystem s_Swerve) {
    TrajectoryConfig configslowforward =
         new TrajectoryConfig(
                 Constants.AutoConstants.kMaxSpeedMetersPerSecondGoOnScale,
                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredGoOnScale)
             .setKinematics(Constants.Swerve.swerveKinematics);

             Trajectory moveForward =
         TrajectoryGenerator.generateTrajectory(
             // Start at the origin facing the +X direction
             new Pose2d(Constants.ShoulderConstants.kGridOffset, 0, new Rotation2d(0)),
              // Move out of community
             List.of(new Translation2d(3, 0),new Translation2d(4, 0)),
             // move to cube 
             new Pose2d(5.3, 0, new Rotation2d(Math.toRadians(0))),
             configslowforward);

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
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(moveForward.getInitialPose())),swerveControllerForward
    );
  }
}
