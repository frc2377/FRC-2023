// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ModeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DeployMode;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.PlaceCargoTopPosition;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ShootFromIntakeHigh;

/*
 Cone Cube Auto

Forward Trajectory
1.) (0.08, 0)
2.) (1, 0)
3.) (2, 0)
4.) (3, 0) 
5.) (5.3, -0.36576) Distance2 Assumes the cube is 14.4 inches to the right of the cone target rewq432

Reverse Trajectory
1.) (5.3, -0.36576) Distance2 Assumes the cube is 14.4 inches to the right of the cone t
2.) (3, 0.15) Distance3 veer left (5.9 inches to the left)
3.) (2, 0.15) Distance3 stay straight
4.) (1, 0.15) Distance3 stay straight
5.) (0.08, -0.254) (5.9 inches to zero and then continue to rhe right 10 inches) 
Distance Assumes the cube target is 16 inches to the right from the start up position of the robot
 */


public class AutoConeCubeBlue extends SequentialCommandGroup {
   
  /** Creates a new AutoConeCubeBlue. */
  public AutoConeCubeBlue(SwerveSubsystem s_Swerve, ArmSubsystem3 s_arm3, ExtenderSubsystem s_extender, ClawSubsystem s_claw,  ModeSubsystem s_mode_subsystem, IntakeSubsystem s_Intake) {
    // Add your commands in the addCommands() call, e.g.
    double Distance = 0;
    double Distance2 = 0;
    double Distance3 = 0;
   
    Distance = -0.254;// the offset to get to the endpoint(to get to the cube grid)
    Distance2 = -0.36576;// distance to the right to gather the cube.
    Distance3 = 0.15; // the value to swing out 
   

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
             // Move out of community
            List.of(new Translation2d(1, 0),new Translation2d(2, 0),
            new Translation2d(3, 0)),
            // End on scale
            new Pose2d(5.3, Distance2, new Rotation2d(Math.toRadians(0))),
            configslowforward);

    Trajectory moveBackward =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(5.3, Distance2, new Rotation2d(0)),
            // Move out of community
            List.of(new Translation2d(3, Distance3),
            new Translation2d(2, Distance3),new Translation2d(1, Distance3)),
            // End on scale
            new Pose2d(0.08,/*.24 */ Distance, new Rotation2d(Math.toRadians(0))),
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
                new PIDController(Constants.AutoConstants.kPXController, 0, 0.12),//.12
                new PIDController(Constants.AutoConstants.kPYController, 0, 0.12),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
    
       
          SwerveControllerCommand swerveControllerBackward =
            new SwerveControllerCommand(
               moveBackward,
               s_Swerve::getPose,
               Constants.Swerve.swerveKinematics,
               new PIDController(Constants.AutoConstants.kPXController, 0, 0.12),
               new PIDController(Constants.AutoConstants.kPYController, 0, 0.12),
               thetaController,
               () -> Rotation2d.fromDegrees(180.10),// rotate 180 degrees
               s_Swerve::setModuleStates,
               s_Swerve);
    addCommands( new PlaceCargoTopPosition( s_Swerve,  s_arm3,  s_extender,  s_claw,  s_mode_subsystem),
    new DeployIntake(s_Intake),
    new InstantCommand(() -> s_Swerve.resetOdometry(moveForward.getInitialPose())),swerveControllerForward,
    new RetractIntake(s_Intake),
    new InstantCommand(() -> s_Swerve.resetOdometry(moveBackward.getInitialPose())),swerveControllerBackward,
    new ShootFromIntakeHigh(s_Intake),
    new InstantCommand(() -> s_Swerve.stopSwerve()));
  }
}