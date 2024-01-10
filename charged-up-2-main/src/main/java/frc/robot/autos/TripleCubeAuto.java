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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.HybridDeployment;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ShootFromIntakeHigh;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TripleCubeAuto extends SequentialCommandGroup {
  /** Creates a new TripleCubeAuto. */
  public TripleCubeAuto(SwerveSubsystem s_Swerve, IntakeSubsystem s_Intake, ArmSubsystem3 s_Arm3, ClawSubsystem s_Claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Alliance m_alliance = DriverStation.getAlliance();

    double Distance1 = 0;
    double Distance2 = 0;
    double Distance3 = 0;

    if (m_alliance == Alliance.Red) {
     Distance1 = -0.3;
     Distance2 = -0.15;
     Distance3 = -1.419;
    } else {
    // Distance = 0.35;
     Distance1 = 0.3;
     Distance2 = 0.15;
     Distance3 = 1.419;
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

    Trajectory moveForwardStep1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Constants.ShoulderConstants.kGridOffset, 0, new Rotation2d(0)),
             // Move out of community
            List.of(new Translation2d(1, Distance1), new Translation2d(2, Distance1), new Translation2d(3, Distance1)),
            // End on scale
            new Pose2d(5.3, Distance2, new Rotation2d(Math.toRadians(0))),
            configslowforward);

    Trajectory moveBackwardStep2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(5.3, Distance2, new Rotation2d(0)),
            // Move out of community
            List.of(new Translation2d(3, Distance1), new Translation2d(2, Distance1), new Translation2d(1, Distance1)),
            // End on scale
            new Pose2d(0.08, Distance2, new Rotation2d(Math.toRadians(0))),
            configslowbackward);

    Trajectory moveForwardStep3 =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Constants.ShoulderConstants.kGridOffset, Distance2, new Rotation2d(180)),
                List.of(new Translation2d(Distance1, 1), new Translation2d(Distance1, 2), new Translation2d(Distance1, 3)),
                new Pose2d(Distance3, 5.3, new Rotation2d(Math.toRadians(0))),
                configslowbackward);
    
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerForwardStep1 =
        new SwerveControllerCommand(
            moveForwardStep1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0.12),//.12
            new PIDController(Constants.AutoConstants.kPYController, 0, 0.12),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerBackwardStep2 =
        new SwerveControllerCommand(
            moveBackwardStep2,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0.12),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0.12),
            thetaController,
            () -> Rotation2d.fromDegrees(180),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerForwardStep3 =
            new SwerveControllerCommand(
                moveForwardStep3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0.12),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0.12),
                thetaController,
                () -> Rotation2d.fromDegrees(180),
                s_Swerve::setModuleStates,
                s_Swerve);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HybridDeployment(s_Arm3, s_Claw),
      new DeployIntake(s_Intake),
      new InstantCommand(() -> s_Swerve.resetOdometry(moveForwardStep1.getInitialPose())), swerveControllerForwardStep1,
      new RetractIntake(s_Intake),
      new InstantCommand(() -> s_Swerve.resetOdometry(moveBackwardStep2.getInitialPose())), swerveControllerBackwardStep2,
      new ShootFromIntakeHigh(s_Intake),
      new InstantCommand(() -> s_Swerve.resetOdometry(moveForwardStep3.getInitialPose())), swerveControllerForwardStep3,
      new DeployIntake(s_Intake),
      new InstantCommand(() -> s_Swerve.stopSwerve())
    );
  }
}
