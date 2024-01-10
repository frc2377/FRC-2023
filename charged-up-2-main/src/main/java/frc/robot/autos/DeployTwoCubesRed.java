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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.PlaceCargoTopPosition;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ShootFromIntakeHigh;
import frc.robot.commands.ShootFromIntakeMid;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ModeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DeployMode;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployTwoCubesRed extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public DeployTwoCubesRed(SwerveSubsystem s_Swerve,IntakeSubsystem s_Intake, ArmSubsystem3 s_arm3, ExtenderSubsystem s_extender, ClawSubsystem s_claw, ModeSubsystem s_mode_subsystem) {
    s_mode_subsystem.toggleMode();


    double Distance = 0;
    double Distance2 = 0;
    double Distance3 = 0;
  //  double Distance4 = 0;

     Distance = -0.15;
     Distance2 = - 0.3;
     Distance3 = -0.2;

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
            List.of(new Translation2d(1, Distance2),new Translation2d(2, Distance2),
             new Translation2d(3, Distance2)),
            // End on scale
            new Pose2d(5.3, Distance, new Rotation2d(Math.toRadians(0))),
            configslowforward);

    Trajectory moveBackward =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(5.3, Distance, new Rotation2d(0)),
            // Move out of community
            List.of(new Translation2d(3, Distance2),new Translation2d(2, Distance2),
            new Translation2d(1, Distance2)),
            // End on scale
            new Pose2d(0.08,/*.24 */ Distance3, new Rotation2d(Math.toRadians(0))),
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
           () -> Rotation2d.fromDegrees(179.99),//  this is blue value 
           s_Swerve::setModuleStates,
           s_Swerve);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PlaceCargoTopPosition( s_Swerve,  s_arm3,  s_extender,  s_claw,  s_mode_subsystem),
      new DeployIntake(s_Intake),
      new InstantCommand(() -> s_Swerve.resetOdometry(moveForward.getInitialPose())),swerveControllerForward,
      new RetractIntake(s_Intake),
      new InstantCommand(() -> s_Swerve.resetOdometry(moveBackward.getInitialPose())),swerveControllerBackward,
      new ShootFromIntakeMid(s_Intake),
      new InstantCommand(() -> s_Swerve.stopSwerve())
    );
    s_mode_subsystem.toggleMode();
  }
}
