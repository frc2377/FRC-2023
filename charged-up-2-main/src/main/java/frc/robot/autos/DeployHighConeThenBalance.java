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
import frc.robot.commands.Autobalance;
import frc.robot.commands.PlaceCargoTopPosition;
import frc.robot.subsystems.ArmSubsystem3;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ModeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployHighConeThenBalance extends SequentialCommandGroup {
  /** Creates a new Auto_23_1_sc. 
 * @param IntakeSubsystem */
  public DeployHighConeThenBalance(SwerveSubsystem s_Swerve, ArmSubsystem3 s_arm3, ExtenderSubsystem s_Extender, ClawSubsystem s_claw,  ModeSubsystem s_mode_subsystem, IntakeSubsystem s_IntakeSubsystem) {
     TrajectoryConfig configslowforward =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecondGoOnScale,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredGoOnScale)
            .setKinematics(Constants.Swerve.swerveKinematics);

     TrajectoryConfig configslowbackward =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecondGoOnScale,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredGoOnScale)
             .setKinematics(Constants.Swerve.swerveKinematics);
        configslowbackward.setReversed(true); // hope this works!!!

    // Move to base of charger
    Trajectory moveForward =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Constants.ShoulderConstants.kGridOffset, 0, new Rotation2d(0)),
             // Move out of community
            List.of(new Translation2d(1.0, 0),new Translation2d(2.0, 0)),
            new Pose2d(3.473 , 0, new Rotation2d(Math.toRadians(0))),

            // End on scale
     //       new Pose2d(4.4, 0, new Rotation2d(Math.toRadians(0))),
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

    
    addCommands(
            // move robot away from grid (drive forward) so that the arm can come out
            new PlaceCargoTopPosition(s_Swerve, s_arm3, s_Extender, s_claw, s_mode_subsystem),
            //new WaitCommand(6),
            new InstantCommand(() -> s_Swerve.resetOdometry(moveForward.getInitialPose())),swerveControllerForward,
            // new DeployIntakeAutoOriginal(s_IntakeSubsystem),
            // new InstantCommand(() -> s_Swerve.resetOdometry(moveForward2.getInitialPose())),swerveControllerForward2,
            new Autobalance(s_Swerve)
           // new RetractIntake(s_IntakeSubsystem)
        ); 
  }
  }


