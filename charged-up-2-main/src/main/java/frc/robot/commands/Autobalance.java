// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ChargingStationConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// command to auto balance

public class Autobalance extends CommandBase {

  private boolean pitch_balanced;
  private double roll;
  private SwerveSubsystem m_swerve;
  private int balanceCount;
  private double balanceSpeed; //decrementing max speed value
  private boolean lastRoll;  //comparitors to determine +/- roll change
  private boolean currentRoll;      

  /** Creates a new Autobalance. */
  public Autobalance(SwerveSubsystem subsystem) {
    // Determines if the robot is balanced on the charging station
    m_swerve = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  // understand what pitch is relitive to orintation of robot
  @Override
  public void initialize() {
    pitch_balanced = false;
    balanceCount = 0;  
    lastRoll = true;
    currentRoll = true;
    balanceSpeed = Constants.Swerve.autoBalanceMaxSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    roll = m_swerve.getRoll();
    SmartDashboard.putNumber("Roll", roll);
    
    if (roll > ChargingStationConstants.kAngleVariance) {
      currentRoll = true;  //true for positive roll
      if(currentRoll != lastRoll){
        lastRoll = currentRoll;
        balanceSpeed = balanceSpeed * 0.4; //reduce speed when sign (+/-) of roll changes
      }
      // Move robot forward
      m_swerve.drive(new Translation2d(0.15, 0).times(balanceSpeed), 0, false,
          false);
    } else if (roll < -ChargingStationConstants.kAngleVariance) {
      currentRoll = false; //false for negative roll
      if(currentRoll != lastRoll){
        lastRoll = currentRoll;
        balanceSpeed = balanceSpeed * 0.4;  //reduce speed when sign (+/-) of roll changes
      }
      // Move robot backward
      m_swerve.drive(new Translation2d(-0.15, 0).times(balanceSpeed), 0, false,
          false);
    } else {
      balanceCount++;
      m_swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.autoBalanceMaxSpeed), 
           0, false, false);
      if(balanceCount > 40)
      {
      // lock wheels
      pitch_balanced = true;
      m_swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.autoBalanceMaxSpeed), 
           0.25, false, false);
      } 
    } // end of roll check

  }
   
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
   
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pitch_balanced;
  }
}
