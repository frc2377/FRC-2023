// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ShoulderConstants;

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new ExtenderSubsystem. */
   private final CANSparkMax m_extender;
   private final SparkMaxPIDController m_extenderPID;
   private final RelativeEncoder m_extenderEncoder;
   private double m_setDistanceInRot;
   private static double m_gearRatio = ExtenderConstants.kExtenderGearRatio;
  public ExtenderSubsystem() {
     m_extender = new CANSparkMax(ShoulderConstants.kExtenderSpeedControllerCanID, CANSparkMaxLowLevel.MotorType.kBrushless);
     m_extenderPID = m_extender.getPIDController();
     m_extenderEncoder = m_extender.getEncoder();
     m_extender.restoreFactoryDefaults();
     m_extenderEncoder.setPosition(0);// assumes always in 

     m_extender.setInverted(ExtenderConstants.kExtenderInvert);
     m_extender.setIdleMode(ExtenderConstants.kExtenderNeutralMode);    
     m_extenderPID.setP(ExtenderConstants.kExtender_P);
     m_extenderPID.setI(ExtenderConstants.kExtender_I);
     m_extenderPID.setD(ExtenderConstants.kExtender_D);
     m_extenderPID.setFF(ExtenderConstants.kExtender_FF);
     m_extenderPID.setOutputRange(
           ExtenderConstants.kExtender_MinOutput,
           ExtenderConstants.kExtender_MaxOutput);
  
     //Set soft limits
    m_extender.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_extender.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ExtenderConstants.kExtender_FwdSoftLimit);
    m_extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ExtenderConstants.kExtender_RevSoftLimit);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Encoder", m_extenderEncoder.getPosition());
  }
  //used to set the motors rotations given a distance in inches
  public void setPositionInches(double in) {
    double distance_in_rotations = inToRotations(in);
    m_setDistanceInRot = distance_in_rotations;
    SmartDashboard.putNumber("SetExtenderRotations", distance_in_rotations);
    m_extenderPID.setReference(m_setDistanceInRot, CANSparkMax.ControlType.kPosition);
  }

  public double inToRotations(double s_in) {
    return (m_gearRatio * 3 * s_in);  /* distance in cm per rotation  one gear rotions is rougly equal to 1 cm*/
  }



}
