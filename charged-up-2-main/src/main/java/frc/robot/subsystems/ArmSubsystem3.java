// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.util.DeployMode;

public class ArmSubsystem3 extends SubsystemBase {
  private static double m_gearRatio = ShoulderConstants.kShoulderAngleGearRatio;
  private CANSparkMax m_ShouldMotor;
  private SparkMaxPIDController m_pidShoulderController;
  private RelativeEncoder m_RelativeEncoder;
  private CANCoder m_CANCoder;
  private double m_StartupOffSet;
  private DeployMode m_DeployMode;

  

  /** Creates a new ArmSubsystem3. */
  public ArmSubsystem3(DeployMode p_deploymode) {
    m_DeployMode = p_deploymode;
    m_ShouldMotor = new CANSparkMax(ShoulderConstants.kShoulderSpeedControllerCanID, MotorType.kBrushless);
    m_ShouldMotor.restoreFactoryDefaults();
    m_ShouldMotor.setInverted(ShoulderConstants.kShoulderInvert); // inverted so that number go up from back of bot to
                                                                  // the front of the bot (limit switch are also
                                                                  // supporting this configuration)


    // set soft limits
    m_ShouldMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_ShouldMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_ShouldMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) degreesToRotations(ShoulderConstants.kShoulder_RevSoftLimit));
    m_ShouldMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) degreesToRotations(ShoulderConstants.kShoulder_FwdSoftLimit));

    m_ShouldMotor.setIdleMode(ShoulderConstants.kShoulderNeutralMode);
    m_pidShoulderController = m_ShouldMotor.getPIDController();
    m_pidShoulderController.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);

    // initialize cancoder
    m_CANCoder = new CANCoder(ShoulderConstants.kShoulderCANCoderCanID);
  
    m_RelativeEncoder = m_ShouldMotor.getEncoder();   
    m_pidShoulderController.setP(ShoulderConstants.kShoulder_P);
    m_pidShoulderController.setI(ShoulderConstants.kShoulder_I);
    m_pidShoulderController.setD(ShoulderConstants.kShoulder_D);
    m_pidShoulderController.setIZone(ShoulderConstants.kShoulder_Zone);
    m_pidShoulderController.setFF(ShoulderConstants.kShoulder_FF);
    getOffset();
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        getOffset();
      } catch (Exception e) {
      }
      }).start();
    setPositionDegrees(ShoulderConstants.kShoulderStowPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("SetPoint", m_setRotations);
    //SmartDashboard.putNumber("SetDegrees", rotationsToDegrees(m_setRotations));
    SmartDashboard.putNumber("Shoulder Encoder Position", m_RelativeEncoder.getPosition());
    SmartDashboard.putNumber("ArmCANcoder", m_CANCoder.getAbsolutePosition());
    

  }

  public void setPositionDegrees(double degrees) {
    double rotations = degreesToRotations(degrees - m_StartupOffSet);
    //System.out.println("Setting encoder to : " + degrees);
    m_pidShoulderController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    if (m_DeployMode.getprePlace())
         m_DeployMode.setPrePlaceFalse2();
  }

  public double degreesToRotations(double degrees) {
    // make sure we don't go over 360 degrees
    double normalizeDegrees = degrees % 360.0;
    return normalizeDegrees / 360.0 * m_gearRatio;
  }
  

  public double rotationsToDegrees(double rotations) {
    return rotations / m_gearRatio * 360.0;
  }

  // get cancoder value, determine difference from the constant offset, set the
  // relative position to that
  public void getOffset() {
    double l_CANCoderAbsolutePosition = m_CANCoder.getAbsolutePosition();
    // offset value
    m_StartupOffSet = l_CANCoderAbsolutePosition - ShoulderConstants.kShoulderKey; // finds the distance from
                                                                                          // shoulderkey absolute value
                                                                                          // to start up position.
    // m_RelativeEncoder.setPosition(l_StartupOffSet);
    // m_RelativeEncoder.
    SmartDashboard.putNumber("start up offset", m_StartupOffSet);
    // m_setRotations = m_RelativeEncoder.getPosition();
  }

  public void setPositionDegreesTopWithMode(ModeSubsystem s_mode_subsystem) {
    if (s_mode_subsystem.getMode().getMode())// if true then cone
      setPositionDegrees(ShoulderConstants.kShoulderConeTopPosition);
    else
      setPositionDegrees(ShoulderConstants.kShoulderCubeTopPlacePosition);
  }

  public void setPositionDegreesMiddleWithMode(ModeSubsystem s_mode_subsystem) {
    if (s_mode_subsystem.getMode().getMode())// if true then cone
      setPositionDegrees(ShoulderConstants.kShoulderConeMiddlePosition);
    else
      setPositionDegrees(ShoulderConstants.kShoulderCubeMiddlePlacePosition);
  }


  public double getPosition() { // returns degrees that the relitive encoder is set too
    return rotationsToDegrees(m_RelativeEncoder.getPosition());
  }

  public double getEncoderPosition() {
    return m_RelativeEncoder.getPosition();
  }
}
