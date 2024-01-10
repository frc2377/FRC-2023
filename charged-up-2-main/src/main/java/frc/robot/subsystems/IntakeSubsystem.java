// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;


import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import com.revrobotics.CANSparkMaxLowLevel;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  // Declare Motor Controllers
 // private final CANSparkMax m_intakeMotorBottom = new CANSparkMax(Constants.Intake.kIntakeMotorBottom, MotorType.kBrushless);
  //private final CANSparkMax m_intakeMotorTop = new CANSparkMax(Constants.Intake.kIntakeMotorBottom, MotorType.kBrushless);
  
  private final PneumaticHub m_ph;
  private final CANSparkMax m_upperWheels;
  private final CANSparkMax m_lowerWheels;
  private final DigitalInput m_IntakeSensor = new DigitalInput(Constants.Intake.kIntakeSensorPort);
  // Declare SparkMAX PID Controllers
  // private final SparkMaxPIDController m_stagerPID =
  // m_stagerMotor.getPIDController();

  private Solenoid m_intake; //Controls the deplyoment and retraction of the intake 

  public IntakeSubsystem(PneumaticHub ph) {
    m_ph = ph;
    m_intake = m_ph.makeSolenoid(Constants.PneumaticConstants.kIntakeSolenoid);
    m_upperWheels = new CANSparkMax(Intake.kIntakeMotorTop, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_lowerWheels = new CANSparkMax(Intake.kIntakeMotorBottom, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_upperWheels.restoreFactoryDefaults();
    m_lowerWheels.restoreFactoryDefaults();
    m_lowerWheels.setInverted(false);
    m_upperWheels.setInverted(true);
    m_lowerWheels.setIdleMode(Constants.Swerve.driveNeutralMode);
    m_upperWheels.setIdleMode(Constants.Swerve.driveNeutralMode);
  }

  public double getOutputCurrentLowerWheels(){
    return m_lowerWheels.getOutputCurrent();
  }

  public double getOutputCurrentUpperWheels(){
    return m_upperWheels.getOutputCurrent();
  }

  public void setLowerWheelsPercentage(double percentage) {
  m_upperWheels.set(percentage);
}

  public boolean getIntakeHasCube(){
    return !m_IntakeSensor.get();
  }

  public void setUpperWheelsPercentage(double percentage) {
  m_lowerWheels.set(-percentage);
}

public void setWheelsOn(){
  m_lowerWheels.set(-Intake.kIntakeMotorPercentage);
  m_upperWheels.set(Intake.kIntakeMotorPercentage);
}


public void ShootFromIntakeMid(){
 // setWheelsOff();
  m_lowerWheels.set(Intake.kIntakeMotorShootMidPercentage);
  m_upperWheels.set(-Intake.kIntakeMotorShootMidPercentage);

}

public void ShootFromIntakeHigh(){
 // setWheelsOff();
  m_lowerWheels.set(Intake.kIntakeMotorShootHighPercentage);
  m_upperWheels.set(-Intake.kIntakeMotorShootHighPercentage);

}

public void ShootFromIntakeLow(){
 //  setWheelsOff();
  m_lowerWheels.set(Intake.kIntakeMotorShootLowPercentage);
  m_upperWheels.set(-Intake.kIntakeMotorShootLowPercentage);

}

public void setWheelsOff(){
  m_lowerWheels.set(0);
  m_upperWheels.set(0);
}
public void setWheelsMostlyOff(){
  m_lowerWheels.set(-.01);
  m_upperWheels.set(.025);
}
public void setWheelsFrontSpin(){
  m_lowerWheels.set(1);
  m_upperWheels.set(-1);
}
public void setWheelsReverse(){
  m_lowerWheels.set(Intake.kIntakeMotorPercentage);
  m_upperWheels.set(-Intake.kIntakeMotorPercentage);
}

// Toggle Intake off
 public void OpenIntake() {
  m_intake.set(true);
}

// Toggle Intake on 
public void CloseIntake() {
  m_intake.set(false);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Intake Lower Wheels Current Output ", getOutputCurrentLowerWheels());
     SmartDashboard.putNumber("Intake Upper Wheels Current Output ", getOutputCurrentUpperWheels());
  }
}
