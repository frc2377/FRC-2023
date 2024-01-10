// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticHub;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final PneumaticHub m_ph;
  private final Solenoid m_wrist;
  private final Solenoid m_claw;
  
  public ClawSubsystem(PneumaticHub ph) {
    m_ph = ph;
    m_claw = m_ph.makeSolenoid(PneumaticConstants.kClawSolenoid);
    m_wrist = m_ph.makeSolenoid(PneumaticConstants.kWristSolenoid);
    
  }


    // Turn wrist 180 Degrees 
    public void ToggleWrist() {
      m_wrist.set(!m_wrist.get());
    }
  
    //Toggle Claw open/close
    public void ToggleClaw() {
      m_claw.set(!m_claw.get());
    }

    // Toggle Claw off
    public void OpenClaw() {
      m_claw.set(false);
    }
  
    // Toggle Claw on 
    public void CloseClaw() {
      m_claw.set(true);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("pressure", m_ph.getPressure(0));
  }
}
