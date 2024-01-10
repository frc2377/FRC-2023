// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DeployMode;

public class ModeSubsystem extends SubsystemBase {
  /** Creates a new ModeSubsytem. */
  private DeployMode m_mode;
  public ModeSubsystem(DeployMode s_mode) {
    m_mode = s_mode; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("mode", m_mode.getMode());
    SmartDashboard.putBoolean("preplace", m_mode.getprePlace());

  }

  public void toggleMode(){
    m_mode.toggleMode(); 
  
  }
  
  public void setPrePlaceFalse() {
    m_mode.setPrePlaceFalse2();
  }
  public void setPrePlaceTrue() {
    m_mode.setPrePlaceTrue2();
  }
  
  public DeployMode getMode() {
    return m_mode;
  }

}
