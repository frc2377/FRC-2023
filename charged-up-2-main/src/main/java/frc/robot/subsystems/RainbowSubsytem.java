// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.util.RainbowMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RainbowSubsytem extends SubsystemBase {
  /** Creates a new RainbowSubsytem. */
  private RainbowMode m_rainbow;
   
  public RainbowSubsytem(RainbowMode s_rainbowToggled) {
    m_rainbow = s_rainbowToggled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleRainbow(){
    m_rainbow.toggleRainbow();
  }

  public RainbowMode getRainbowToggle(){
    return m_rainbow;
  }
}
