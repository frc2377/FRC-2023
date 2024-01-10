// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class RainbowMode {

private boolean m_rainbow;   // true for on, false for off
public RainbowMode(boolean s_rainbow) {
     m_rainbow = s_rainbow; 
}

public void  toggleRainbow() {
    m_rainbow = !m_rainbow; 
}
public boolean getRainbow() {
      return m_rainbow;
}









}
