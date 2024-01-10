// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DeployMode {

    private boolean m_mode;   // true for cone, false for cube
    private boolean m_preplace; // true for preplace, false for not
    public DeployMode(boolean p_mode, boolean p_preplace) {
     m_mode = p_mode; 
     m_preplace = p_preplace;
    }

    public void  toggleMode() {
        m_mode = !m_mode; 
    }
    
    public void  setPrePlaceTrue2() {
        m_preplace = true; 
    }

    public void  setPrePlaceFalse2() {
        m_preplace = false; 
    }


    public boolean getMode() {
        return m_mode;
    }
    public boolean getprePlace() {
        SmartDashboard.putBoolean("preplacegetonbuttonpress", m_preplace);
        return m_preplace;
    }
    
}



