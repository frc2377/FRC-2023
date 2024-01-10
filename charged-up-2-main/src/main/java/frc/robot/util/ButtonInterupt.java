// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class ButtonInterupt {

    private boolean m_state = false; // false by default true if set by driver to cancel
    public ButtonInterupt(boolean s_state) {
        m_state = s_state; 
    }

    public void  toggleState() {
        m_state = !m_state; 
    }

    public void setState(boolean s_state) {
        m_state = s_state;
    }

    public boolean getState() {
        return m_state;
    }
}



