// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// led default command

package frc.robot.commands;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ModeSubsystem;
import frc.robot.subsystems.RainbowSubsytem;
import frc.robot.subsystems.GridLimelightSubsystem;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.LimeLightConstants;

public class LEDDefaultCommand extends CommandBase {
  /** Creates counter new LimelightDefaultCommand. */
  private LEDSubsystem m_LED;
  private ModeSubsystem m_ModeSubsytem;
  private RainbowSubsytem m_RainbowMode;
  
  
    public LEDDefaultCommand( LEDSubsystem s_LED, ModeSubsystem s_ModeSubsytem, RainbowSubsytem s_RainbowMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LED = s_LED;  
    m_ModeSubsytem = s_ModeSubsytem;  
    m_RainbowMode = s_RainbowMode;
    addRequirements(m_LED,m_ModeSubsytem, m_RainbowMode);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LED.setModeColor(Constants.LEDs.kYellow, LEDs.kSatDefault); // for cone on start up
    m_LED.setTargetColor(Constants.LEDs.kRed, LEDs.kSatDefault);  // for I can't see the target on start up
    m_LED.startLEDs();
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
     if(m_RainbowMode.getRainbowToggle().getRainbow() == true)
    {
      m_LED.setRainbow();
      // counter++;
      // if (counter == 4)
      //   counter = 0; 
    }
    if (!m_RainbowMode.getRainbowToggle().getRainbow())
    {
      if(Math.abs(GridLimelightSubsystem.leftRightDistanceMeters()) < LimeLightConstants.kAlignThreshold){
        m_LED.setTargetColor(Constants.LEDs.kGreen, LEDs.kSatDefault);
      }
      else if (GridLimelightSubsystem.hasTarget()){
        m_LED.setTargetColor(Constants.LEDs.kWhite, LEDs.kSat0);
      }
      else{
        m_LED.setTargetColor(Constants.LEDs.kRed, LEDs.kSatDefault);
      }
      if(m_ModeSubsytem.getMode().getMode())//if true then cone
        m_LED.setModeColor(LEDs.kYellow, LEDs.kSatDefault);
      else
          m_LED.setModeColor(LEDs.kBlue, LEDs.kSatDefault);
    }
   }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_LED.stopLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
