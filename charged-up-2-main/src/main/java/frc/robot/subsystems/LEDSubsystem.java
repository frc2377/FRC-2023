package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixleHue;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led = new AddressableLED(Constants.LEDs.kPwmChannel);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDs.kLEDBufferLength);
    m_led.setLength(m_ledBuffer.getLength());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startLEDs() {
    m_led.start();
  }

  public void stopLEDs() {
    m_led.stop();
  }

  public void setRainbow()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
    {
       final var hue = (m_rainbowFirstPixleHue + (i * 180 / m_ledBuffer.getLength())) % 180;
       m_ledBuffer.setHSV(i, hue, 255, 128);
    }  
   m_rainbowFirstPixleHue += 3;

   m_rainbowFirstPixleHue %= 180;
  
   m_led.setData(m_ledBuffer);
  }
  


  public void setTargetColor(int hue, int sat) {
    for (var i = 0; i < Constants.LEDs.kLEDTargetLength; i++) {
      m_ledBuffer.setHSV(i, hue, sat, 128);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setModeColor(int hue, int sat) {
    for (var i = Constants.LEDs.kLEDTargetLength; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, hue, sat, 128);
    }  
    m_led.setData(m_ledBuffer);
  }
}

