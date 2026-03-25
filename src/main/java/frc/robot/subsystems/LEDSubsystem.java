
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    // lights
    private AddressableLED leds = new AddressableLED(3);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(120);

    final Color kPuce = new Color(80, 20, 40);
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern alliance = LEDPattern.solid(Color.kWhite);
    LEDPattern puce = LEDPattern.solid(kPuce);

    public LEDSubsystem() {

    }

    // A method to set the speed (percentage) of the feeder roller
    public void defaultColor() {
        updatePattern(alliance);
    }

    public void updatePattern(LEDPattern pattern) {
        // Apply the LED pattern to the data buffer
        pattern.applyTo(m_ledBuffer);
        // Write the data to the LED strip
        leds.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {

    }

    public void configureLEDS() {
        leds.setLength(m_ledBuffer.getLength());
        leds.setData(m_ledBuffer);
        leds.start();

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            alliance = red;
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            alliance = blue;
        } else {
            alliance = puce;
        }
    }

}
