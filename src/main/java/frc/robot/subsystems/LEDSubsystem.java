
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    // lights
    private AddressableLED leds = new AddressableLED(3);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(102);


    
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern alliance = LEDPattern.solid(Color.kWhite);
    LEDPattern puce = LEDPattern.solid(Constants.kPuce);

    public LEDSubsystem() {
          leds.setLength(m_ledBuffer.getLength());
        leds.setData(m_ledBuffer);
        leds.start();


    }



    // A method to set the speed (percentage) of the feeder roller
    public void defaultColor(LEDPattern color) {
        updatePattern(color);
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

    public void configureAlliance() {
      var alliance = DriverStation.getAlliance();
      LEDPattern alliancePattern;
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
          alliancePattern = red;
      } else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
          alliancePattern = blue;
      } else {
          alliancePattern = puce;
      }
      defaultColor(alliancePattern);
    }

    /** Returns true if alliance data is available from FMS/DS. */
    public boolean isAllianceKnown() {
        return DriverStation.getAlliance().isPresent();
    }


}
