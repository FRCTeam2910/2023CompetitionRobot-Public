package org.frcteam2910.c2023.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOSim implements LEDIO {
    private final AddressableLED addressableLED;
    private final AddressableLEDBuffer addressableLEDBuffer;

    public LEDIOSim(int port, int length) {

        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        this.addressableLED = new AddressableLED(port);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        this.addressableLEDBuffer = new AddressableLEDBuffer(length);
        this.addressableLED.setLength(addressableLEDBuffer.getLength());

        // Set the data
        this.addressableLED.setData(addressableLEDBuffer);
        this.addressableLED.start();
    }

    @Override
    public void setLEDs(int red, int green, int blue) {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            addressableLEDBuffer.setRGB(i, red, green, blue);
        }

        addressableLED.setData(addressableLEDBuffer);
    }
}
