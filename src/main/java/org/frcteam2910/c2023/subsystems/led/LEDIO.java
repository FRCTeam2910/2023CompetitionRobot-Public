package org.frcteam2910.c2023.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

/**
 * LEDIO interface where you update the red, green, and blue LEDIO inputs
 */
public interface LEDIO {
    default void updateInputs(LEDIOInputs inputs, int red, int green, int blue) {
        inputs.red = red;
        inputs.green = green;
        inputs.blue = blue;
    }

    /**
     * Sets the LEDs to a red, green, and blue value
     */
    default void setLEDs(int red, int green, int blue) {}

    @AutoLog
    class LEDIOInputs {
        public long red;
        public long green;
        public long blue;
    }
}
