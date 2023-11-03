package org.frcteam2910.c2023.subsystems.led;

public class LEDState {
    public static final LEDState OFF = new LEDState(0, 0, 0); // The LED lights are off
    public static final LEDState INITIALIZE = new LEDState(255, 105, 180); // The LED lights are pink
    public static final LEDState LIMELIGHT = new LEDState(57, 255, 20); // The LED lights are neon green
    public static final LEDState GET_CONE = new LEDState(255, 125, 0); // The LED lights are yellow
    public static final LEDState GET_CUBE = new LEDState(176, 38, 255); // The LED lights are purple
    public static final LEDState ARM_NOT_ZEROED = new LEDState(255, 0, 0); // The LED lights are red
    public static final LEDState ARM_ZEROED = new LEDState(0, 145, 0); // The LED lights are green
    public static final LEDState ROBOT_NOT_ZEROED = new LEDState(128, 0, 0); // The LED lights are maroon
    public static final LEDState ROBOT_ZEROED = new LEDState(0, 155, 119); // The LED lights are emerald green
    public static final LEDState LOW_BATTERY = new LEDState(255, 128, 0); // The LED lights are orange
    public static final LEDState ALIGN_BAD = new LEDState(255, 0, 0);
    public static final LEDState WOULD_EJECT = new LEDState(255, 255, 255);

    public static final LEDState CHARGING_STATION = new LEDState(31, 81, 255); // The LED lights are neon blue

    private LEDState() {}

    public LEDState(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public void copyFrom(LEDState other) {
        this.red = other.red;
        this.green = other.green;
        this.blue = other.blue;
    }

    public int red;
    public int green;
    public int blue;
}
