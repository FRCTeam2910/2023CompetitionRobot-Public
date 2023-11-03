package org.frcteam2910.c2023.subsystems.led;

import com.ctre.phoenix.led.CANdle;

public class LEDIOCANdle implements LEDIO {

    private final CANdle candle;

    public LEDIOCANdle(int port, String canBus) {
        this.candle = new CANdle(port, canBus);
        candle.configFactoryDefault();
    }

    @Override
    public void setLEDs(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }
}
