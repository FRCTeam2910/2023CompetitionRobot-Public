package org.frcteam2910.c2023.subsystems.drivetrain;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import org.littletonrobotics.junction.AutoLog;

/**
 * Connects the software to the hardware and directly receives data from the gyroscope.
 */
public interface GyroIO {
    /**
     * Reads information from sources (hardware or simulation) and updates the inputs object.
     *
     * @param inputs Contains the defaults for the input values listed above.
     */
    default void updateInputs(GyroIOInputs inputs) {}

    default BaseStatusSignalValue[] getSignals() {
        return new BaseStatusSignalValue[0];
    }

    /**
     * Holds data that can be read from the corresponding gyroscope IO implementation.
     */
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public double yaw = 0;
        public double pitch = 0;
        public double roll = 0;
        public double angularVelocity = 0;
    }
}
