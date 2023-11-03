package org.frcteam2910.c2023.subsystems.drivetrain;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.GyroTrimConfigs;
import com.ctre.phoenixpro.configs.MountPoseConfigs;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

/**
 * Implementation of the gyroscope class. Uses the Pigeon2 gyroscope to receive data about the robots yaw, pitch, and roll values.
 */
public class GyroIOPigeon2 implements GyroIO {

    /**
     * Pigeon2 gyroscope object. Receives direct data from the gyroscope about the yaw, pitch, and roll.
     */
    private final Pigeon2 gyro;

    /**
     * Pigeon2 gyroIO implementation.
     *
     * @param canId  Is the unique identifier to the gyroscope used on the robot.
     * @param canBus The name of the CAN bus the device is connected to.
     */
    private StatusSignalValue<Double> yawSignal;

    private StatusSignalValue<Double> angularVelocitySignal;

    private BaseStatusSignalValue[] signals;

    public GyroIOPigeon2(int canId, int mountPose, double error, String canBus) {
        gyro = new Pigeon2(canId, canBus);

        Pigeon2Configuration config = new Pigeon2Configuration();
        MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
        mountPoseConfigs.MountPoseYaw = mountPose;
        GyroTrimConfigs gyroTrimConfigs = new GyroTrimConfigs();
        gyroTrimConfigs.GyroScalarZ = error;
        config.MountPose = mountPoseConfigs;
        config.GyroTrim = gyroTrimConfigs;
        gyro.getConfigurator().apply(config);

        yawSignal = gyro.getYaw();
        angularVelocitySignal = gyro.getAngularVelocityZ();
        signals = new BaseStatusSignalValue[2];
        signals[0] = yawSignal;
        signals[1] = angularVelocitySignal;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;

        inputs.yaw = Units.degreesToRadians(
                BaseStatusSignalValue.getLatencyCompensatedValue(yawSignal, angularVelocitySignal));
        inputs.pitch = Units.degreesToRadians(gyro.getPitch().getValue());
        inputs.roll = Units.degreesToRadians(gyro.getRoll().getValue());
        inputs.angularVelocity = Units.degreesToRadians(angularVelocitySignal.getValue());
    }

    @Override
    public BaseStatusSignalValue[] getSignals() {
        return signals;
    }
}
