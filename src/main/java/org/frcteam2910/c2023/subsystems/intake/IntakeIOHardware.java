package org.frcteam2910.c2023.subsystems.intake;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.AnalogInput;
import org.frcteam2910.c2023.config.PortConfiguration;

public class IntakeIOHardware implements IntakeIO {
    private static final double GEAR_RATIO = 1.0;
    private static final double VELOCITY_COEFFICIENT = (2 * Math.PI) / (2048 * GEAR_RATIO) * 10;

    private final TalonFX motor;
    private final AnalogInput beamBreak;
    private final TimeOfFlight timeOfFlight;

    public IntakeIOHardware(PortConfiguration portConfiguration) {
        motor = new TalonFX(portConfiguration.intakeMotorID, portConfiguration.ArmCANBus);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.currentLimit = 40;
        config.supplyCurrLimit.enable = true;
        motor.configAllSettings(config);
        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);

        motor.setSensorPhase(true);
        motor.setInverted(TalonFXInvertType.CounterClockwise);

        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        beamBreak = new AnalogInput(portConfiguration.beamBreakID);

        // Configure with a unique CAN ID
        timeOfFlight = new TimeOfFlight(portConfiguration.timeOfFlightID);
        timeOfFlight.setRangingMode(TimeOfFlight.RangingMode.Short, 0.1);
        // Set a more focused sensing beam
        timeOfFlight.setRangeOfInterest(6, 6, 14, 14);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAngularVelocityRadPerSec = motor.getSelectedSensorVelocity() * VELOCITY_COEFFICIENT;
        inputs.intakeCurrentDrawAmps = motor.getSupplyCurrent();
        inputs.intakeAppliedVolts = motor.getMotorOutputVoltage();

        inputs.intakeMotorTemp = motor.getTemperature();

        inputs.beamBreakVoltage = beamBreak.getVoltage();

        // convert millimeters to meters
        inputs.timeOfFlightMeters = (timeOfFlight.getRange() / 1000.0) - 0.04;
        inputs.isRangeValid = timeOfFlight.isRangeValid();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.set(TalonFXControlMode.PercentOutput, voltage / 12);
    }
}
