package org.frcteam2910.c2023.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {}

    @AutoLog
    class IntakeIOInputs {
        public double intakeAngularVelocityRadPerSec;
        public double intakeAppliedVolts;
        public double intakeCurrentDrawAmps;

        public double intakeMotorTemp;

        public double beamBreakVoltage;

        public double timeOfFlightMeters;
        public boolean isRangeValid;
    }

    default void setMotorVoltage(double percentOutput) {}
}
