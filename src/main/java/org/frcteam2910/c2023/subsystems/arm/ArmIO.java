package org.frcteam2910.c2023.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default void updateInputs(ArmIOInputs inputs) {}

    default void setTargetShoulderAngle(double angleRad) {}

    default void setTargetExtensionLength(double lengthMeters) {}

    default void setTargetWristAngle(double angleRad) {}

    default void setShoulderVoltage(double voltage) {}

    default void setExtensionVoltage(double voltage) {}

    default void setWristVoltage(double voltage) {}

    default void setSensorPositionFromMeasurement(ArmJoint joint, double position) {}

    default void setNeutralMode(boolean brake) {}

    default void setShoulderAccelerationConstraint(boolean fastConstraints) {}

    default void setExtensionCurrentLimit(boolean highCurrent) {}

    @AutoLog
    class ArmIOInputs {
        public double shoulderAngleRad;
        public double shoulderAppliedVolts;
        public double shoulderCurrentDrawAmps;
        public double shoulderAngularVelocityRadPerSec;

        public double wristAngleRad;
        public double wristAppliedVolts;
        public double wristCurrentDrawAmps;
        public double wristAngularVelocityRadPerSec;

        public double extensionAppliedVolts;
        public double extensionCurrentDrawAmps;
        public double extensionVelocityMetersPerSec;
        public double extensionPositionMeters;

        public double shoulderMotorOneTemp;
        public double shoulderMotorTwoTemp;
        public double shoulderMotorThreeTemp;
        public double shoulderMotorFourTemp;

        public double extensionMotorOneTemp;
        public double extensionMotorTwoTemp;

        public double wristMotorTemp;

        public boolean isFastShoulderAcceleration;
        public boolean isHighExtensionCurrentLimit;
    }
}
