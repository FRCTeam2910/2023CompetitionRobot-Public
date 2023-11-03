package org.frcteam2910.c2023.util;

public class ArmPositions {
    private double shoulderAngleRad = 0;
    private double extensionLengthMeters = 0;
    private double wristAngleRad = 0;

    public ArmPositions(double shoulderAngleRad, double extensionLengthMeters, double wristAngleRad) {
        this.shoulderAngleRad = shoulderAngleRad;
        this.extensionLengthMeters = extensionLengthMeters;
        this.wristAngleRad = wristAngleRad;
    }

    public double getShoulderAngleRad() {
        return shoulderAngleRad;
    }

    public double getExtensionLengthMeters() {
        return extensionLengthMeters;
    }

    public double getWristAngleRad() {
        return wristAngleRad;
    }
}
