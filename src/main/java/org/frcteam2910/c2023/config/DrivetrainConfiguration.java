package org.frcteam2910.c2023.config;

import edu.wpi.first.math.controller.PIDController;

public class DrivetrainConfiguration {
    public double[] swerveModuleOffsets;
    public int mountingAngle;
    public double error;
    public PIDController xController = new PIDController(0, 0, 0);
    public PIDController yController = new PIDController(0, 0, 0);
    public PIDController rotationController = new PIDController(0, 0, 0);
}
