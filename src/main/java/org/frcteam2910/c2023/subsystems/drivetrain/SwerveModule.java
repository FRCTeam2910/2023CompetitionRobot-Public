package org.frcteam2910.c2023.subsystems.drivetrain;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the calculation logic for the swerve drive modules
 */
public class SwerveModule {

    /**
     * Brings in the swerve module IO class. This gives us direct control to the hardware/simulated swerve module
     */
    private final SwerveModuleIO io;
    /**
     * Brings in the swerve module data for positioning calculations
     */
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    /**
     * Module identification name (FrontLeft, FrontRight...etc)
     */
    private final String name;

    /**
     * Initializes the IO and the identification name.
     *
     * @param io   The IO class. Changes based off of if we're simulating or running the code on an actual machine.
     * @param name The identification name of the swerve module.
     */
    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;
    }

    /**
     * Does the math calculations necessary to get the current swerve module state to the target swerve module state. Finds the fastest way to the swerve module angle and drive velocity.
     *
     * @param targetState The target swerve module state (contains the target drive velocity and target angle for the specified swerve module)
     */
    public void setTargetState(SwerveModuleState targetState) {
        double currentAngle = inputs.steerPositionRad; // Current angle of the swerve module
        double targetAngle = MathUtil.inputModulus(
                targetState.angle.getRadians(),
                0,
                2 * Math.PI); // Target angle of the swerve module, limited to a domain between 0 and 2π.

        double absoluteAngle = MathUtil.inputModulus(
                currentAngle, 0, 2 * Math.PI); // Limiting the domain of the current angle to a domain of 0 to 2π.

        double angleError = MathUtil.inputModulus(
                targetAngle - absoluteAngle,
                -Math.PI,
                Math.PI); // Finding the difference in between the current and target angle (in radians).
        double resultAngle = currentAngle
                + angleError; // Adding that distance to our current angle (directly from the steer encoder). Becomes
        // our target angle

        // Setting the target swerve module state values (drive velocity and steer angle).
        io.setTargetDriveVelocity(targetState.speedMetersPerSecond);
        io.setTargetSteerPosition(resultAngle);
    }

    public void setTargetSteerAngle(SwerveModuleState targetState, double turnSpeed) {
        double currentAngle = inputs.steerPositionRad; // Current angle of the swerve module
        double targetAngle = MathUtil.inputModulus(
                targetState.angle.getRadians(),
                0,
                2 * Math.PI); // Target angle of the swerve module, limited to a domain between 0 and 2π.

        double absoluteAngle = MathUtil.inputModulus(
                currentAngle, 0, 2 * Math.PI); // Limiting the domain of the current angle to a domain of 0 to 2π.

        double angleError = MathUtil.inputModulus(
                targetAngle - absoluteAngle,
                -Math.PI,
                Math.PI); // Finding the difference in between the current and target angle (in radians).
        double resultAngle = currentAngle
                + angleError; // Adding that distance to our current angle (directly from the steer encoder). Becomes
        // our target angle

        // Setting the target swerve module state values (drive velocity and steer angle).
        io.setTargetDriveVelocity(targetState.speedMetersPerSecond);
        io.setTargetSteerAngle(resultAngle, turnSpeed);
    }

    /**
     * Getting the current swerve module position
     *
     * @return The drive speed and steer angle of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, new Rotation2d(inputs.steerPositionRad));
    }

    /**
     * Updating the inputs through the IO class and then logging those inputs
     */
    public void updateInputs() {
        if (DriverStation.isDisabled()) {
            io.resetToAbsoluteAngle();
        }

        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/" + name + "Module", inputs);
    }

    public BaseStatusSignalValue[] getSignals() {
        return io.getSignals();
    }
}
