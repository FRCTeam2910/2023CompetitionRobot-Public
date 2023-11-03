package org.frcteam2910.c2023.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.arm.ArmJoint;
import org.frcteam2910.c2023.subsystems.arm.ArmSubsystem;
import org.frcteam2910.c2023.util.ArmPositions;
import org.frcteam2910.c2023.util.constants.ArmPoseConstants;

public class SimultaneousHomeArmCommand extends CommandBase {
    /**
     * The velocity that the motors must be less than for it to count as the shoulder, wrist, or telescope being down.
     */
    private static final double WRIST_VELOCITY_THRESHOLD = Units.degreesToRadians(0.1);

    private static final double EXTENSION_VELOCITY_THRESHOLD = Units.inchesToMeters(0.1);
    private static final double SHOULDER_VELOCITY_THRESHOLD = Units.degreesToRadians(0.1);
    /**
     * The speed the motors will be moving at during the zeroing process.
     */
    private static final double WRIST_ZEROING_VOLTAGE = 1.5;

    private static final double EXTENSION_ZEROING_VOLTAGE = -1.0;
    private static final double SHOULDER_ZEROING_VOLTAGE = -1.0;
    /**
     * This is the time period that the arm will be getting zeroed at.
     */
    private static final double ZERO_VELOCITY_TIME_PERIOD = 0.25;

    private final ArmSubsystem armSubsystem;

    /**
     * The current timestamp in the zeroing process
     */
    private double zeroTimeStamp;

    private boolean wasHomed;

    /**
     * Zeros the arm so that all values are set relative to the arm.
     * @param armSubsystem {@link ArmSubsystem}
     */
    public SimultaneousHomeArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        zeroTimeStamp = Double.NaN; // Initializing starting time stamp
        wasHomed = armSubsystem.isZeroed();
        armSubsystem.setZeroed(false); // Setting zeroed field within the subsystem to be false.
    }

    @Override
    public void execute() {
        armSubsystem.setJointVoltage(ArmJoint.SHOULDER, SHOULDER_ZEROING_VOLTAGE);
        armSubsystem.setJointVoltage(ArmJoint.EXTENSION, EXTENSION_ZEROING_VOLTAGE);
        armSubsystem.setJointVoltage(ArmJoint.WRIST, WRIST_ZEROING_VOLTAGE);
    }

    @Override
    public void end(boolean interrupted) {
        // Stopping the motors
        armSubsystem.setJointVoltage(ArmJoint.SHOULDER, 0.0);
        armSubsystem.setJointVoltage(ArmJoint.EXTENSION, 0.0);
        armSubsystem.setJointVoltage(ArmJoint.WRIST, 0.0);

        armSubsystem.setZeroed(wasHomed);

        if (!interrupted) {
            armSubsystem.setAllSensorPositionsFromMeasurement(new ArmPositions(
                    ArmSubsystem.MIN_SHOULDER_ANGLE_RAD,
                    ArmSubsystem.MIN_EXTENSION_LENGTH - Units.inchesToMeters(0.25),
                    ArmSubsystem.MAX_WRIST_ANGLE_RAD));
            armSubsystem.setZeroed(true);
            armSubsystem.setTargetPose(ArmPoseConstants.STOW);
        }
    }

    @Override
    public boolean isFinished() {
        boolean shoulderStopped = Math.abs(armSubsystem.getVelocity(ArmJoint.SHOULDER)) < SHOULDER_VELOCITY_THRESHOLD;
        boolean extensionStopped =
                Math.abs(armSubsystem.getVelocity(ArmJoint.EXTENSION)) < EXTENSION_VELOCITY_THRESHOLD;
        boolean wristStopped = Math.abs(armSubsystem.getVelocity(ArmJoint.WRIST)) < WRIST_VELOCITY_THRESHOLD;

        if (shoulderStopped && extensionStopped && wristStopped) {
            if (!Double.isFinite(zeroTimeStamp)) {
                zeroTimeStamp = Timer.getFPGATimestamp();
                return false;
            } else {
                return Timer.getFPGATimestamp() - zeroTimeStamp >= ZERO_VELOCITY_TIME_PERIOD;
            }
        } else {
            zeroTimeStamp = Double.NaN;
            return false;
        }
    }
}
