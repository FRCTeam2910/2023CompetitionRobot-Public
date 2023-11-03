package org.frcteam2910.c2023.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.arm.ArmJoint;
import org.frcteam2910.c2023.subsystems.arm.ArmSubsystem;
import org.frcteam2910.c2023.subsystems.led.LEDSubsystem;
import org.frcteam2910.c2023.util.constants.ArmPoseConstants;

/**
 * Zeros the arm. Sets the proper 0 values for the shoulder, telescope, and wrist.
 */
public class HomeArmCommand extends CommandBase {
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
    private static final double ZERO_VELOCITY_TIME_PERIOD = 0.5;

    private final ArmSubsystem armSubsystem;
    private final LEDSubsystem ledSubsystem;

    /**
     * The current timestamp in the zeroing process
     */
    private double zeroTimeStamp;

    /**
     * The current joint of the arm we are homing
     */
    private ArmJoint currentJoint = ArmJoint.WRIST;

    /**
     * Zeros the arm so that all values are set relative to the arm.
     * @param armSubsystem {@link ArmSubsystem}
     */
    public HomeArmCommand(ArmSubsystem armSubsystem, LEDSubsystem ledSubsystem) {
        this.armSubsystem = armSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        currentJoint = ArmJoint.WRIST;
        zeroTimeStamp = Double.NaN; // Initializing starting time stamp
        armSubsystem.setZeroed(false); // Setting zeroed field within the subsystem to be false.
        ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_ROBOT_ARM_NOT_ZEROED);
    }

    @Override
    public void execute() {
        // Slowly moving each part of the arm backwards until we see the velocity dip below the velocity threshold
        // Setting the time stamp to the current FPGA time

        switch (currentJoint) {
            case WRIST:
                if (manageTimer(WRIST_VELOCITY_THRESHOLD)) {
                    armSubsystem.setSensorPositionFromMeasurement(ArmJoint.WRIST, ArmSubsystem.MAX_WRIST_ANGLE_RAD);
                    currentJoint = ArmJoint.EXTENSION;
                    zeroTimeStamp = Double.NaN;
                    armSubsystem.setJointVoltage(ArmJoint.WRIST, 0.0);
                } else {
                    armSubsystem.setJointVoltage(currentJoint, WRIST_ZEROING_VOLTAGE);
                }
                break;
            case EXTENSION:
                if (manageTimer(EXTENSION_VELOCITY_THRESHOLD)) {
                    armSubsystem.setSensorPositionFromMeasurement(
                            ArmJoint.EXTENSION, (ArmSubsystem.MIN_EXTENSION_LENGTH - Units.inchesToMeters(0.25)));
                    currentJoint = ArmJoint.SHOULDER;
                    zeroTimeStamp = Double.NaN;
                    armSubsystem.setJointVoltage(ArmJoint.EXTENSION, 0.0);
                } else {
                    armSubsystem.setJointVoltage(currentJoint, EXTENSION_ZEROING_VOLTAGE);
                }
                break;
            case SHOULDER:
                if (manageTimer(SHOULDER_VELOCITY_THRESHOLD)) {
                    armSubsystem.setSensorPositionFromMeasurement(
                            ArmJoint.SHOULDER, ArmSubsystem.MIN_SHOULDER_ANGLE_RAD);
                    armSubsystem.setJointVoltage(ArmJoint.SHOULDER, 0.0);
                    armSubsystem.setZeroed(true);
                } else {
                    armSubsystem.setJointVoltage(currentJoint, SHOULDER_ZEROING_VOLTAGE);
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stopping the motors
        armSubsystem.setJointVoltage(ArmJoint.EXTENSION, 0.0);
        armSubsystem.setJointVoltage(ArmJoint.SHOULDER, 0.0);
        armSubsystem.setJointVoltage(ArmJoint.WRIST, 0.0);

        if (!interrupted) {
            armSubsystem.setZeroed(true);
            armSubsystem.setTargetPose(ArmPoseConstants.STOW);
            ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_ROBOT_ARM_ZEROED);
        }
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isZeroed();
    }

    private boolean manageTimer(double velocityThreshold) {
        if (Math.abs(armSubsystem.getVelocity(currentJoint)) < velocityThreshold) {
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
