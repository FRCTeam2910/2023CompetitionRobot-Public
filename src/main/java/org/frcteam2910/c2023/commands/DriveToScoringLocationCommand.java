package org.frcteam2910.c2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.drivetrain.DrivetrainSubsystem;
import org.frcteam2910.c2023.subsystems.intake.IntakeSubsystem;
import org.frcteam2910.c2023.subsystems.led.LEDSubsystem;
import org.frcteam2910.c2023.subsystems.vision.VisionSubsystem;
import org.frcteam2910.c2023.util.GamePiece;
import org.frcteam2910.c2023.util.GridLevel;
import org.frcteam2910.c2023.util.OperatorDashboard;
import org.frcteam2910.c2023.util.constants.WaypointConstants;
import org.littletonrobotics.junction.Logger;

public class DriveToScoringLocationCommand extends CommandBase {
    /** Drivetrain object to access subsystem. */
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final VisionSubsystem visionSubsystem;

    private final IntakeSubsystem intakeSubsystem;

    private LEDSubsystem ledSubsystem = null;
    /** Operator dashboard to get scoring position offset and target piece. */
    private final OperatorDashboard operatorDashboard;
    /** Desired pose to drive to. */
    private Pose2d desiredPose;
    /** Allowable error between pose setpoint and actual pose. */
    private final double ALLOWABLE_POSITIVE_X_POSE_ERROR = Units.inchesToMeters(1.0);

    private final double ALLOWABLE_NEGATIVE_X_POSE_ERROR = -Units.inchesToMeters(3.0);
    private final double ALLOWABLE_Y_POSE_ERROR = Units.inchesToMeters(1.0);

    private final double ALLOWABLE_ANGLE_ERROR = Units.degreesToRadians(1.0);
    private final double ALLOWABLE_PITCH_ERROR = Units.degreesToRadians(3.0);

    private static final double STATIC_FRICTION_CONSTANT = 0.0;
    private static final double ANGULAR_STATIC_FRICTION_CONSTANT = Units.degreesToRadians(7.5);

    private static final double SCORING_VELOCITY_COEFFICIENT = 0.2;
    private static final double SCORING_ALIGN_VELOCITY_COEFFICIENT = 0.4;
    private static final double LOW_SCORING_VELOCITY_COEFFICIENT = 0.7;
    private static final double AUTO_ALIGN_VELOCITY_COEFFICIENT = 0.3;

    private static final double MAX_COLUMN_DISTANCE = Units.inchesToMeters(36);

    /** PID controller which uses the error between the robot's current y-pose and the pose setpoint to output a field-relative y-velocity.  */
    private final PIDController yController = new PIDController(3.0, 0.5, 0);
    /** PID controller which uses the error between the current angle and setpoint angle to output angular velocity */
    private final PIDController thetaController = new PIDController(5.0, 0, 0.2);

    private final Debouncer autoEjectDebouncer = new Debouncer(0.1);

    private Pose2d initialPosition;
    private final Supplier<Rotation2d> targetAngle;

    private final DoubleSupplier xVelocitySupplier;
    private final DoubleSupplier yVelocitySupplier;

    private XboxController controller = null;
    private final BooleanSupplier aButtonPressed;
    private final BooleanSupplier rightTriggerHeld;
    private BooleanSupplier autoScoreSupplier = () -> false;
    private boolean takeInitialPose;
    private boolean notStartedPlacement;

    public DriveToScoringLocationCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            OperatorDashboard operatorDashboard,
            Rotation2d targetAngle,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            BooleanSupplier aButtonPressed,
            BooleanSupplier rightTriggerHeld,
            VisionSubsystem visionSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this(
                drivetrainSubsystem,
                operatorDashboard,
                () -> targetAngle,
                xVelocitySupplier,
                yVelocitySupplier,
                aButtonPressed,
                rightTriggerHeld,
                visionSubsystem,
                intakeSubsystem);
    }

    public DriveToScoringLocationCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            OperatorDashboard operatorDashboard,
            Supplier<Rotation2d> targetAngle,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            XboxController controller,
            BooleanSupplier aButtonPressed,
            BooleanSupplier rightTriggerHeld,
            VisionSubsystem visionSubsystem,
            IntakeSubsystem intakeSubsystem,
            BooleanSupplier autoScoreSupplier,
            LEDSubsystem ledSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.operatorDashboard = operatorDashboard;
        this.targetAngle = targetAngle;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.controller = controller;
        this.aButtonPressed = aButtonPressed;
        this.rightTriggerHeld = rightTriggerHeld;
        this.visionSubsystem = visionSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.autoScoreSupplier = autoScoreSupplier;
        this.ledSubsystem = ledSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToScoringLocationCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            OperatorDashboard operatorDashboard,
            Supplier<Rotation2d> targetAngle,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            BooleanSupplier aButtonPressed,
            BooleanSupplier rightTriggerHeld,
            VisionSubsystem visionSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.operatorDashboard = operatorDashboard;
        this.targetAngle = targetAngle;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.aButtonPressed = aButtonPressed;
        this.rightTriggerHeld = rightTriggerHeld;
        this.visionSubsystem = visionSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        autoEjectDebouncer.calculate(false);

        yController.reset();
        yController.setIntegratorRange(-0.5, 0.5);

        thetaController.reset();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        initialPosition = drivetrainSubsystem.getPose();

        notStartedPlacement = true;
    }

    @Override
    public void execute() {
        double xVelocity;
        double yVelocity;
        int waypoint;

        if (operatorDashboard.getSelectedGridLevel() == GridLevel.LOW) {
            xVelocity = xVelocitySupplier.getAsDouble() * LOW_SCORING_VELOCITY_COEFFICIENT;
            yVelocity = yVelocitySupplier.getAsDouble() * LOW_SCORING_VELOCITY_COEFFICIENT;
        } else {
            xVelocity = xVelocitySupplier.getAsDouble() * SCORING_VELOCITY_COEFFICIENT;
            yVelocity = yVelocitySupplier.getAsDouble()
                    * (aButtonPressed.getAsBoolean()
                            ? SCORING_ALIGN_VELOCITY_COEFFICIENT
                            : SCORING_VELOCITY_COEFFICIENT);
        }

        if (aButtonPressed.getAsBoolean()) {
            initialPosition = drivetrainSubsystem.getPose();
            takeInitialPose = false;

            waypoint = WaypointConstants.getClosestNode(initialPosition, operatorDashboard, intakeSubsystem);

            Logger.getInstance().recordOutput("AutoScoreSupplier", autoScoreSupplier.getAsBoolean());

            if (!rightTriggerHeld.getAsBoolean() && !autoScoreSupplier.getAsBoolean() && notStartedPlacement) {
                desiredPose = WaypointConstants.getPoseByWaypoint(waypoint);

                Logger.getInstance()
                        .recordOutput(
                                "Drive/TargetPose",
                                new Pose2d(desiredPose.getX(), desiredPose.getY(), targetAngle.get()));

                desiredPose = new Pose2d(
                        desiredPose.getX(),
                        desiredPose.getY()
                                - (intakeSubsystem.getTargetPiece() == GamePiece.CONE
                                        ? intakeSubsystem.getConeOffset().getY()
                                        : 0.0)
                                - Units.inchesToMeters(operatorDashboard.getTranslationOffset()),
                        desiredPose.getRotation());

                Logger.getInstance()
                        .recordOutput(
                                "Drive/TargetPose with ToF",
                                new Pose2d(desiredPose.getX(), desiredPose.getY(), targetAngle.get()));

                if (controller != null) {
                    if (isAllowablePosition(!operatorDashboard.getShouldUseAutoScore())) {
                        controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                    } else {
                        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                    }
                }

                drivetrainSubsystem.setGoodToEject(isAllowablePosition(true));
            } else {
                notStartedPlacement = false;
            }

            //            visionSubsystem.setUseSingleTag(true, WaypointConstants.getClosestTagByWaypoint(waypoint));
            Logger.getInstance().recordOutput("Vision/ClosestTag", WaypointConstants.getClosestTagByWaypoint(waypoint));
            Logger.getInstance().recordOutput("Drive/isAllowablePosition", isAllowablePosition());

            if (shouldResetAccumulator()) {
                yController.reset();
            }

            if (ledSubsystem != null) {
                if (isHorizontalTranslationAcceptable()) {
                    if (isAllowablePosition(true)) {
                        if (intakeSubsystem.getTargetPiece() == GamePiece.CONE) {
                            ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_GETTING_CONE_FLASHING_WHITE);
                        } else {
                            ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_GETTING_CUBE_FLASHING_WHITE);
                        }
                    } else {
                        if (intakeSubsystem.getTargetPiece() == GamePiece.CONE) {
                            ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_GETTING_CONE_FLASHING);
                        } else {
                            ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_GETTING_CUBE_FLASHING);
                        }
                    }
                } else {
                    ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_OFF);
                }
            }
            if (desiredPose != null) {
                Logger.getInstance()
                        .recordOutput(
                                "DistanceFromAutoAlign",
                                drivetrainSubsystem
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(desiredPose.getTranslation()));
                if (drivetrainSubsystem.getPose().getTranslation().getDistance(desiredPose.getTranslation())
                        < MAX_COLUMN_DISTANCE) {
                    yVelocity =
                            yController.calculate(drivetrainSubsystem.getPose().getY(), desiredPose.getY());
                    yVelocity += Math.copySign(STATIC_FRICTION_CONSTANT, yVelocity);
                    double minVelocity = Math.min(
                            Math.abs(yVelocity),
                            drivetrainSubsystem.getMaxVelocityMetersPerSec() * AUTO_ALIGN_VELOCITY_COEFFICIENT);
                    yVelocity = Math.copySign(minVelocity, yVelocity);
                } else {
                    yController.reset();
                    if (ledSubsystem != null) {
                        ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_ALIGN_BAD);
                    }
                }
            }

            if (desiredPose != null
                    && desiredPose.getX() > drivetrainSubsystem.getPose().getX()) {
                xVelocity = Math.max(0, xVelocity);
            }
        } else {
            if (ledSubsystem != null) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_OFF);
            }
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            drivetrainSubsystem.setGoodToEject(false);
            visionSubsystem.setUseSingleTag(false);
            takeInitialPose = true;
        }

        double rotationalVelocity = thetaController.calculate(
                drivetrainSubsystem.getPose().getRotation().getRadians()
                        - Units.degreesToRadians(operatorDashboard.getRotationOffset()),
                targetAngle.get().getRadians());

        //        rotationalVelocity += Math.copySign(ANGULAR_STATIC_FRICTION_CONSTANT, rotationalVelocity);

        Logger.getInstance().recordOutput("Drive/TargetAngle", targetAngle.get().getRadians());
        Logger.getInstance().recordOutput("Auto Align/yVelocity", yVelocity);

        drivetrainSubsystem.setTargetVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                rotationalVelocity,
                drivetrainSubsystem
                        .getPose()
                        .getRotation()
                        .plus(new Rotation2d(drivetrainSubsystem.getAngularVelocity()
                                * DefaultDriveCommand.ANGULAR_VELOCITY_COEFFICIENT))));
    }

    @Override
    public void end(boolean interrupted) {
        if (controller != null) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }
        operatorDashboard.setScoringPositionOffset(0);
        visionSubsystem.setUseSingleTag(false);
        if (ledSubsystem != null) {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_OFF);
        }
    }

    private boolean isAllowablePosition() {
        return isAllowablePosition(false);
    }

    private boolean isAllowablePosition(boolean withX) {
        if (desiredPose == null) {
            return false;
        }

        double drivetrainXPose = drivetrainSubsystem.getPose().getX();
        double drivetrainYPose = drivetrainSubsystem.getPose().getY();
        double drivetrainAngle = drivetrainSubsystem.getPose().getRotation().getRadians();
        double drivetrainPitch = drivetrainSubsystem.getGyroInputs().pitch;

        double desiredXPose = desiredPose.getX();
        double desiredYPose = desiredPose.getY();
        double desiredAngle = desiredPose.getRotation().getRadians();
        double desiredPitch = 0.0;

        double xError = drivetrainXPose - desiredXPose;
        double yError = Math.abs(drivetrainYPose - desiredYPose);
        double angleError = Math.abs(
                MathUtil.angleModulus(drivetrainAngle - Units.degreesToRadians(operatorDashboard.getRotationOffset()))
                        - MathUtil.angleModulus(desiredAngle));
        double pitchError = Math.abs(MathUtil.angleModulus(drivetrainPitch) - MathUtil.angleModulus(desiredPitch));

        Logger.getInstance().recordOutput("Auto Align/Angle error", angleError);
        Logger.getInstance().recordOutput("Auto Align/Y error", yError);
        Logger.getInstance().recordOutput("Auto Align/X error", xError);

        Logger.getInstance()
                .recordOutput(
                        "Auto Align/X allowable",
                        xError < ALLOWABLE_POSITIVE_X_POSE_ERROR && xError > ALLOWABLE_NEGATIVE_X_POSE_ERROR);
        Logger.getInstance().recordOutput("Auto Align/Y allowable", yError <= ALLOWABLE_Y_POSE_ERROR);
        Logger.getInstance().recordOutput("Auto Align/Angle allowable", angleError <= ALLOWABLE_ANGLE_ERROR);
        Logger.getInstance().recordOutput("Auto Align/Pitch allowable", pitchError <= ALLOWABLE_PITCH_ERROR);

        if (withX) {
            return autoEjectDebouncer.calculate(
                    (xError < ALLOWABLE_POSITIVE_X_POSE_ERROR && xError > ALLOWABLE_NEGATIVE_X_POSE_ERROR)
                            && yError <= ALLOWABLE_Y_POSE_ERROR
                            && angleError <= ALLOWABLE_ANGLE_ERROR
                            && pitchError <= ALLOWABLE_PITCH_ERROR);
        } else {
            return yError <= ALLOWABLE_Y_POSE_ERROR
                    && angleError <= ALLOWABLE_ANGLE_ERROR
                    && pitchError <= ALLOWABLE_PITCH_ERROR;
        }
    }

    private boolean isHorizontalTranslationAcceptable() {
        double drivetrainYPose = drivetrainSubsystem.getPose().getY();
        double desiredYPose = desiredPose.getY();

        double yError = Math.abs(drivetrainYPose - desiredYPose);

        return yError <= ALLOWABLE_Y_POSE_ERROR;
    }

    private boolean shouldResetAccumulator() {
        double drivetrainYPose = drivetrainSubsystem.getPose().getY();
        double desiredYPose = desiredPose.getY();

        double yError = Math.abs(drivetrainYPose - desiredYPose);

        return yError <= ALLOWABLE_Y_POSE_ERROR / 5 && yError >= ALLOWABLE_Y_POSE_ERROR * 2;
    }
}
