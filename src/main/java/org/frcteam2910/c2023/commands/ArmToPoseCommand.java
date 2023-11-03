package org.frcteam2910.c2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.arm.ArmSubsystem;
import org.frcteam2910.c2023.subsystems.intake.IntakeSubsystem;
import org.frcteam2910.c2023.util.ArmPositions;
import org.frcteam2910.c2023.util.GamePiece;
import org.frcteam2910.c2023.util.GridLevel;
import org.frcteam2910.c2023.util.OperatorDashboard;

public class ArmToPoseCommand extends CommandBase {
    private static final double HIGH_SHOULDER_OFFSET = Units.degreesToRadians(1.0);
    private static final double MID_SHOULDER_OFFSET = Units.degreesToRadians(1.5);

    private static final double HIGH_EXTENSION_OFFSET = Units.inchesToMeters(0.0);
    private static final double MID_EXTENSION_OFFSET = Units.inchesToMeters(0.0);

    private static final double HIGH_WRIST_OFFSET = Units.degreesToRadians(0.0);
    private static final double MID_WRIST_OFFSET = Units.degreesToRadians(0.0);

    private final ArmSubsystem arm;
    private Supplier<ArmPositions> targetPoseSupplier;
    private final boolean perpetual;
    private final BooleanSupplier aButton;
    private final OperatorDashboard dashboard;
    private final IntakeSubsystem intake;

    public ArmToPoseCommand(ArmSubsystem arm, ArmPositions targetPose) {
        this(arm, () -> targetPose, false, () -> false, null, null);
    }

    public ArmToPoseCommand(ArmSubsystem arm, Supplier<ArmPositions> targetPoseSupplier) {
        this(arm, targetPoseSupplier, false, () -> false, null, null);
    }

    public ArmToPoseCommand(ArmSubsystem arm, Supplier<ArmPositions> targetPoseSupplier, boolean perpetual) {
        this(arm, targetPoseSupplier, perpetual, () -> false, null, null);
    }

    public ArmToPoseCommand(
            ArmSubsystem arm,
            Supplier<ArmPositions> targetPoseSupplier,
            boolean perpetual,
            BooleanSupplier aButton,
            OperatorDashboard dashboard,
            IntakeSubsystem intake) {
        this.arm = arm;
        this.targetPoseSupplier = targetPoseSupplier;
        this.perpetual = perpetual;
        this.aButton = aButton;
        this.dashboard = dashboard;
        this.intake = intake;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        ArmPositions targetPose;
        targetPose = targetPoseSupplier.get();
        if (dashboard != null) {
            if (aButton.getAsBoolean()
                    && intake.getTargetPiece() == GamePiece.CONE
                    && (dashboard.getSelectedGridLevel() == GridLevel.HIGH
                            || dashboard.getSelectedGridLevel() == GridLevel.MIDDLE)) {
                targetPose = new ArmPositions(
                        targetPose.getShoulderAngleRad()
                                - (dashboard.getSelectedGridLevel() == GridLevel.HIGH
                                        ? HIGH_SHOULDER_OFFSET
                                        : MID_SHOULDER_OFFSET),
                        targetPose.getExtensionLengthMeters()
                                + (dashboard.getSelectedGridLevel() == GridLevel.HIGH
                                        ? HIGH_EXTENSION_OFFSET
                                        : MID_EXTENSION_OFFSET),
                        targetPose.getWristAngleRad()
                                - (dashboard.getSelectedGridLevel() == GridLevel.HIGH
                                        ? HIGH_WRIST_OFFSET
                                        : MID_WRIST_OFFSET));
            }
        }
        arm.setTargetPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        if (perpetual) {
            return false;
        } else {
            return arm.atTarget();
        }
    }

    public void setTargetPoseSupplier(Supplier<ArmPositions> targetPoseSupplier) {
        this.targetPoseSupplier = targetPoseSupplier;
    }
}
