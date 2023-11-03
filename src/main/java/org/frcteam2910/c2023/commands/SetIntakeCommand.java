package org.frcteam2910.c2023.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.intake.IntakeSubsystem;
import org.frcteam2910.c2023.util.GamePiece;
import org.frcteam2910.c2023.util.OperatorDashboard;

public class SetIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final XboxController controller;
    private final Supplier<IntakeSubsystem.TargetIntakeStates> targetStateSupplier;
    private final Supplier<GamePiece> pieceSupplier;
    private boolean withController;

    public SetIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            OperatorDashboard dashboard,
            XboxController controller,
            IntakeSubsystem.TargetIntakeStates targetState) {
        this(intakeSubsystem, dashboard, controller, () -> targetState);
        withController = true;
        addRequirements(intakeSubsystem);
    }

    public SetIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            OperatorDashboard dashboard,
            XboxController controller,
            Supplier<IntakeSubsystem.TargetIntakeStates> targetStateSupplier) {
        intake = intakeSubsystem;
        this.controller = controller;
        this.targetStateSupplier = targetStateSupplier;
        this.pieceSupplier = dashboard::getSelectedGamePiece;
        withController = true;
        addRequirements(intakeSubsystem);
    }

    public SetIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            XboxController controller,
            Supplier<IntakeSubsystem.TargetIntakeStates> targetStateSupplier,
            Supplier<GamePiece> pieceSupplier) {
        intake = intakeSubsystem;
        this.controller = controller;
        this.targetStateSupplier = targetStateSupplier;
        this.pieceSupplier = pieceSupplier;
        withController = true;
        addRequirements(intakeSubsystem);
    }

    public SetIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            Supplier<IntakeSubsystem.TargetIntakeStates> targetStateSupplier,
            Supplier<GamePiece> pieceSupplier) {
        intake = intakeSubsystem;
        this.controller = null;
        this.targetStateSupplier = targetStateSupplier;
        this.pieceSupplier = pieceSupplier;
        withController = false;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intake.setTargetState(targetStateSupplier.get());
        intake.setTargetPiece(pieceSupplier.get());
        if (controller != null) {
            if (intake.hasGamePiece() && withController) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 1);
            } else {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // if (targetStateSupplier.get() == IntakeSubsystem.TargetIntakeStates.COLLECT) {
        intake.setTargetState(IntakeSubsystem.TargetIntakeStates.HOLDING);
        // } else {
        // intake.setTargetState(IntakeSubsystem.TargetIntakeStates.STOP);
        // }
        if (controller != null) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}
