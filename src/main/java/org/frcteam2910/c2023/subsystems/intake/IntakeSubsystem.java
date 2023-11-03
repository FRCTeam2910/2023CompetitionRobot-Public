package org.frcteam2910.c2023.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.c2023.util.GamePiece;
import org.frcteam2910.c2023.util.OperatorDashboard;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private static final double BEAM_BREAK_THRESHOLD_VOLTAGE = 1.0;
    private static final double HAS_PIECE_CURRENT_THRESHOLD_AMPS = 5.0;
    private static final double HAS_PIECE_VELOCITY_THRESHOLD = 25.0;

    /**
     * Feature flag for using ToF or Beam Break sensor
     */
    private static final boolean IS_USING_TOF_SENSOR = true;

    private static final double INTAKE_WIDTH_METERS = Units.inchesToMeters(15.0);

    private static final double CONE_RADIUS_METERS = Units.inchesToMeters(7.5 / 2.0);

    private static final double MAX_TIMEOFFLIGHT_DISTANCE = INTAKE_WIDTH_METERS;

    private static final double TIMEOFFLIGHT_OFFSET = -(INTAKE_WIDTH_METERS / 2.0);

    private final OperatorDashboard operatorDashboard;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    private GamePiece targetPiece = GamePiece.CONE;
    private double intakeVoltage = 0;

    private TargetIntakeStates targetState = TargetIntakeStates.STOP;
    private CurrentIntakeStates currentState = CurrentIntakeStates.STOPPED;

    private double lastDetectedHasGamePieceTimestamp;
    private double lastAtLowAmpsTimestamp;

    public IntakeSubsystem(IntakeIO io, OperatorDashboard operatorDashboard) {
        this.io = io;
        this.operatorDashboard = operatorDashboard;
    }

    @Override
    public void periodic() {
        io.updateInputs(intakeInputs);
        Logger.getInstance().processInputs("Intake", intakeInputs);
        Logger.getInstance().recordOutput("Intake/Beam Break", hasGamePiece());
        Logger.getInstance().recordOutput("Intake/Target State", targetState.name());
        Logger.getInstance().recordOutput("Intake/Current State", currentState.name());
        Logger.getInstance().recordOutput("Intake/Has Piece (Velocity)", hasGamePiece());
        Logger.getInstance().recordOutput("Intake/Current Draw Piece", getCurrentDrawHasPiece());
        Logger.getInstance().recordOutput("Intake/Cone Offset", getConeOffset().getY());

        // setTargetPiece(operatorDashboard.getSelectedGamePiece());

        checkIntakeState(targetState);
        applyStates(currentState);
        // advanceStates(currentState);

        io.setMotorVoltage(intakeVoltage);

        Logger.getInstance().recordOutput("Target Piece", targetPiece.toString());
        Logger.getInstance()
                .recordOutput(
                        "Target Level", operatorDashboard.getSelectedGridLevel().name());
    }

    public void setTargetState(TargetIntakeStates targetState) {
        this.targetState = targetState;
    }

    private void checkIntakeState(TargetIntakeStates targetIntakeStates) {
        // reset offset every loop
        switch (targetIntakeStates) {
            case COLLECT:
                switch (targetPiece) {
                    case CONE:
                        currentState = CurrentIntakeStates.COLLECTING_CONE;
                        break;
                    case CUBE:
                        currentState = CurrentIntakeStates.COLLECTING_CUBE;
                        break;
                }
                break;
            case EJECT:
                switch (targetPiece) {
                    case CONE:
                        currentState = CurrentIntakeStates.EJECTING_CONE;
                        break;
                    case CUBE:
                        currentState = CurrentIntakeStates.EJECTING_CUBE;
                        break;
                    default:
                        break;
                }
                break;
            case HOLDING:
                switch (targetPiece) {
                    case CONE:
                        currentState = CurrentIntakeStates.HOLDING_CONE;
                        break;
                    case CUBE:
                        currentState = CurrentIntakeStates.HOLDING_CUBE;
                        break;
                }
                break;
            case STOP:
                currentState = CurrentIntakeStates.STOPPED;
                break;
            case FAST_EJECT:
                switch (targetPiece) {
                    case CONE:
                        currentState = CurrentIntakeStates.EJECTING_CONE;
                        break;
                    case CUBE:
                        currentState = CurrentIntakeStates.FAST_EJECTING_CUBE;
                        break;
                }
        }
    }

    private void advanceStates(CurrentIntakeStates currentState) {
        switch (currentState) {
                //            case HOLDING_CONE:
                //            case HOLDING_CUBE:
                //                if (Timer.getFPGATimestamp() - lastDetectedBeamBreakTimestamp > 0.25) {
                //                    this.currentState = CurrentIntakeStates.STOPPED;
                //                    this.targetState = TargetIntakeStates.STOP;
                //                }
                //                break;
            case COLLECTING_CONE:
                if (hasGamePiece()) {
                    this.currentState = CurrentIntakeStates.HOLDING_CONE;
                }
                break;
            case COLLECTING_CUBE:
                if (hasGamePiece() && Math.abs(lastAtLowAmpsTimestamp - Timer.getFPGATimestamp()) > 0.1) {
                    this.currentState = CurrentIntakeStates.HOLDING_CUBE;
                }
                break;
            case EJECTING_CONE:
            case EJECTING_CUBE:
                if (!hasGamePiece()) {
                    this.currentState = CurrentIntakeStates.STOPPED;
                    this.targetState = TargetIntakeStates.STOP;
                }
                break;
            default:
                break;
        }
    }

    private void applyStates(CurrentIntakeStates currentState) {
        switch (currentState) {
            case COLLECTING_CONE:
                intakeVoltage = -12;
                break;
            case COLLECTING_CUBE:
                if (intakeInputs.intakeCurrentDrawAmps < HAS_PIECE_CURRENT_THRESHOLD_AMPS) {
                    lastAtLowAmpsTimestamp = Timer.getFPGATimestamp();
                }
                intakeVoltage = 10;
                break;
            case EJECTING_CONE:
                //                if (operatorDashboard.getSelectedGridLevel() == GridLevel.LOW) {
                //                    intakeVoltage = 12.0;
                //                }
                //                intakeVoltage = 10.0;
                intakeVoltage = 12.0;
                break;
            case EJECTING_CUBE:
                intakeVoltage = -3.0;
                break;
            case HOLDING_CONE:
                if (hasGamePiece()) {
                    lastDetectedHasGamePieceTimestamp = Timer.getFPGATimestamp();
                }
                intakeVoltage = -1.0;
                break;
            case HOLDING_CUBE:
                if (hasGamePiece()) {
                    lastDetectedHasGamePieceTimestamp = Timer.getFPGATimestamp();
                }
                intakeVoltage = 0.7;
                break;
            case STOPPED:
                intakeVoltage = 0;
                break;
            case FAST_EJECTING_CUBE:
                intakeVoltage = -6.0;
                break;
        }
    }

    public void setTargetPiece(GamePiece targetPiece) {
        this.targetPiece = targetPiece;
    }

    public GamePiece getTargetPiece() {
        return targetPiece;
    }

    public boolean hasGamePiece() {
        return Math.abs(intakeInputs.intakeAngularVelocityRadPerSec) < HAS_PIECE_VELOCITY_THRESHOLD;
    }

    /**
     * Get the offset to the center of the cone in the robot relative frame.
     *
     * @return The offset to the center of the cone in the robot relative frame.
     */
    public Translation2d getConeOffset() {
        double y = 0.0;
        if (IS_USING_TOF_SENSOR) {
            // Get edge measurement and add the cone radius.
            var offset = intakeInputs.timeOfFlightMeters + CONE_RADIUS_METERS;
            // if mount location is in positive y direction subtract the offset,
            // otherwise add the offset.
            y = TIMEOFFLIGHT_OFFSET + offset;
        }
        return new Translation2d(
                0.0,
                MathUtil.clamp(
                        y,
                        -((INTAKE_WIDTH_METERS / 2) - CONE_RADIUS_METERS),
                        (INTAKE_WIDTH_METERS / 2) - CONE_RADIUS_METERS));
    }

    public boolean isHolding() {
        return currentState == CurrentIntakeStates.HOLDING_CONE || currentState == CurrentIntakeStates.HOLDING_CUBE;
    }

    public boolean getCurrentDrawHasPiece() {
        return intakeInputs.intakeCurrentDrawAmps > HAS_PIECE_CURRENT_THRESHOLD_AMPS;
    }

    public enum TargetIntakeStates {
        COLLECT,
        EJECT,
        HOLDING,
        STOP,
        FAST_EJECT,
    }

    private enum CurrentIntakeStates {
        COLLECTING_CUBE,
        HOLDING_CUBE,
        EJECTING_CUBE,
        COLLECTING_CONE,
        HOLDING_CONE,
        EJECTING_CONE,
        STOPPED,
        FAST_EJECTING_CUBE
    }
}
