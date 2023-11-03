package org.frcteam2910.c2023.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.frcteam2910.c2023.Robot;
import org.frcteam2910.c2023.util.Interpolation;

public class ArmIOSim implements ArmIO {
    private static final DCMotor SHOULDER_MOTOR = DCMotor.getFalcon500(4).withReduction(100.0);
    private static final DCMotor WRIST_MOTOR = DCMotor.getFalcon500(1).withReduction(1.0);
    private static final DCMotor EXTENSION_MOTOR = DCMotor.getFalcon500(2).withReduction(1.0);

    private static final double SHOULDER_CURRENT_LIMIT_AMPS = 40.0;
    private static final double SHOULDER_MINIMUM_MOMENT_OF_INERTIA = 1.0;
    private static final double SHOULDER_MASS_KG = Units.lbsToKilograms(20);
    private static final Translation2d SHOULDER_CG_OFFSET = new Translation2d();
    private static final double SHOULDER_AXLE_OFFSET = 0;
    private static final double EXTENSION_CURRENT_LIMIT_AMPS = 20;
    private static final double EXTENSION_PULLEY_RADIUS_METERS = Units.inchesToMeters(3);
    private static final double EXTENSION_MINIMUM_MOMENT_OF_INERTIA = 1.0;
    private static final double EXTENSION_MAXIMUM_MOMENT_OF_INERTIA = 2.0;
    private static final double EXTENSION_MINIMUM_LENGTH_METERS = Units.inchesToMeters(20);
    private static final double EXTENSION_MAXIMUM_LENGTH_METERS = Units.inchesToMeters(40);
    private static final double EXTENSION_MASS_KG = Units.lbsToKilograms(20);
    private static final double EXTENSION_CG_OFFSET = Units.inchesToMeters(5);
    private static final Translation2d EXTENSION_MINIMUM_CG = new Translation2d();
    private static final Translation2d EXTENSION_MAXIMUM_CG = new Translation2d();
    private static final double WRIST_CURRENT_LIMIT_AMPS = 10;
    private static final double WRIST_MINIMUM_MOMENT_OF_INERTIA = 1.0;
    private static final double WRIST_MASS_KG = Units.lbsToKilograms(5);
    private static final Translation2d WRIST_CG_OFFSET = new Translation2d();
    private static final Translation2d WRIST_AXLE_OFFSET = new Translation2d();

    private final PIDController shoulderFeedback = new PIDController(1, 0, 0);
    private final PIDController wristFeedback = new PIDController(1, 0, 0);
    private final PIDController extensionFeedback = new PIDController(1, 0, 0);

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        Matrix<N6, N1> systemStates = VecBuilder.fill(
                inputs.shoulderAngleRad,
                inputs.shoulderAngularVelocityRadPerSec,
                inputs.extensionPositionMeters,
                inputs.extensionVelocityMetersPerSec,
                inputs.wristAngleRad,
                inputs.wristAngularVelocityRadPerSec);
        Matrix<N3, N1> systemInputs = VecBuilder.fill(
                shoulderFeedback.calculate(inputs.shoulderAngleRad),
                extensionFeedback.calculate(inputs.extensionPositionMeters),
                wristFeedback.calculate(inputs.wristAngleRad));
        Matrix<N6, N1> nextState =
                NumericalIntegration.rk4(this::calculateSystem, systemStates, systemInputs, Robot.defaultPeriodSecs);

        inputs.shoulderAngleRad = nextState.get(0, 0);
        inputs.shoulderAngularVelocityRadPerSec = nextState.get(1, 0);
        inputs.shoulderAppliedVolts = systemInputs.get(0, 0);
        inputs.shoulderCurrentDrawAmps = Math.min(
                Math.abs(SHOULDER_MOTOR.getCurrent(
                        inputs.shoulderAngularVelocityRadPerSec, inputs.shoulderAppliedVolts)),
                SHOULDER_CURRENT_LIMIT_AMPS);

        inputs.extensionPositionMeters = nextState.get(2, 0);
        inputs.extensionVelocityMetersPerSec = nextState.get(3, 0);
        inputs.extensionAppliedVolts = systemInputs.get(1, 0);
        inputs.extensionCurrentDrawAmps = Math.min(
                Math.abs(
                        EXTENSION_MOTOR.getCurrent(inputs.extensionVelocityMetersPerSec, inputs.extensionAppliedVolts)),
                EXTENSION_CURRENT_LIMIT_AMPS);

        inputs.wristAngleRad = nextState.get(4, 0);
        inputs.wristAngularVelocityRadPerSec = nextState.get(5, 0);
        inputs.wristAppliedVolts = systemInputs.get(2, 0);
        inputs.wristCurrentDrawAmps = Math.min(
                Math.abs(WRIST_MOTOR.getCurrent(inputs.wristAngularVelocityRadPerSec, inputs.wristAppliedVolts)),
                WRIST_CURRENT_LIMIT_AMPS);
    }

    private double calculateShoulderAngularMomentOfInertia(
            double shoulderAngle, double extensionLength, double wristAngle) {
        double t = Interpolation.inverseInterpolate(
                EXTENSION_MINIMUM_LENGTH_METERS, EXTENSION_MAXIMUM_LENGTH_METERS, extensionLength);
        double extensionMomentOfInertiaFunction =
                MathUtil.interpolate(EXTENSION_MINIMUM_MOMENT_OF_INERTIA, EXTENSION_MAXIMUM_MOMENT_OF_INERTIA, t);

        Translation2d extensionCG = EXTENSION_MINIMUM_CG.interpolate(EXTENSION_MAXIMUM_CG, t);
        double cgExtensionDistanceToShoulderPivot = extensionCG.getNorm();

        double wristMomentOfInertia = calculateWristAngularMomentOfInertia(shoulderAngle, extensionLength, wristAngle);
        double extensionMomentOfInertia = extensionMomentOfInertiaFunction
                + EXTENSION_MASS_KG * cgExtensionDistanceToShoulderPivot * cgExtensionDistanceToShoulderPivot;

        return wristMomentOfInertia + extensionMomentOfInertia;
    }

    private double calculateWristAngularMomentOfInertia(
            double shoulderAngle, double extensionLength, double wristAngle) {
        Translation2d wristToWristPivot = WRIST_AXLE_OFFSET.rotateBy(new Rotation2d(wristAngle));
        Translation2d wristToShoulderDistance = new Translation2d(extensionLength, SHOULDER_AXLE_OFFSET)
                .rotateBy(new Rotation2d(shoulderAngle))
                .plus(wristToWristPivot);
        return WRIST_MINIMUM_MOMENT_OF_INERTIA
                + WRIST_MASS_KG * wristToShoulderDistance.getNorm() * wristToShoulderDistance.getNorm();
    }

    private Matrix<N6, N1> calculateSystem(Matrix<N6, N1> states, Matrix<N3, N1> inputs) {
        // States
        double shoulderAngle = states.get(0, 0);
        double shoulderAngularVelocity = states.get(1, 0);

        double extensionLength = states.get(2, 0);
        double extensionVelocity = states.get(3, 0);

        double wristAngle = states.get(4, 0);
        double wristAngularVelocity = states.get(5, 0);

        // Inputs
        double shoulderVoltage = inputs.get(0, 0);
        double extensionVoltage = inputs.get(1, 0);
        double wristVoltage = inputs.get(2, 0);

        double shoulderCurrent = SHOULDER_MOTOR.getCurrent(shoulderAngularVelocity, shoulderVoltage);
        shoulderCurrent = MathUtil.clamp(shoulderCurrent, -SHOULDER_CURRENT_LIMIT_AMPS, SHOULDER_CURRENT_LIMIT_AMPS);
        double shoulderTorque = SHOULDER_MOTOR.getTorque(shoulderCurrent);
        double shoulderAngularAcceleration =
                shoulderTorque / calculateShoulderAngularMomentOfInertia(shoulderAngle, extensionLength, wristAngle);

        double extensionCurrent = EXTENSION_MOTOR.getCurrent(extensionVelocity, extensionVoltage);
        extensionCurrent =
                MathUtil.clamp(extensionCurrent, -EXTENSION_CURRENT_LIMIT_AMPS, EXTENSION_CURRENT_LIMIT_AMPS);
        double extensionTorque = EXTENSION_MOTOR.getTorque(extensionCurrent);
        double extensionForce = extensionTorque / EXTENSION_PULLEY_RADIUS_METERS;
        double extensionAcceleration = extensionForce / EXTENSION_MASS_KG;

        double wristCurrent = WRIST_MOTOR.getCurrent(wristAngularVelocity, wristVoltage);
        wristCurrent = MathUtil.clamp(wristCurrent, -WRIST_CURRENT_LIMIT_AMPS, WRIST_CURRENT_LIMIT_AMPS);
        double wristTorque = WRIST_MOTOR.getTorque(wristCurrent);
        double wristAngularAcceleration =
                wristTorque / calculateWristAngularMomentOfInertia(shoulderAngle, extensionLength, wristAngle);

        return VecBuilder.fill(
                shoulderAngularVelocity,
                shoulderAngularAcceleration,
                extensionVelocity,
                extensionAcceleration,
                wristAngularVelocity,
                wristAngularAcceleration);
    }
}
