package org.frcteam2910.c2023.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.math.util.Units;
import org.frcteam2910.c2023.config.PortConfiguration;

public class ArmIOFalcon500 implements ArmIO {
    private static final double SHOULDER_GEAR_RATIO = (60.0 / 12.0) * (50.0 / 36.0) * (50.0 / 18.0) * (58.0 / 10.0);
    private static final double SHOULDER_POSITION_COEFFICIENT = (2 * Math.PI) / (2048 * SHOULDER_GEAR_RATIO);
    private static final double SHOULDER_VELOCITY_COEFFICIENT = SHOULDER_POSITION_COEFFICIENT * 10;

    public static final double SHOULDER_SLOW_ACCELERATION = Units.degreesToRadians(500);
    public static final double SHOULDER_FAST_ACCELERATION = Units.degreesToRadians(750);
    public static final double SHOULDER_VELOCITY = Units.degreesToRadians(300);

    private static final double SHOULDER_SLOW_ACCELERATION_CONSTRAINT =
            SHOULDER_SLOW_ACCELERATION / SHOULDER_VELOCITY_COEFFICIENT;
    private static final double SHOULDER_FAST_ACCELERATION_CONSTRAINT =
            SHOULDER_FAST_ACCELERATION / SHOULDER_VELOCITY_COEFFICIENT;
    private static final double SHOULDER_VELOCITY_CONSTRAINT = SHOULDER_VELOCITY / SHOULDER_VELOCITY_COEFFICIENT;

    private static final double EXTENSION_GEAR_RATIO = 64.0 / 15.0;
    private static final double EXTENSION_PULLEY_DIAMETER = Units.inchesToMeters(1.504);
    private static final double EXTENSION_POSITION_COEFFICIENT =
            (Math.PI * EXTENSION_PULLEY_DIAMETER) / (2048 * EXTENSION_GEAR_RATIO) * 2;
    private static final double EXTENSION_VELOCITY_COEFFICIENT = EXTENSION_POSITION_COEFFICIENT * 10;

    private static final double WRIST_GEAR_RATIO = (50.0 / 9.0) * (40.0 / 15.0) * (40.0 / 15.0);
    private static final double WRIST_POSITION_COEFFICIENT = (2 * Math.PI) / (2048 * WRIST_GEAR_RATIO);
    private static final double WRIST_VELOCITY_COEFFICIENT = WRIST_POSITION_COEFFICIENT * 10;

    private boolean isFastShoulderAcceleration = false;
    private boolean isHighExtensionCurrentLimit = true;

    private final TalonFX shoulderOne;
    private final TalonFX shoulderTwo;
    private final TalonFX shoulderThree;
    private final TalonFX shoulderFour;

    private final TalonFX extensionOne;
    private final TalonFX extensionTwo;

    private final TalonFX wrist;

    public ArmIOFalcon500(PortConfiguration configuration) {
        shoulderOne = new TalonFX(configuration.shoulderMotorOneID, configuration.CANBus);
        shoulderTwo = new TalonFX(configuration.shoulderMotorTwoID, configuration.CANBus);
        shoulderThree = new TalonFX(configuration.shoulderMotorThreeID, configuration.CANBus);
        shoulderFour = new TalonFX(configuration.shoulderMotorFourID, configuration.CANBus);

        extensionOne = new TalonFX(configuration.armMotorOneID, configuration.ArmCANBus);
        extensionTwo = new TalonFX(configuration.armMotorTwoID, configuration.ArmCANBus);

        wrist = new TalonFX(configuration.wristMotorID, configuration.ArmCANBus);

        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.supplyCurrLimit.currentLimit = 25;
        shoulderConfig.supplyCurrLimit.enable = true;
        shoulderConfig.voltageCompSaturation = 12;
        shoulderConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        shoulderConfig.slot0.kP = 0.1;
        shoulderConfig.slot0.kI = 0.0;
        shoulderConfig.slot0.kD = 0.0;
        shoulderConfig.motionAcceleration = SHOULDER_SLOW_ACCELERATION_CONSTRAINT;
        shoulderConfig.motionCruiseVelocity = SHOULDER_VELOCITY_CONSTRAINT;
        //        shoulderConfig.motionCurveStrength = 8;

        shoulderOne.configAllSettings(shoulderConfig);
        shoulderTwo.configAllSettings(shoulderConfig);
        shoulderThree.configAllSettings(shoulderConfig);
        shoulderFour.configAllSettings(shoulderConfig);

        shoulderTwo.follow(shoulderOne);
        shoulderThree.follow(shoulderOne);
        shoulderFour.follow(shoulderOne);

        shoulderOne.setSensorPhase(true);
        shoulderTwo.setSensorPhase(true);
        shoulderThree.setSensorPhase(true);
        shoulderFour.setSensorPhase(true);
        shoulderOne.setInverted(TalonFXInvertType.CounterClockwise);
        shoulderTwo.setInverted(TalonFXInvertType.CounterClockwise);
        shoulderThree.setInverted(TalonFXInvertType.Clockwise);
        shoulderFour.setInverted(TalonFXInvertType.Clockwise);

        shoulderTwo.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        shoulderTwo.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        shoulderThree.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        shoulderThree.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        shoulderFour.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        shoulderFour.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        shoulderOne.setNeutralMode(NeutralMode.Brake);
        shoulderTwo.setNeutralMode(NeutralMode.Brake);
        shoulderThree.setNeutralMode(NeutralMode.Brake);
        shoulderFour.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        extensionConfig.supplyCurrLimit.currentLimit = 30;
        extensionConfig.supplyCurrLimit.enable = true;
        extensionConfig.voltageCompSaturation = 12;
        extensionConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        extensionConfig.slot0.kP = 0.5;
        extensionConfig.slot0.kI = 0.0;
        extensionConfig.slot0.kD = 0.0;
        extensionConfig.motionAcceleration = Units.inchesToMeters(300) / EXTENSION_VELOCITY_COEFFICIENT;
        extensionConfig.motionCruiseVelocity = Units.inchesToMeters(100) / EXTENSION_VELOCITY_COEFFICIENT;

        extensionOne.configAllSettings(extensionConfig);
        extensionTwo.configAllSettings(extensionConfig);

        extensionTwo.follow(extensionOne);

        extensionOne.setSensorPhase(true);
        extensionTwo.setSensorPhase(true);
        extensionOne.setInverted(TalonFXInvertType.Clockwise);
        extensionTwo.setInverted(TalonFXInvertType.FollowMaster);

        extensionTwo.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        extensionTwo.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        extensionOne.setNeutralMode(NeutralMode.Brake);
        extensionTwo.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.supplyCurrLimit.currentLimit = 30;
        wristConfig.supplyCurrLimit.enable = true;
        wristConfig.voltageCompSaturation = 12;
        wristConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        wristConfig.slot0.kP = 0.5;
        wristConfig.slot0.kI = 0.0;
        wristConfig.slot0.kD = 0.0;
        wristConfig.motionAcceleration = Units.degreesToRadians(5000) / WRIST_VELOCITY_COEFFICIENT;
        wristConfig.motionCruiseVelocity = Units.degreesToRadians(1000) / WRIST_VELOCITY_COEFFICIENT;

        wrist.configAllSettings(wristConfig);

        wrist.setSensorPhase(true);
        wrist.setInverted(TalonFXInvertType.CounterClockwise);

        wrist.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderAngleRad = shoulderOne.getSelectedSensorPosition() * SHOULDER_POSITION_COEFFICIENT;
        inputs.shoulderAngularVelocityRadPerSec =
                shoulderOne.getSelectedSensorVelocity() * SHOULDER_VELOCITY_COEFFICIENT;
        inputs.shoulderAppliedVolts = shoulderOne.getMotorOutputVoltage();
        inputs.shoulderCurrentDrawAmps = shoulderOne.getSupplyCurrent();

        inputs.extensionPositionMeters = extensionOne.getSelectedSensorPosition() * EXTENSION_POSITION_COEFFICIENT;
        inputs.extensionVelocityMetersPerSec =
                extensionOne.getSelectedSensorVelocity() * EXTENSION_VELOCITY_COEFFICIENT;
        inputs.extensionAppliedVolts = extensionOne.getMotorOutputVoltage();
        inputs.extensionCurrentDrawAmps = extensionOne.getSupplyCurrent();

        inputs.wristAngleRad = wrist.getSelectedSensorPosition() * WRIST_POSITION_COEFFICIENT;
        inputs.wristAngularVelocityRadPerSec = wrist.getSelectedSensorVelocity() * WRIST_VELOCITY_COEFFICIENT;
        inputs.wristAppliedVolts = wrist.getMotorOutputVoltage();
        inputs.wristCurrentDrawAmps = wrist.getSupplyCurrent();

        inputs.shoulderMotorOneTemp = shoulderOne.getTemperature();
        inputs.shoulderMotorTwoTemp = shoulderTwo.getTemperature();
        inputs.shoulderMotorThreeTemp = shoulderThree.getTemperature();
        inputs.shoulderMotorFourTemp = shoulderFour.getTemperature();

        inputs.extensionMotorOneTemp = extensionOne.getTemperature();
        inputs.extensionMotorTwoTemp = extensionTwo.getTemperature();

        inputs.wristMotorTemp = wrist.getTemperature();

        inputs.isFastShoulderAcceleration = isFastShoulderAcceleration;
        inputs.isHighExtensionCurrentLimit = isHighExtensionCurrentLimit;
    }

    @Override
    public void setTargetShoulderAngle(double angleRad) {
        shoulderOne.set(TalonFXControlMode.MotionMagic, angleRad / SHOULDER_POSITION_COEFFICIENT);
    }

    @Override
    public void setTargetExtensionLength(double lengthMeters) {
        extensionOne.set(TalonFXControlMode.MotionMagic, lengthMeters / EXTENSION_POSITION_COEFFICIENT);
    }

    @Override
    public void setTargetWristAngle(double angleRad) {
        wrist.set(ControlMode.MotionMagic, angleRad / WRIST_POSITION_COEFFICIENT);
    }

    @Override
    public void setShoulderVoltage(double voltage) {
        shoulderOne.set(ControlMode.PercentOutput, voltage / 12.0);
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        extensionOne.set(ControlMode.PercentOutput, voltage / 12.0);
    }

    @Override
    public void setWristVoltage(double voltage) {
        wrist.set(ControlMode.PercentOutput, voltage / 12.0);
    }

    @Override
    public void setSensorPositionFromMeasurement(ArmJoint joint, double position) {
        switch (joint) {
            case SHOULDER:
                shoulderOne.setSelectedSensorPosition(position / SHOULDER_POSITION_COEFFICIENT);
                break;
            case EXTENSION:
                extensionOne.setSelectedSensorPosition(position / EXTENSION_POSITION_COEFFICIENT);
                break;
            case WRIST:
                wrist.setSelectedSensorPosition(position / WRIST_POSITION_COEFFICIENT);
                break;
        }
    }

    @Override
    public void setNeutralMode(boolean brake) {
        if (brake) {
            shoulderOne.setNeutralMode(NeutralMode.Brake);
            shoulderTwo.setNeutralMode(NeutralMode.Brake);
            shoulderThree.setNeutralMode(NeutralMode.Brake);
            shoulderFour.setNeutralMode(NeutralMode.Brake);

            extensionOne.setNeutralMode(NeutralMode.Brake);
            extensionTwo.setNeutralMode(NeutralMode.Brake);

            wrist.setNeutralMode(NeutralMode.Brake);
        } else {
            shoulderOne.setNeutralMode(NeutralMode.Coast);
            shoulderTwo.setNeutralMode(NeutralMode.Coast);
            shoulderThree.setNeutralMode(NeutralMode.Coast);
            shoulderFour.setNeutralMode(NeutralMode.Coast);

            extensionOne.setNeutralMode(NeutralMode.Coast);
            extensionTwo.setNeutralMode(NeutralMode.Coast);

            wrist.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void setShoulderAccelerationConstraint(boolean fastConstraints) {
        isFastShoulderAcceleration = fastConstraints;
        if (fastConstraints) {
            shoulderOne.configMotionAcceleration(SHOULDER_FAST_ACCELERATION_CONSTRAINT);
            shoulderTwo.configMotionAcceleration(SHOULDER_FAST_ACCELERATION_CONSTRAINT);
            shoulderThree.configMotionAcceleration(SHOULDER_FAST_ACCELERATION_CONSTRAINT);
            shoulderFour.configMotionAcceleration(SHOULDER_FAST_ACCELERATION_CONSTRAINT);
        } else {
            shoulderOne.configMotionAcceleration(SHOULDER_SLOW_ACCELERATION_CONSTRAINT);
            shoulderTwo.configMotionAcceleration(SHOULDER_SLOW_ACCELERATION_CONSTRAINT);
            shoulderThree.configMotionAcceleration(SHOULDER_SLOW_ACCELERATION_CONSTRAINT);
            shoulderFour.configMotionAcceleration(SHOULDER_SLOW_ACCELERATION_CONSTRAINT);
        }
    }

    @Override
    public void setExtensionCurrentLimit(boolean highCurrent) {
        //        isHighExtensionCurrentLimit = highCurrent;
        //        if (highCurrent) {
        //            extensionOne.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
        //            extensionTwo.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
        //        } else {
        //            extensionOne.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 0, 0));
        //            extensionTwo.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 0, 0));
        //        }
    }
}
