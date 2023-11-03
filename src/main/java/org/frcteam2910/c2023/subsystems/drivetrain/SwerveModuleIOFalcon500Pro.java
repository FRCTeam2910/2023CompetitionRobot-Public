package org.frcteam2910.c2023.subsystems.drivetrain;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * Implementation of the Swerve Module IO class. We use the Falcon500 motors to directly link the software to the motors.
 * For use with the SDS MK4i modules only.
 */
public class SwerveModuleIOFalcon500Pro implements SwerveModuleIO {
    public static final int DRIVE_CURRENT_LIMIT = 70;
    public static final int SECONDARY_DRIVE_CURRENT_LIMIT = 30;
    public static final int STEER_CURRENT_LIMIT = 30;

    // Used to calculate feed forward for turn speed in 2nd order dynamics calc.
    public static final double TURN_kA = 0;
    public static final double TURN_kS = 0.05;
    public static final double TURN_kV = 0.1;

    // Constants specific to the hardware
    /**
     * Radius of the wheel. Can be used to figure out distance data
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.95);
    /**
     * From motor rotations to the wheel revolutions
     */
    private static final double DRIVE_GEAR_RATIO = (50.0 / 18.0) * (16.0 / 28.0) * (45.0 / 15.0);

    private static final double DRIVE_GEAR_RATIO_SECONDARY = (50.0 / 24.0) * (19.0 / 25.0) * (45.0 / 15.0);
    /**
     * Conversion constant: From motor encoder ticks to position data (m)
     * <p>
     * motor encoder ticks -> meters: motor encoder ticks * constant
     * meters -> motor encoder ticks: meters / constant
     */
    private static final double DRIVE_SENSOR_POSITION_COEFFICIENT =
            (2 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO;

    private static final double DRIVE_SENSOR_POSITION_COEFFICIENT_SECONDARY =
            (2 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO_SECONDARY;

    /**
     * Conversion constant: From motor encoder ticks to velocity data (m/s)
     * <p>
     * motor encoder ticks / 100 ms -> meters per second: motor encoder ticks * constant
     * meters per second -> motor encoder ticks / 100 ms: meters per second / constant
     */
    private static final double DRIVE_SENSOR_VELOCITY_COEFFICIENT = DRIVE_SENSOR_POSITION_COEFFICIENT;

    private static final double DRIVE_SENSOR_VELOCITY_COEFFICIENT_SECONDARY =
            DRIVE_SENSOR_POSITION_COEFFICIENT_SECONDARY;
    /**
     * From motor rotations to the wheel rotations (150 / 7 motor rotations : 1 full rotation of the wheel [2π])
     */
    private static final double STEER_GEAR_RATIO = 150.0 / 7;
    /**
     * Conversion constant: From motor encoder ticks to angle data (steer position) (radians)
     * <p>
     * motor encoder ticks -> radians: motor encoder ticks * constant
     * radians -> motor encoder ticks: radians / constant
     */
    private static final double STEER_SENSOR_POSITION_COEFFICIENT = (2 * Math.PI) / (2048 * STEER_GEAR_RATIO);
    /**
     * Conversion constant: From motor encoder ticks to angular velocity data (steer velocity) (radians/s)
     * <p>
     * motor encoder ticks / 100 ms -> radians per second: motor encoder ticks / 100 ms * constant
     * radians per second -> motor encoder ticks / 100 ms: radians per second / constant
     */
    private static final double STEER_SENSOR_VELOCITY_COEFFICIENT = STEER_SENSOR_POSITION_COEFFICIENT * 10;

    private static final double VELOCITY_COEFFICIENT = 1.8;

    // Hardware object initialization
    /**
     * TalonFX swerve module drive motor
     */
    private final TalonFX primaryDriveMotor;

    private final TalonFX secondaryDriveMotor;
    /**
     * TalonFX swerve module steer motor
     */
    private final TalonFX steerMotor;
    /**
     * Swerve module steer encoder (absolute angular position)
     */
    private final CANcoder steerEncoder;

    // Target Variables. Used only for data logging
    private double targetVelocityMetersPerSeconds = 0;
    private double targetSteerAngleRadians = 0;
    private ArmFeedforward turnFFController;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0, true, 0, 0, false);
    private final VoltageOut voltageControl = new VoltageOut(0, true, false);
    private final PositionVoltage steerPositionControl = new PositionVoltage(0, false, 0, 0, false);

    private StatusSignalValue<Double> primaryDrivePositionSignal;
    private StatusSignalValue<Double> primaryDriveVelocitySignal;
    private StatusSignalValue<Double> secondaryDrivePosition;
    private StatusSignalValue<Double> secondaryDriveVelocity;
    private StatusSignalValue<Double> steerPositionSignal;
    private StatusSignalValue<Double> steerVelocitySignal;
    private BaseStatusSignalValue[] signals;

    private final boolean usingDoubleDriveMotor;

    /**
     * Initializes the motors, encoder, and the settings for each of the devices.
     *
     * @param driveMotorId        Drive motor CAN ID
     * @param steerMotorId        Steer motor CAN ID
     * @param steerEncoderId      Steer encoder CAN ID
     * @param canBus              The name of the CAN bus the device is connected to.
     * @param steerAngleOffsetRad This is the offset applied to the angle motor's absolute encoder so that a reading of 0 degrees means the module is facing forwards.
     */
    public SwerveModuleIOFalcon500Pro(
            int driveMotorId,
            int steerMotorId,
            int steerEncoderId,
            String canBus,
            double steerAngleOffsetRad,
            boolean usingDoubleDriveMotor,
            int secondaryDriveMotorId) {
        this.usingDoubleDriveMotor = usingDoubleDriveMotor;

        primaryDriveMotor = new TalonFX(driveMotorId, canBus);
        steerMotor = new TalonFX(steerMotorId, canBus);

        steerEncoder = new CANcoder(steerEncoderId, canBus);

        FeedbackConfigs feedBackConfigs = new FeedbackConfigs();
        feedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
        driveCurrentLimitsConfigs.SupplyCurrentLimit =
                usingDoubleDriveMotor ? SECONDARY_DRIVE_CURRENT_LIMIT : DRIVE_CURRENT_LIMIT;
        driveCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback = feedBackConfigs;
        driveConfig.CurrentLimits = driveCurrentLimitsConfigs;
        driveConfig.MotorOutput = motorOutputConfigs;
        driveConfig.Slot0.kP = 0.0;
        driveConfig.Slot0.kV = 1023.0
                / ((12.0 / VELOCITY_COEFFICIENT)
                        / (usingDoubleDriveMotor
                                ? DRIVE_SENSOR_VELOCITY_COEFFICIENT_SECONDARY
                                : DRIVE_SENSOR_VELOCITY_COEFFICIENT));

        primaryDriveMotor.getConfigurator().apply(driveConfig);

        if (usingDoubleDriveMotor) {
            secondaryDriveMotor = new TalonFX(secondaryDriveMotorId, canBus);
            secondaryDriveMotor.getConfigurator().apply(driveConfig);
            secondaryDriveMotor.setControl(new Follower(primaryDriveMotor.getDeviceID(), false));
        } else {
            secondaryDriveMotor = null;
        }

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        //        steerConfig.
        //        steerConfig.primaryPID.selectedFeedbackSensor =
        // TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        //        steerConfig.slot0.kP = 0.2;
        //        steerConfig.supplyCurrLimit.currentLimit = STEER_CURRENT_LIMIT;
        //        steerConfig.supplyCurrLimit.enable = true;

        FeedbackConfigs feedBackSteerConfigs = new FeedbackConfigs();
        feedBackSteerConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedBackSteerConfigs.FeedbackRemoteSensorID = steerEncoderId;
        feedBackSteerConfigs.RotorToSensorRatio = STEER_GEAR_RATIO;

        CurrentLimitsConfigs steerCurrentLimitsConfigs = new CurrentLimitsConfigs();
        steerCurrentLimitsConfigs.SupplyCurrentLimit = STEER_CURRENT_LIMIT;
        steerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 100;

        steerConfig.Feedback = feedBackSteerConfigs;
        steerConfig.CurrentLimits = steerCurrentLimitsConfigs;
        steerConfig.Slot0 = slot0Configs;

        MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
        steerMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerConfig.MotorOutput = steerMotorOutputConfigs;

        steerMotor.getConfigurator().apply(steerConfig);

        CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs encoderMagnetSensorConfigs = new MagnetSensorConfigs();
        encoderMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderMagnetSensorConfigs.MagnetOffset = Units.radiansToRotations(steerAngleOffsetRad);
        steerEncoderConfig.MagnetSensor = encoderMagnetSensorConfigs;

        steerEncoder.getConfigurator().apply(steerEncoderConfig);

        turnFFController = new ArmFeedforward(TURN_kS, 0.0, TURN_kV, TURN_kA);

        primaryDrivePositionSignal = primaryDriveMotor.getPosition();
        primaryDriveVelocitySignal = primaryDriveMotor.getVelocity();
        steerPositionSignal = steerEncoder.getPosition();
        steerVelocitySignal = steerEncoder.getVelocity();

        signals = new BaseStatusSignalValue[4];

        signals[0] = primaryDrivePositionSignal;
        signals[1] = primaryDriveVelocitySignal;
        signals[2] = steerPositionSignal;
        signals[3] = steerVelocitySignal;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        double driveAppliedVolts = primaryDriveMotor.getSupplyVoltage().getValue();
        double steerAppliedVolts = steerMotor.getSupplyVoltage().getValue();

        inputs.drivePositionMeters =
                BaseStatusSignalValue.getLatencyCompensatedValue(primaryDrivePositionSignal, primaryDriveVelocitySignal)
                        * (usingDoubleDriveMotor
                                ? DRIVE_SENSOR_POSITION_COEFFICIENT_SECONDARY
                                : DRIVE_SENSOR_POSITION_COEFFICIENT);
        ;
        inputs.driveVelocityMetersPerSec = primaryDriveVelocitySignal.getValue()
                * (usingDoubleDriveMotor
                        ? DRIVE_SENSOR_VELOCITY_COEFFICIENT_SECONDARY
                        : DRIVE_SENSOR_VELOCITY_COEFFICIENT);
        ;
        inputs.driveCurrentDrawAmps = primaryDriveMotor.getSupplyCurrent().getValue();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;

        inputs.steerPositionRad = Units.rotationsToRadians(
                BaseStatusSignalValue.getLatencyCompensatedValue(steerPositionSignal, steerVelocitySignal));
        inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocitySignal.getValue());
        inputs.steerCurrentDrawAmps = steerMotor.getSupplyCurrent().getValue();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.targetSteerPositionRad = targetSteerAngleRadians;

        inputs.steerAbsolutePosition =
                Units.rotationsToRadians(steerEncoder.getAbsolutePosition().getValue());
    }

    @Override
    public void setTargetSteerPosition(double targetSteerPositionRad) {
        steerMotor.setControl(steerPositionControl
                .withPosition(Units.radiansToRotations(targetSteerPositionRad))
                .withFeedForward(0));
        this.targetSteerAngleRadians = targetSteerPositionRad;
    }

    @Override
    public void setTargetSteerAngle(double targetSteerAngleRadians, double turnSpeed) {
        final double turnArbFFComponent = turnFFController.calculate(targetSteerAngleRadians, turnSpeed);
        steerMotor.setControl(steerPositionControl
                .withPosition(Units.radiansToRotations(targetSteerAngleRadians))
                .withFeedForward(turnArbFFComponent));
        this.targetSteerAngleRadians = targetSteerAngleRadians;
    }

    @Override
    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
        primaryDriveMotor.setControl(
                voltageControl.withOutput((targetDriveVelocityMetersPerSec / getMaxVelocity()) * 12));

        Logger.getInstance().recordOutput("Voltage", (targetDriveVelocityMetersPerSec / getMaxVelocity()) * 12);

        this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
    }

    @Override
    public void resetToAbsoluteAngle() {}

    @Override
    public double getMaxVelocity() {
        return 12.0 / VELOCITY_COEFFICIENT;
    }

    @Override
    public BaseStatusSignalValue[] getSignals() {
        return signals;
    }
}
