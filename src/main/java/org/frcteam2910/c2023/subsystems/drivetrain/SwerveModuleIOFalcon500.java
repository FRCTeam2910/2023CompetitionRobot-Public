package org.frcteam2910.c2023.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

/**
 * Implementation of the Swerve Module IO class. We use the Falcon500 motors to directly link the software to the motors.
 * For use with the SDS MK4i modules only.
 */
public class SwerveModuleIOFalcon500 implements SwerveModuleIO {
    public static final int DRIVE_CURRENT_LIMIT = 70;
    public static final int STEER_CURRENT_LIMIT = 30;

    // Constants specific to the hardware
    /**
     * Radius of the wheel. Can be used to figure out distance data
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.95);
    /**
     * From motor rotations to the wheel revolutions
     */
    private static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    /**
     * Conversion constant: From motor encoder ticks to position data (m)
     * <p>
     * motor encoder ticks -> meters: motor encoder ticks * constant
     * meters -> motor encoder ticks: meters / constant
     */
    private static final double DRIVE_SENSOR_POSITION_COEFFICIENT =
            (2 * Math.PI * WHEEL_RADIUS_METERS) / (2048 * DRIVE_GEAR_RATIO);
    /**
     * Conversion constant: From motor encoder ticks to velocity data (m/s)
     * <p>
     * motor encoder ticks / 100 ms -> meters per second: motor encoder ticks * constant
     * meters per second -> motor encoder ticks / 100 ms: meters per second / constant
     */
    private static final double DRIVE_SENSOR_VELOCITY_COEFFICIENT = DRIVE_SENSOR_POSITION_COEFFICIENT * 10;
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

    private static final double VELOCITY_COEFFICIENT = 2;

    // Hardware object initialization
    /**
     * TalonFX swerve module drive motor
     */
    private final TalonFX driveMotor;
    /**
     * TalonFX swerve module steer motor
     */
    private final TalonFX steerMotor;
    /**
     * Swerve module steer encoder (absolute angular position)
     */
    private final CANCoder steerEncoder;

    // Target Variables. Used only for data logging
    private double targetVelocityMetersPerSeconds = 0;
    private double targetSteerPositionRad = 0;

    /**
     * Initializes the motors, encoder, and the settings for each of the devices.
     *
     * @param driveMotorId        Drive motor CAN ID
     * @param steerMotorId        Steer motor CAN ID
     * @param steerEncoderId      Steer encoder CAN ID
     * @param canBus              The name of the CAN bus the device is connected to.
     * @param steerAngleOffsetRad This is the offset applied to the angle motor's absolute encoder so that a reading of 0 degrees means the module is facing forwards.
     */
    public SwerveModuleIOFalcon500(
            int driveMotorId, int steerMotorId, int steerEncoderId, String canBus, double steerAngleOffsetRad) {
        driveMotor = new TalonFX(driveMotorId, canBus);
        steerMotor = new TalonFX(steerMotorId, canBus);

        steerEncoder = new CANCoder(steerEncoderId, canBus);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        driveConfig.slot0.kP = 0.00;
        driveConfig.slot0.kF = 1023.0 / ((12.0 / VELOCITY_COEFFICIENT) / DRIVE_SENSOR_VELOCITY_COEFFICIENT);
        driveConfig.supplyCurrLimit.currentLimit = DRIVE_CURRENT_LIMIT;
        driveConfig.supplyCurrLimit.enable = true;

        driveMotor.configAllSettings(driveConfig);
        driveMotor.setInverted(TalonFXInvertType.Clockwise);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        steerConfig.slot0.kP = 0.2;
        steerConfig.supplyCurrLimit.currentLimit = STEER_CURRENT_LIMIT;
        steerConfig.supplyCurrLimit.enable = true;

        steerMotor.configAllSettings(steerConfig);
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(TalonFXInvertType.Clockwise);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        CANCoderConfiguration steerEncoderConfig = new CANCoderConfiguration();
        steerEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        steerEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        steerEncoderConfig.magnetOffsetDegrees = Units.radiansToDegrees(steerAngleOffsetRad);

        steerEncoder.configAllSettings(steerEncoderConfig);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        double driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        double steerAppliedVolts = steerMotor.getMotorOutputVoltage();

        inputs.drivePositionMeters = driveMotor.getSelectedSensorPosition() * DRIVE_SENSOR_POSITION_COEFFICIENT;
        inputs.driveVelocityMetersPerSec = driveMotor.getSelectedSensorVelocity() * DRIVE_SENSOR_VELOCITY_COEFFICIENT;
        inputs.driveCurrentDrawAmps = driveMotor.getSupplyCurrent();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;

        inputs.steerPositionTicks = steerMotor.getSelectedSensorPosition();
        inputs.steerPositionRad = inputs.steerPositionTicks * STEER_SENSOR_POSITION_COEFFICIENT;
        inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
        inputs.steerVelocityRadPerSec = steerMotor.getSelectedSensorVelocity() * STEER_SENSOR_VELOCITY_COEFFICIENT;
        inputs.steerCurrentDrawAmps = steerMotor.getSupplyCurrent();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.targetSteerPositionRad = targetSteerPositionRad;

        inputs.steerAbsolutePosition =
                MathUtil.inputModulus(Units.degreesToRadians(steerEncoder.getAbsolutePosition()), 0, 2 * Math.PI);
    }

    @Override
    public void setTargetSteerPosition(double targetSteerPositionRad) {
        steerMotor.set(TalonFXControlMode.Position, targetSteerPositionRad / STEER_SENSOR_POSITION_COEFFICIENT);
        this.targetSteerPositionRad = targetSteerPositionRad;
    }

    @Override
    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
        driveMotor.set(
                TalonFXControlMode.Velocity, targetDriveVelocityMetersPerSec / DRIVE_SENSOR_VELOCITY_COEFFICIENT);
        this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
    }

    @Override
    public void resetToAbsoluteAngle() {
        steerMotor.setSelectedSensorPosition(
                Units.degreesToRadians(steerEncoder.getAbsolutePosition()) / STEER_SENSOR_POSITION_COEFFICIENT);
    }

    @Override
    public double getMaxVelocity() {
        return 12.0 / VELOCITY_COEFFICIENT;
    }
}
