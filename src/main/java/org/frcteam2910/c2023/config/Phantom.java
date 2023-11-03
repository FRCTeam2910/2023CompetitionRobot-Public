package org.frcteam2910.c2023.config;

import edu.wpi.first.math.controller.PIDController;

public class Phantom implements RobotConstants {
    // Ports and IDs
    private final int GYRO_ID = 1;

    private final int FRONT_LEFT_DRIVE_MOTOR_ID = 7;
    private final int SECONDARY_FRONT_LEFT_DRIVE_MOTOR_ID = 20;
    private final int FRONT_LEFT_STEER_MOTOR_ID = 8;
    private final int FRONT_LEFT_STEER_ENCODER_ID = 4;

    private final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    private final int SECONDARY_FRONT_RIGHT_DRIVE_MOTOR_ID = 18;
    private final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
    private final int FRONT_RIGHT_STEER_ENCODER_ID = 3;

    private final int BACK_LEFT_DRIVE_MOTOR_ID = 9;
    private final int SECONDARY_BACK_LEFT_DRIVE_MOTOR_ID = 19;
    private final int BACK_LEFT_STEER_MOTOR_ID = 6;
    private final int BACK_LEFT_STEER_ENCODER_ID = 2;

    private final int BACK_RIGHT_DRIVE_MOTOR_ID = 1;
    private final int SECONDARY_BACK_RIGHT_DRIVE_MOTOR_ID = 17;
    private final int BACK_RIGHT_STEER_MOTOR_ID = 2;
    private final int BACK_RIGHT_STEER_ENCODER_ID = 1;
    public static final int SHOULDER_MOTOR_ONE_ID = 5;
    public static final int SHOULDER_MOTOR_TWO_ID = 10;
    public static final int SHOULDER_MOTOR_THREE_ID = 11;
    public static final int SHOULDER_MOTOR_FOUR_ID = 12;

    public static final int ARM_MOTOR_ONE_ID = 13;
    public static final int ARM_MOTOR_TWO_ID = 14;

    private final int INTAKE_MOTOR_ID = 15;
    private final int BEAM_BREAK_ID = 0;
    private final int TIME_OF_FLIGHT_ID = 0;

    public static final int WRIST_MOTOR_ID = 16;

    public static final int HOME_INPUT_ID = 0;
    public static final int BRAKE_INPUT_ID = 1;

    public static final int CANDLE_ID = 0;

    private final int APRIL_TAG_CAMERA_PIPELINE = 0;
    private final int DRIVE_CAMERA_PIPELINE = 0;
    private final int GAME_PIECE_CAMERA_PIPELINE = 0;

    private final String CAN_BUS = "CANivore";
    private final String ARM_CAN_BUS = "";

    private final boolean USING_PHOENIX_PRO = true;
    private final boolean USING_SECONDARY_DRIVE_MOTOR = true;

    private PortConfiguration portConfiguration = new PortConfiguration();

    // Drivetrain Constants
    private final double FRONT_LEFT_STEER_OFFSET_RAD = -Math.toRadians(251.01);
    private final double FRONT_RIGHT_STEER_OFFSET_RAD = -Math.toRadians(54.755);
    private final double BACK_LEFT_STEER_OFFSET_RAD = -Math.toRadians(121.816);
    private final double BACK_RIGHT_STEER_OFFSET_RAD = -Math.toRadians(130.781);
    private static final int MOUNTING_ANGLE = 90;
    private static final double GYRO_ERROR = 2.08;
    private final double X_TRANSLATION_CONTROLLER_KP = 4;
    private final double X_TRANSLATION_CONTROLLER_KI = 0;
    private final double X_TRANSLATION_CONTROLLER_KD = 0;

    private final double Y_TRANSLATION_CONTROLLER_KP = 4;
    private final double Y_TRANSLATION_CONTROLLER_KI = 0;
    private final double Y_TRANSLATION_CONTROLLER_KD = 0;

    private final double ROTATION_CONTROLLER_KP = 4;
    private final double ROTATION_CONTROLLER_KI = 0;
    private final double ROTATION_CONTROLLER_KD = 0;

    private DrivetrainConfiguration drivetrainConfiguration = new DrivetrainConfiguration();

    public Phantom() {
        // Initialize PortConfiguration
        portConfiguration.gyroID = GYRO_ID;

        portConfiguration.frontLeftDriveMotorID = FRONT_LEFT_DRIVE_MOTOR_ID;
        portConfiguration.secondaryFrontLeftDriveMotorID = SECONDARY_FRONT_LEFT_DRIVE_MOTOR_ID;
        portConfiguration.frontLeftSteerMotorID = FRONT_LEFT_STEER_MOTOR_ID;
        portConfiguration.frontLeftSteerEncoderID = FRONT_LEFT_STEER_ENCODER_ID;

        portConfiguration.frontRightDriveMotorID = FRONT_RIGHT_DRIVE_MOTOR_ID;
        portConfiguration.secondaryFrontRightDriveMotorID = SECONDARY_FRONT_RIGHT_DRIVE_MOTOR_ID;
        portConfiguration.frontRightSteerMotorID = FRONT_RIGHT_STEER_MOTOR_ID;
        portConfiguration.frontRightSteerEncoderID = FRONT_RIGHT_STEER_ENCODER_ID;

        portConfiguration.backLeftDriveMotorID = BACK_LEFT_DRIVE_MOTOR_ID;
        portConfiguration.secondaryBackLeftDriveMotorID = SECONDARY_BACK_LEFT_DRIVE_MOTOR_ID;
        portConfiguration.backLeftSteerMotorID = BACK_LEFT_STEER_MOTOR_ID;
        portConfiguration.backLeftSteerEncoderID = BACK_LEFT_STEER_ENCODER_ID;

        portConfiguration.backRightDriveMotorID = BACK_RIGHT_DRIVE_MOTOR_ID;
        portConfiguration.secondaryBackRightDriveMotorID = SECONDARY_BACK_RIGHT_DRIVE_MOTOR_ID;
        portConfiguration.backRightSteerMotorID = BACK_RIGHT_STEER_MOTOR_ID;
        portConfiguration.backRightSteerEncoderID = BACK_RIGHT_STEER_ENCODER_ID;

        portConfiguration.intakeMotorID = INTAKE_MOTOR_ID;
        portConfiguration.beamBreakID = BEAM_BREAK_ID;
        portConfiguration.timeOfFlightID = TIME_OF_FLIGHT_ID;

        portConfiguration.shoulderMotorOneID = SHOULDER_MOTOR_ONE_ID;
        portConfiguration.shoulderMotorTwoID = SHOULDER_MOTOR_TWO_ID;
        portConfiguration.shoulderMotorThreeID = SHOULDER_MOTOR_THREE_ID;
        portConfiguration.shoulderMotorFourID = SHOULDER_MOTOR_FOUR_ID;

        portConfiguration.armMotorOneID = ARM_MOTOR_ONE_ID;
        portConfiguration.armMotorTwoID = ARM_MOTOR_TWO_ID;

        portConfiguration.wristMotorID = WRIST_MOTOR_ID;

        portConfiguration.homeInputID = HOME_INPUT_ID;
        portConfiguration.brakeInputID = BRAKE_INPUT_ID;

        portConfiguration.candleID = CANDLE_ID;

        portConfiguration.aprilTagCameraPipeline = APRIL_TAG_CAMERA_PIPELINE;
        portConfiguration.driveCameraPipeline = DRIVE_CAMERA_PIPELINE;
        portConfiguration.gamePieceCameraPipeline = GAME_PIECE_CAMERA_PIPELINE;

        portConfiguration.CANBus = CAN_BUS;
        portConfiguration.ArmCANBus = ARM_CAN_BUS;

        portConfiguration.usingPhoenixPro = USING_PHOENIX_PRO;
        portConfiguration.usingSecondaryDriveMotor = USING_SECONDARY_DRIVE_MOTOR;

        // Initialize DrivetrainConfiguration
        drivetrainConfiguration.swerveModuleOffsets = new double[] {
            FRONT_LEFT_STEER_OFFSET_RAD,
            FRONT_RIGHT_STEER_OFFSET_RAD,
            BACK_LEFT_STEER_OFFSET_RAD,
            BACK_RIGHT_STEER_OFFSET_RAD
        };
        drivetrainConfiguration.mountingAngle = MOUNTING_ANGLE;
        drivetrainConfiguration.error = GYRO_ERROR;
        drivetrainConfiguration.xController = new PIDController(
                X_TRANSLATION_CONTROLLER_KP, X_TRANSLATION_CONTROLLER_KI, X_TRANSLATION_CONTROLLER_KD);
        drivetrainConfiguration.yController = new PIDController(
                Y_TRANSLATION_CONTROLLER_KP, Y_TRANSLATION_CONTROLLER_KI, Y_TRANSLATION_CONTROLLER_KD);
        drivetrainConfiguration.rotationController =
                new PIDController(ROTATION_CONTROLLER_KP, ROTATION_CONTROLLER_KI, ROTATION_CONTROLLER_KD);
    }

    @Override
    public DrivetrainConfiguration getDrivetrainConfiguration() {
        return drivetrainConfiguration;
    }

    @Override
    public PortConfiguration getPortConfiguration() {
        return portConfiguration;
    }
}
