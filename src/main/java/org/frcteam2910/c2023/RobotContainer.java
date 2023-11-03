// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam2910.c2023;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.*;
import org.frcteam2910.c2023.commands.DefaultDriveCommand;
import org.frcteam2910.c2023.commands.*;
import org.frcteam2910.c2023.config.DrivetrainConfiguration;
import org.frcteam2910.c2023.config.PortConfiguration;
import org.frcteam2910.c2023.config.RobotConstants;
import org.frcteam2910.c2023.config.RobotIdentity;
import org.frcteam2910.c2023.subsystems.arm.ArmIOFalcon500;
import org.frcteam2910.c2023.subsystems.arm.ArmIOSim;
import org.frcteam2910.c2023.subsystems.arm.ArmSubsystem;
import org.frcteam2910.c2023.subsystems.drivetrain.*;
import org.frcteam2910.c2023.subsystems.intake.IntakeIO;
import org.frcteam2910.c2023.subsystems.intake.IntakeIOHardware;
import org.frcteam2910.c2023.subsystems.intake.IntakeSubsystem;
import org.frcteam2910.c2023.subsystems.led.LEDIOCANdle;
import org.frcteam2910.c2023.subsystems.led.LEDIOSim;
import org.frcteam2910.c2023.subsystems.led.LEDSubsystem;
import org.frcteam2910.c2023.subsystems.vision.VisionIO;
import org.frcteam2910.c2023.subsystems.vision.VisionSubsystem;
import org.frcteam2910.c2023.util.PathChooser;
import org.frcteam2910.c2023.util.PathTrajectories;
import org.frcteam2910.c2023.util.constants.ArmPoseConstants;
import org.frcteam2910.c2023.util.constants.Constants;
import org.frcteam2910.c2023.util.superstructure.Superstructure;
import org.frcteam2910.c2023.util.superstructure.SuperstructureIOHardware;
import org.frcteam2910.c2023.util.*;

public class RobotContainer {
    private final PathChooser pathChooser;

    private final DrivetrainSubsystem drivetrainSubsystem;

    private final LEDSubsystem ledSubsystem;

    private final ArmSubsystem armSubsystem;

    private final IntakeSubsystem intakeSubsystem;

    private final VisionSubsystem vision;

    private final XboxController primaryController = new XboxController(Constants.PRIMARY_XBOX_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(Constants.OPERATOR_XBOX_CONTROLLER_PORT);

    private final OperatorDashboard operatorDashboard;

    private final Superstructure superstructure;

    public RobotContainer() {
        RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
        DrivetrainConfiguration drivetrainConfiguration = robot.getDrivetrainConfiguration();
        PortConfiguration portConfiguration = robot.getPortConfiguration();
        vision = new VisionSubsystem(
                new VisionIO[] {
                    //            new VisionIOPhoton("front-left", Constants.FRONT_LEFT_PHOTON_POSE),
                    //            new VisionIOPhoton("back-left", Constants.BACK_LEFT_PHOTON_POSE),
                    //            new VisionIOPhoton("front-right", Constants.FRONT_RIGHT_PHOTON_POSE),
                    //            new VisionIOPhoton("back-right", Constants.BACK_RIGHT_PHOTON_POSE)
                });

        if (Robot.isReal()) {
            double[] swerveModuleOffsets = drivetrainConfiguration.swerveModuleOffsets;
            if (portConfiguration.usingPhoenixPro) {
                drivetrainSubsystem = new DrivetrainSubsystem(
                        new GyroIOPigeon2(
                                portConfiguration.gyroID,
                                drivetrainConfiguration.mountingAngle,
                                drivetrainConfiguration.error,
                                portConfiguration.CANBus),
                        new SwerveModuleIOFalcon500Pro(
                                portConfiguration.frontLeftDriveMotorID,
                                portConfiguration.frontLeftSteerMotorID,
                                portConfiguration.frontLeftSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[0],
                                portConfiguration.usingSecondaryDriveMotor,
                                portConfiguration.secondaryFrontLeftDriveMotorID),
                        new SwerveModuleIOFalcon500Pro(
                                portConfiguration.frontRightDriveMotorID,
                                portConfiguration.frontRightSteerMotorID,
                                portConfiguration.frontRightSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[1],
                                portConfiguration.usingSecondaryDriveMotor,
                                portConfiguration.secondaryFrontRightDriveMotorID),
                        new SwerveModuleIOFalcon500Pro(
                                portConfiguration.backLeftDriveMotorID,
                                portConfiguration.backLeftSteerMotorID,
                                portConfiguration.backLeftSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[2],
                                portConfiguration.usingSecondaryDriveMotor,
                                portConfiguration.secondaryBackLeftDriveMotorID),
                        new SwerveModuleIOFalcon500Pro(
                                portConfiguration.backRightDriveMotorID,
                                portConfiguration.backRightSteerMotorID,
                                portConfiguration.backRightSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[3],
                                portConfiguration.usingSecondaryDriveMotor,
                                portConfiguration.secondaryBackRightDriveMotorID),
                        vision,
                        drivetrainConfiguration);
            } else {
                drivetrainSubsystem = new DrivetrainSubsystem(
                        new GyroIOPigeon2(
                                portConfiguration.gyroID,
                                drivetrainConfiguration.mountingAngle,
                                drivetrainConfiguration.error,
                                portConfiguration.CANBus),
                        new SwerveModuleIOFalcon500(
                                portConfiguration.frontLeftDriveMotorID,
                                portConfiguration.frontLeftSteerMotorID,
                                portConfiguration.frontLeftSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[0]),
                        new SwerveModuleIOFalcon500(
                                portConfiguration.frontRightDriveMotorID,
                                portConfiguration.frontRightSteerMotorID,
                                portConfiguration.frontRightSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[1]),
                        new SwerveModuleIOFalcon500(
                                portConfiguration.backLeftDriveMotorID,
                                portConfiguration.backLeftSteerMotorID,
                                portConfiguration.backLeftSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[2]),
                        new SwerveModuleIOFalcon500(
                                portConfiguration.backRightDriveMotorID,
                                portConfiguration.backRightSteerMotorID,
                                portConfiguration.backRightSteerEncoderID,
                                portConfiguration.CANBus,
                                swerveModuleOffsets[3]),
                        vision,
                        drivetrainConfiguration);
            }
            ledSubsystem = new LEDSubsystem(new LEDIOCANdle(portConfiguration.candleID, portConfiguration.CANBus));

            armSubsystem = new ArmSubsystem(new ArmIOFalcon500(portConfiguration));

            operatorDashboard = new OperatorDashboard(this);

            intakeSubsystem = new IntakeSubsystem(new IntakeIOHardware(portConfiguration), operatorDashboard);
        } else {
            drivetrainSubsystem = new DrivetrainSubsystem(
                    null,
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim(),
                    vision,
                    drivetrainConfiguration);
            ledSubsystem = new LEDSubsystem(new LEDIOSim(9, 6));
            armSubsystem = new ArmSubsystem(new ArmIOSim());
            operatorDashboard = new OperatorDashboard(this);
            intakeSubsystem = new IntakeSubsystem(new IntakeIO() {}, operatorDashboard);
        }

        drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                drivetrainSubsystem,
                () -> -adjustJoystickValue(primaryController.getLeftY())
                        * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                () -> -adjustJoystickValue(primaryController.getLeftX())
                        * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                () -> -adjustJoystickValue(primaryController.getRightX())
                        * drivetrainSubsystem.getMaxAngularVelocityRadPerSec(),
                primaryController::getRightStickButton));

        superstructure = new Superstructure(new SuperstructureIOHardware(portConfiguration));

        pathChooser = new PathChooser(new PathTrajectories());

        configureBindings();
    }

    private static double adjustJoystickValue(double value) {
        value = MathUtil.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);
        return value;
    }

    private void configureBindings() {
        // Eject piece - X
        new Trigger(primaryController::getXButton)
                .whileTrue(new SetIntakeCommand(
                        intakeSubsystem,
                        primaryController,
                        () -> IntakeSubsystem.TargetIntakeStates.EJECT,
                        intakeSubsystem::getTargetPiece));

        // Arm up to free a stuck piece - B
        new Trigger(primaryController::getBButton)
                .whileTrue(new ArmToPoseCommand(armSubsystem, ArmPoseConstants.ARM_UP));

        // Home arm - Y
        new Trigger(primaryController::getYButton).onTrue(new SimultaneousHomeArmCommand(armSubsystem));

        // Rotate to set angle - A

        new Trigger(() -> primaryController.getAButton() && !primaryController.getRightBumper())
                .onTrue(new SnapToAngleCommand(
                        drivetrainSubsystem,
                        this::getDesiredRotation,
                        () -> -adjustJoystickValue(primaryController.getLeftY())
                                * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                        () -> -adjustJoystickValue(primaryController.getLeftX())
                                * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                        false,
                        true,
                        primaryController::getRightX));

        // Reset Pose - Back button
        new Trigger(primaryController::getBackButton).onTrue(new InstantCommand(drivetrainSubsystem::resetPose));

        // Score Piece - Right bumper and then right trigger to commit
        ArmToPoseCommand armToPoseCommand = new ArmToPoseCommand(
                armSubsystem,
                () -> getArmScoringPosition(false),
                true,
                primaryController::getAButton,
                operatorDashboard,
                intakeSubsystem);
        new Trigger(primaryController::getRightBumper)
                .whileTrue(new InstantCommand(
                                () -> intakeSubsystem.setTargetPiece(operatorDashboard.getSelectedGamePiece()))
                        .andThen(new SequentialCommandGroup(new WaitUntilCommand(() -> Math.abs(drivetrainSubsystem
                                                .getPose()
                                                .getRotation()
                                                .minus(getScoringRotation(true))
                                                .getRadians())
                                        < SnapToAngleCommand.ALLOWABLE_ANGLE_ERROR)
                                .andThen(new InstantCommand(() ->
                                        armToPoseCommand.setTargetPoseSupplier(() -> getArmScoringPosition(false))))
                                .andThen(armToPoseCommand.alongWith(new WaitUntilCommand(
                                                () -> (primaryController.getRightTriggerAxis() > 0.5
                                                        || ((operatorDashboard.getShouldUseAutoScore()
                                                                        && drivetrainSubsystem.isGoodToEject())
                                                                && armSubsystem.atTarget())))
                                        .andThen(new SetIntakeCommand(
                                                        intakeSubsystem,
                                                        primaryController,
                                                        () -> IntakeSubsystem.TargetIntakeStates.EJECT,
                                                        intakeSubsystem::getTargetPiece)
                                                .alongWith(new WaitCommand(0.0)
                                                        .andThen(new InstantCommand(
                                                                () -> armToPoseCommand.setTargetPoseSupplier(
                                                                        () -> getArmScoringPosition(true))))))))))
                        .alongWith(new DriveToScoringLocationCommand(
                                drivetrainSubsystem,
                                operatorDashboard,
                                () -> getScoringRotation(true),
                                () -> -adjustJoystickValue(primaryController.getLeftY())
                                        * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                                () -> -adjustJoystickValue(primaryController.getLeftX())
                                        * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                                primaryController,
                                primaryController::getAButton,
                                () -> primaryController.getRightTriggerAxis() > 0.5,
                                vision,
                                intakeSubsystem,
                                () -> ((operatorDashboard.getShouldUseAutoScore()
                                                && drivetrainSubsystem.isGoodToEject())
                                        && armSubsystem.atTarget()),
                                ledSubsystem)));

        // Return to Stow
        new Trigger(primaryController::getRightBumperReleased).onTrue(new StowArmCommand(armSubsystem));
        new Trigger(primaryController::getLeftBumperReleased).onTrue(new StowArmCommand(armSubsystem));
        new Trigger(primaryController::getBButtonReleased).onTrue(new StowArmCommand(armSubsystem));
        new Trigger(() -> primaryController.getLeftTriggerAxis() < 0.2).onTrue(new StowArmCommand(armSubsystem));

        // Ground Intake - Left trigger
        new Trigger(() -> primaryController.getLeftTriggerAxis() > 0.2)
                .onTrue(new ArmToPoseCommand(
                                armSubsystem,
                                () -> getArmIntakePosition(
                                        ArmSubsystem.ArmStates.GROUND_INTAKE, operatorDashboard::getSelectedGamePiece),
                                true)
                        .alongWith(new SetIntakeCommand(
                                intakeSubsystem,
                                operatorDashboard,
                                primaryController,
                                IntakeSubsystem.TargetIntakeStates.COLLECT)));

        // Portal Intake - Left bumper
        new Trigger(primaryController::getLeftBumper)
                .whileTrue(new SequentialCommandGroup(new WaitUntilCommand(() -> Math.abs(drivetrainSubsystem
                                                .getPose()
                                                .getRotation()
                                                .minus(getPortalIntakeRotation())
                                                .getRadians())
                                        < SnapToAngleCommand.ALLOWABLE_ANGLE_ERROR)
                                .andThen(new ArmToPoseCommand(
                                                armSubsystem,
                                                () -> getArmIntakePosition(
                                                        ArmSubsystem.ArmStates.PORTAL_INTAKE,
                                                        operatorDashboard::getSelectedGamePiece),
                                                true)
                                        .alongWith(new SetIntakeCommand(
                                                intakeSubsystem,
                                                operatorDashboard,
                                                primaryController,
                                                IntakeSubsystem.TargetIntakeStates.COLLECT))))
                        .alongWith(new SnapToAngleCommand(
                                drivetrainSubsystem,
                                this::getPortalIntakeRotation,
                                () -> -adjustJoystickValue(primaryController.getLeftY())
                                        * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                                () -> -adjustJoystickValue(primaryController.getLeftX())
                                        * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                                false)));

        // POV auto-align changing
        new Trigger(() -> primaryController.getPOV() == 90)
                .onTrue(new InstantCommand(() -> operatorDashboard.changeScoringPositionOffset(1)));
        new Trigger(() -> primaryController.getPOV() == 270)
                .onTrue(new InstantCommand(() -> operatorDashboard.changeScoringPositionOffset(-1)));

        // Button bindings for the physical buttons on the robot
        new Trigger(superstructure::getHomeButtonPressed).onTrue(new DisabledInstantCommand(() -> {
            if (DriverStation.isDisabled()) {
                armSubsystem.setAllSensorPositionsFromMeasurement(ArmPoseConstants.MANUAL_HOME_POSITION);
                armSubsystem.setZeroed(true);
                armSubsystem.setTargetPose(null);
                ledSubsystem.setWantedAction(LEDSubsystem.WantedAction.DISPLAY_ROBOT_ARM_ZEROED);
                //                                drivetrainSubsystem.resetPose();
            }
        }));

        new Trigger(superstructure::getBrakeButtonPressed).onTrue(new DisabledInstantCommand(() -> {
            if (DriverStation.isDisabled()) {
                armSubsystem.toggleBrakeMode();
            }
        }));

        // Align robot for climb
        new Trigger(() -> primaryController.getPOV() == 0)
                .onTrue(new SnapToAngleCommand(
                        drivetrainSubsystem,
                        () -> Rotation2d.fromDegrees(180),
                        () -> -adjustJoystickValue(primaryController.getLeftY())
                                * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                        () -> -adjustJoystickValue(primaryController.getLeftX())
                                * drivetrainSubsystem.getMaxVelocityMetersPerSec(),
                        false,
                        true,
                        primaryController::getRightX));

        // Operator Bindings
        new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.2)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setSelectedGamePiece(GamePiece.CUBE)));
        new Trigger(() -> operatorController.getRightTriggerAxis() > 0.2)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setSelectedGamePiece(GamePiece.CONE)));

        new Trigger(operatorController::getYButton)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setSelectedScoringLevel(GridLevel.HIGH)));
        new Trigger(() -> operatorController.getXButton() || operatorController.getBButton())
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setSelectedScoringLevel(GridLevel.MIDDLE)));
        new Trigger(operatorController::getAButton)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setSelectedScoringLevel(GridLevel.LOW)));

        new Trigger(() -> operatorController.getRightY() > 0.75)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setGroundCone(false)));
        new Trigger(() -> operatorController.getRightY() < -0.75)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setGroundCone(true)));

        new Trigger(() -> operatorController.getLeftY() > 0.75)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setSingleSubCube(false)));
        new Trigger(() -> operatorController.getLeftY() < -0.75)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setSingleSubCube(true)));

        new Trigger(() -> operatorController.getPOV() == 0)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.changeRotationOffset(1.0)));
        new Trigger(() -> operatorController.getPOV() == 180)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.changeRotationOffset(-1.0)));

        new Trigger(() -> operatorController.getPOV() == 90)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.changeTranslationOffset(0.5)));
        new Trigger(() -> operatorController.getPOV() == 270)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.changeTranslationOffset(-0.5)));

        new Trigger(operatorController::getStartButton)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setShouldUseAutoScore(true)));
        new Trigger(operatorController::getBackButton)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setShouldUseAutoScore(false)));

        new Trigger(operatorController::getRightStickButton)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setShouldSupercharge(true)));
        new Trigger(operatorController::getLeftStickButton)
                .onTrue(new DisabledInstantCommand(() -> operatorDashboard.setShouldSupercharge(false)));
    }

    public Command getAutonomousCommand() {
        return pathChooser.getCommand(this);
    }

    public OperatorDashboard getOperatorDashboard() {
        return operatorDashboard;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public LEDSubsystem getLedSubsystem() {
        return ledSubsystem;
    }

    private ArmPositions getArmIntakePosition(ArmSubsystem.ArmStates states, Supplier<GamePiece> gamePieceSupplier) {
        switch (states) {
            case GROUND_INTAKE:
                switch (gamePieceSupplier.get()) {
                    case CONE:
                        if (operatorDashboard.getShouldUprightGroundIntake()) {
                            return ArmPoseConstants.GROUND_CONE_UPRIGHT;
                        }
                        return ArmPoseConstants.GROUND_CONE_FLAT;
                    case CUBE:
                        return ArmPoseConstants.GROUND_CUBE;
                }
            case PORTAL_INTAKE:
                switch (gamePieceSupplier.get()) {
                    case CONE:
                        return ArmPoseConstants.PORTAL_CONE;
                    case CUBE:
                        if (operatorDashboard.getSingleSubCube()) {
                            return ArmPoseConstants.SINGLE_SUBSTATION_CUBE;
                        }
                        return ArmPoseConstants.PORTAL_CUBE;
                }
            default:
                return ArmPoseConstants.STOW;
        }
    }

    private ArmPositions getArmScoringPosition(boolean secondary) {
        switch (operatorDashboard.getSelectedGridLevel()) {
            case LOW:
                switch (intakeSubsystem.getTargetPiece()) {
                    case CONE:
                        double degrees =
                                drivetrainSubsystem.getPose().getRotation().getDegrees();
                        if (Math.abs(degrees) < 90) {
                            return ArmPoseConstants.L1_CONE_SHOOT;
                        } else {
                            return ArmPoseConstants.L1_CONE;
                        }
                    case CUBE:
                        return ArmPoseConstants.L1_CUBE;
                }
            case MIDDLE:
                switch (intakeSubsystem.getTargetPiece()) {
                    case CONE:
                        if (secondary) {
                            return ArmPoseConstants.SECONDARY_L2_CONE;
                        } else {
                            return operatorDashboard.getShouldSupercharge()
                                    ? ArmPoseConstants.L2_CONE_SUPERCHARGED
                                    : ArmPoseConstants.L2_CONE;
                        }
                    case CUBE:
                        double degrees =
                                drivetrainSubsystem.getPose().getRotation().getDegrees();
                        if (Math.abs(degrees) < 90) {
                            return ArmPoseConstants.L2_CUBE_BACK;
                        } else {
                            return ArmPoseConstants.L2_CUBE_FRONT;
                        }
                }
            case HIGH:
                switch (intakeSubsystem.getTargetPiece()) {
                    case CONE:
                        if (secondary) {
                            return ArmPoseConstants.SECONDARY_L3_CONE;
                        } else {
                            return operatorDashboard.getShouldSupercharge()
                                    ? ArmPoseConstants.L3_CONE_SUPERCHARGED
                                    : ArmPoseConstants.L3_CONE;
                        }
                    case CUBE:
                        return ArmPoseConstants.L3_CUBE;
                }
            default:
                return ArmPoseConstants.STOW;
        }
    }

    private Rotation2d getScoringRotation(boolean useIntakePiece) {
        switch (operatorDashboard.getSelectedGridLevel()) {
            case LOW:
                if ((useIntakePiece ? intakeSubsystem.getTargetPiece() : operatorDashboard.getSelectedGamePiece())
                        == GamePiece.CONE) {
                    Rotation2d frontDifference = drivetrainSubsystem.getPose().getRotation();
                    Rotation2d backDifference =
                            drivetrainSubsystem.getPose().getRotation().minus(Rotation2d.fromDegrees(180));
                    return (Math.abs(frontDifference.getRadians()) > Math.abs(backDifference.getRadians()))
                            ? Rotation2d.fromDegrees(180)
                            : Rotation2d.fromDegrees(0);
                } else {
                    return new Rotation2d(Math.toRadians(180));
                }
            case MIDDLE:
                if ((useIntakePiece ? intakeSubsystem.getTargetPiece() : operatorDashboard.getSelectedGamePiece())
                        == GamePiece.CUBE) {
                    Rotation2d frontDifference = drivetrainSubsystem.getPose().getRotation();
                    Rotation2d backDifference =
                            drivetrainSubsystem.getPose().getRotation().minus(Rotation2d.fromDegrees(180));
                    return (Math.abs(frontDifference.getRadians()) > Math.abs(backDifference.getRadians()))
                            ? Rotation2d.fromDegrees(180)
                            : Rotation2d.fromDegrees(0);
                } else {
                    return new Rotation2d(Math.toRadians(0));
                }
            case HIGH:
            default:
                return new Rotation2d(Math.toRadians(0.0));
        }
    }

    private Rotation2d getScoringRotation() {
        return getScoringRotation(false);
    }

    private Rotation2d getPortalIntakeRotation() {
        switch (operatorDashboard.getSelectedGamePiece()) {
            case CONE:
                return new Rotation2d(Math.toRadians(180.0));
            case CUBE:
            default:
                if (operatorDashboard.getSingleSubCube()) {
                    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                        return new Rotation2d(Math.toRadians(270.0));
                    } else {
                        return new Rotation2d(Math.toRadians(90.0));
                    }
                }
                return new Rotation2d(Math.toRadians(0.0));
        }
    }

    private Rotation2d getDesiredRotation() {
        if (intakeSubsystem.hasGamePiece()) {
            return getScoringRotation();
        } else {
            return getPortalIntakeRotation();
        }
    }
}
