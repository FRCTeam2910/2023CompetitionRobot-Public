package org.frcteam2910.c2023.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import org.frcteam2910.c2023.RobotContainer;
import org.frcteam2910.c2023.commands.*;
import org.frcteam2910.c2023.subsystems.drivetrain.DrivetrainSubsystem;
import org.frcteam2910.c2023.subsystems.intake.IntakeSubsystem;
import org.frcteam2910.c2023.util.constants.ArmPoseConstants;
import org.littletonrobotics.junction.Logger;

public class PathChooser {

    public static final double EJECTION_DISTANCE = 0.4;

    //  Enum for auto modes
    private enum AutonomousMode {
        DRIVE_STRAIGHT,
        BLUE_NO_BUMP_THREE_CUBE,
        RED_NO_BUMP_THREE_CUBE,
        NO_BUMP_TWO_CUBE_BALANCE,
        BLUE_NO_BUMP_TWO_AND_A_HALF_CUBE_BALANCE,
        RED_NO_BUMP_TWO_AND_A_HALF_CUBE_BALANCE,
        BLUE_LONG_INTAKE_NO_BUMP_THREE_CUBE_BALANCE,
        RED_LONG_INTAKE_NO_BUMP_THREE_CUBE_BALANCE,
        BLUE_LONG_INTAKE_NO_BUMP_THREE_CUBE_SUBSTATION,
        RED_LONG_INTAKE_NO_BUMP_THREE_CUBE_SUBSTATION,
        RED_BUMP_THREE_CUBE,
        BLUE_BUMP_THREE_CUBE,
        RED_CHEZYBUMP_THREE_CUBE,
        BLUE_CHEZYBUMP_THREE_CUBE,
        RED_CHEZYBUMP_TWO_CUBE,
        BLUE_CHEZYBUMP_TWO_CUBE,
        BUMP_TWO_AND_A_HALF_CUBE,
        BLUE_BUMP_TWO_CUBE,
        RED_BUMP_TWO_CUBE,
        MIDDLE_BALANCE,
        MIDDLE_UP_CUBE_BALANCE,
        MIDDLE_DOWN_CUBE_BALANCE,
        NO_BUMP_EXIT_COMMUNITY,
        BUMP_EXIT_COMMUNITY,
    }

    // Trajectories object
    private final PathTrajectories trajectories;
    // Chooser for autonomous mode
    private static final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    // Add options to chooser
    public PathChooser(PathTrajectories trajectories) {
        this.trajectories = trajectories;
        // autonomousModeChooser.addOption("Drive Straight", AutonomousMode.DRIVE_STRAIGHT);
        autonomousModeChooser.setDefaultOption(
                "Blue Yoshi No Bump 3 Cube Balance", AutonomousMode.BLUE_LONG_INTAKE_NO_BUMP_THREE_CUBE_BALANCE);
        autonomousModeChooser.setDefaultOption(
                "Red Yoshi No Bump 3 Cube Balance", AutonomousMode.RED_LONG_INTAKE_NO_BUMP_THREE_CUBE_BALANCE);
        autonomousModeChooser.addOption(
                "Blue Yoshi No Bump 3 Cube Substation", AutonomousMode.BLUE_LONG_INTAKE_NO_BUMP_THREE_CUBE_SUBSTATION);
        autonomousModeChooser.addOption(
                "Red Yoshi No Bump 3 Cube Substation", AutonomousMode.RED_LONG_INTAKE_NO_BUMP_THREE_CUBE_SUBSTATION);
        autonomousModeChooser.addOption("Blue No Bump 3 Cube", AutonomousMode.BLUE_NO_BUMP_THREE_CUBE);
        autonomousModeChooser.addOption("Red No Bump 3 Cube", AutonomousMode.RED_NO_BUMP_THREE_CUBE);
        // autonomousModeChooser.addOption("No Bump 1 Cube Balance", AutonomousMode.NO_BUMP_ONE_CUBE_BALANCE);;
        autonomousModeChooser.addOption(
                "Blue No Bump 2.5 Cube Balance", AutonomousMode.BLUE_NO_BUMP_TWO_AND_A_HALF_CUBE_BALANCE);
        autonomousModeChooser.addOption(
                "Red No Bump 2.5 Cube Balance", AutonomousMode.RED_NO_BUMP_TWO_AND_A_HALF_CUBE_BALANCE);
        //        autonomousModeChooser.addOption("Bump 1.5 Cube", AutonomousMode.BUMP_ONE_AND_A_HALF_CUBE);
        //        autonomousModeChooser.addOption("Chezy Blue Bump 3 Cube", AutonomousMode.BLUE_CHEZYBUMP_THREE_CUBE);
        //        autonomousModeChooser.addOption("Chezy Red Bump 3 Cube", AutonomousMode.RED_CHEZYBUMP_THREE_CUBE);
        //        autonomousModeChooser.addOption("Chezy Blue Bump 2 Cube", AutonomousMode.BLUE_CHEZYBUMP_TWO_CUBE);
        //        autonomousModeChooser.addOption("Chezy Red Bump 2 Cube", AutonomousMode.RED_CHEZYBUMP_TWO_CUBE);
        autonomousModeChooser.addOption("Blue Bump 3 Cube", AutonomousMode.BLUE_BUMP_THREE_CUBE);
        autonomousModeChooser.addOption("Red Bump 3 Cube", AutonomousMode.RED_BUMP_THREE_CUBE);
        autonomousModeChooser.addOption("Red Bump 2 Cube", AutonomousMode.RED_BUMP_TWO_CUBE);
        autonomousModeChooser.addOption("Blue Bump 2 Cube", AutonomousMode.BLUE_BUMP_TWO_CUBE);
        //        autonomousModeChooser.addOption("Middle No Bump Cube Balance", AutonomousMode.MIDDLE_UP_CUBE_BALANCE);
        //        autonomousModeChooser.addOption("Middle Bump Cube Balance", AutonomousMode.MIDDLE_DOWN_CUBE_BALANCE);
        autonomousModeChooser.addOption("Middle Balance", AutonomousMode.MIDDLE_BALANCE);
        autonomousModeChooser.addOption("No Bump Exit Community", AutonomousMode.NO_BUMP_EXIT_COMMUNITY);
        autonomousModeChooser.addOption("Bump Exit Community", AutonomousMode.BUMP_EXIT_COMMUNITY);
    }

    public Command getDriveStraight(RobotContainer container) {
        return resetDrivetrainPose(container, trajectories.getDriveStraightPath())
                .andThen(follow(container, trajectories.getDriveStraightPath()));
    }

    // Method for the preload score which happens before every sequence
    public Command getResetPoseAndPreloadScore(RobotContainer container, PathPlannerTrajectory trajectory) {
        return resetDrivetrainPose(container, trajectory)
                .alongWith(score(container, ArmPoseConstants.SECONDARY_L3_CONE, GamePiece.CONE));
    }

    // Exit community from white starting position
    public Command getNoBumpExitCommunity(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getNoBumpExitCommunity()));
        command.addCommands(followAndHome(container, trajectories.getNoBumpExitCommunity()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        return follow(container, trajectories.getNoBumpExitCommunity());
    }

    public Command getBumpExitCommunity(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getBumpExitCommunity()));
        command.addCommands(followAndHome(container, trajectories.getBumpExitCommunity()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        return follow(container, trajectories.getBumpExitCommunity());
    }

    public Command getMiddleBalance(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(resetDrivetrainPose(container, trajectories.getMiddleBalance()));
        command.addCommands(score(container, ArmPoseConstants.L3_CONE, GamePiece.CONE));
        //        command.addCommands(follow(container, trajectories.getMiddleBalance()).alongWith(new
        // SimultaneousHomeArmCommand(container.getArmSubsystem())));
        //        command.addCommands(new AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));
        command.addCommands(followAndDoChargingStationAndHome(container, trajectories.getMiddleBalance()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        return follow(container, trajectories.getMiddleBalance());
    }

    public Command getUpCubeMiddleBalance(RobotContainer container) {
        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getFourBlueToPrePlaceB()));
        //        command.addCommands(followWhileIntaking(
        //                container, trajectories.getFourBlueToPrePlaceB(), GamePiece.CUBE, 1, ArmPoseConstants.STOW,
        // false));
        //        command.addCommands(followThenScore(
        //                container,
        //                trajectories.getPrePlaceBToFiveBlue(),
        //                GamePiece.CUBE,
        //                ArmPoseConstants.STOW,
        //                1,
        //                ArmPoseConstants.L3_CUBE));
        //        command.addCommands(followAndDoChargingStationAndHome(container,
        // trajectories.getFiveBlueToChargingStation()));
        //        //        command.addCommands(resetDrivetrainPose(container, trajectories.getFourBlueToPrePlaceB()));
        //        //        command.addCommands(follow(container, trajectories.getFourBlueToPrePlaceB()));
        //        //        command.addCommands(follow(container, trajectories.getPrePlaceBToFiveBlue()));
        //        //        command.addCommands(follow(container, trajectories.getFiveBlueToChargingStation()));
        //        return command.finallyDo(interrupted ->
        // container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getFourBlueToPrePlaceB()));
        command.addCommands(followWhileIntaking(
                container, trajectories.getFourBlueToPrePlaceB(), GamePiece.CUBE, 1, ArmPoseConstants.STOW, false));
        command.addCommands(followAndDoChargingStationAndHomeAndThenScore(
                container, trajectories.getMiddlePrePlaceBToChargingStation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getDownCubeMiddleBalance(RobotContainer container) {
        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getSixGreenToPrePlaceC()));
        //        command.addCommands(followWhileIntaking(
        //                container, trajectories.getSixGreenToPrePlaceC(), GamePiece.CUBE, 1, ArmPoseConstants.STOW,
        // false));
        //        command.addCommands(followThenScore(
        //                container,
        //                trajectories.getPrePlaceCToFiveBlue(),
        //                GamePiece.CUBE,
        //                ArmPoseConstants.STOW,
        //                1,
        //                ArmPoseConstants.L3_CUBE));
        //        command.addCommands(followAndDoChargingStationAndHome(container,
        // trajectories.getFiveBlueToChargingStation()));
        //        //        command.addCommands(resetDrivetrainPose(container, trajectories.getSixGreenToPrePlaceC()));
        //        //        command.addCommands(follow(container, trajectories.getSixGreenToPrePlaceC()));
        //        //        command.addCommands(follow(container, trajectories.getPrePlaceCToFiveBlue()));
        //        //        command.addCommands(follow(container, trajectories.getFiveBlueToChargingStation()));
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getSixGreenToPrePlaceC()));
        command.addCommands(followWhileIntaking(
                container, trajectories.getSixGreenToPrePlaceC(), GamePiece.CUBE, 1, ArmPoseConstants.STOW, false));
        command.addCommands(followAndDoChargingStationAndHomeAndThenScore(
                container, trajectories.getMiddlePrePlaceCToChargingStation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getNoBumpTwoAndAHalfCubeBalance(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getBlueOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(container, trajectories.getBlueOneWhiteToPrePlaceA(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getBluePrePlaceAToTwoWhite(), GamePiece.CUBE, ArmPoseConstants.L3_CUBE));
        command.addCommands(followWhileIntaking(container, trajectories.getBlueTwoWhiteToPrePlaceB(), GamePiece.CUBE));
        //        command.addCommands(follow(container, trajectories.getPrePlaceBToChargingStation()));
        //        command.addCommands(new AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));
        command.addCommands(
                followAndDoChargingStationAndHome(container, trajectories.getBluePrePlaceBToChargingStation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(follow(container, trajectories.getOneWhiteToPrePlaceA()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceAToTwoWhite()));
        //        command.addCommands(follow(container, trajectories.getTwoWhiteToPrePlaceB()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceBToChargingStation()));
        //        return command;

    }

    public Command getRedNoBumpTwoAndAHalfCubeBalance(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getRedOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(container, trajectories.getRedOneWhiteToPrePlaceA(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getRedPrePlaceAToTwoWhite(), GamePiece.CUBE, ArmPoseConstants.L3_CUBE));
        command.addCommands(followWhileIntaking(container, trajectories.getRedTwoWhiteToPrePlaceB(), GamePiece.CUBE));
        command.addCommands(
                followAndDoChargingStationAndHome(container, trajectories.getRedPrePlaceBToChargingStation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(follow(container, trajectories.getOneWhiteToPrePlaceA()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceAToTwoWhite()));
        //        command.addCommands(follow(container, trajectories.getTwoWhiteToPrePlaceB()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceBToChargingStation()));
        //        return command;

    }

    public Command getNoBumpThreeCube(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getBlueOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(container, trajectories.getBlueOneWhiteToPrePlaceA(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getBluePrePlaceAToTwoWhite(), GamePiece.CUBE, ArmPoseConstants.L3_CUBE));
        command.addCommands(followWhileIntaking(container, trajectories.getBlueTwoWhiteToPrePlaceB(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getBluePrePlaceBToTwoWhite(), GamePiece.CUBE, ArmPoseConstants.L2_CUBE_BACK));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));

        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(follow(container, trajectories.getOneWhiteToPrePlaceA()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceAToTwoWhite()));
        //        command.addCommands(follow(container, trajectories.getTwoWhiteToPrePlaceB()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceBToTwoWhite()));
        //        return command;
    }

    public Command getRedNoBumpThreeCube(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getRedOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getRedOneWhiteToPrePlaceA(),
                GamePiece.CUBE,
                3,
                ArmPoseConstants.ARM_UP,
                false));
        command.addCommands(followThenScore(
                container, trajectories.getRedPrePlaceAToTwoWhite(), GamePiece.CUBE, ArmPoseConstants.L3_CUBE));
        command.addCommands(followWhileIntaking(container, trajectories.getRedTwoWhiteToPrePlaceB(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getRedPrePlaceBToTwoWhite(), GamePiece.CUBE, ArmPoseConstants.L2_CUBE_BACK));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));

        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(follow(container, trajectories.getOneWhiteToPrePlaceA()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceAToTwoWhite()));
        //        command.addCommands(follow(container, trajectories.getTwoWhiteToPrePlaceB()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceBToTwoWhite()));
        //        return command;
    }

    public Command getBumpThreeCube(RobotContainer container) {
        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(resetDrivetrainPose(container, trajectories.getNineOrangeToBumpOut()));
        //        command.addCommands(follow(container, trajectories.getNineOrangeToBumpOut()));
        //        command.addCommands(follow(container, trajectories.getBumpOut()));
        //        command.addCommands(follow(container, trajectories.getBumpOutToPrePlaceD()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceDToBump()));
        //        command.addCommands(follow(container, trajectories.getBumpToPrePlaceC()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceCToBumpReturn()));
        //        command.addCommands(follow(container, trajectories.getBumpReturn()));
        //        command.addCommands(follow(container, trajectories.getBumpReturnToEightOrange()));
        //        return command.finallyDo(interrupted ->
        // container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                getResetPoseAndPreloadScore(container, trajectories.getBlueThreeBumpNineOrangeToPrePlaceD()));
        //        command.addCommands(new InstantCommand(() ->
        // container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW)));
        //        command.addCommands(follow(container, trajectories.getNineOrangeToBumpOut()));
        //        command.addCommands(follow(container, trajectories.getBumpOut()));
        //        command.addCommands(followWhileIntaking(
        //                container, trajectories.getBumpOutToPrePlaceD(), GamePiece.CUBE, 1, ArmPoseConstants.STOW,
        // false));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getBlueThreeBumpNineOrangeToPrePlaceD(),
                GamePiece.CUBE,
                2.5,
                ArmPoseConstants.STOW,
                false));
        //        command.addCommands(followThenScore(
        //                container,
        //                trajectories.getPrePlaceDToBump(),
        //                GamePiece.CUBE,
        //                ArmPoseConstants.LONG_L1_CUBE_BACK,
        //                1.5,
        //                ArmPoseConstants.STOW, true));
        command.addCommands(follow(container, trajectories.getBluePrePlaceDToBump())
                .alongWith(new WaitCommand(trajectories.getBluePrePlaceDToBump().getTotalTimeSeconds() - 1.5)
                        .andThen(new ArmToPoseCommand(
                                container.getArmSubsystem(), ArmPoseConstants.BUMP_LONG_L1_CUBE_BACK))));
        command.addCommands(score(container, GamePiece.CUBE, true));

        //        command.addCommands(followWhileIntaking(
        //                container,
        //                trajectories.getBumpToPrePlaceC(),
        //                GamePiece.CUBE,
        //                trajectories.getBumpToPrePlaceC().getTotalTimeSeconds(),
        //                ArmPoseConstants.GROUND_CUBE,
        //                false));
        command.addCommands(
                followWhileIntakingWithNoWait(container, trajectories.getBlueBumpToPrePlaceC(), GamePiece.CUBE));
        //        command.addCommands(follow(container, trajectories.getPrePlaceCToBumpReturn()));
        //        command.addCommands(follow(container, trajectories.getBumpReturn()));
        // command.addCommands(followThenScore(container, trajectories.getBumpReturnToEightOrange(), GamePiece.CUBE,
        // ArmPoseConstants.L3_CUBE, 0.5, ArmPoseConstants.ARM_UP));
        //        command.addCommands(follow(container, trajectories.getThreeBumpPrePlaceCToEightOrange())
        //                .alongWith(new WaitCommand(
        //                        trajectories.getThreeBumpPrePlaceCToEightOrange().getTotalTimeSeconds() - 0.5)
        //                        .andThen(new ArmToPoseCommand(container.getArmSubsystem(),
        // ArmPoseConstants.L3_CUBE))));
        //        command.addCommands(score(container, GamePiece.CUBE));
        command.addCommands(followThenScore(
                container,
                trajectories.getBluePrePlaceCToEightORange(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE,
                2,
                ArmPoseConstants.STOW));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getRedBumpThreeCube(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getRedNineOrangeToPrePlaceD()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getRedNineOrangeToPrePlaceD(),
                GamePiece.CUBE,
                2.5,
                ArmPoseConstants.STOW,
                false));
        command.addCommands(follow(container, trajectories.getRedPrePlaceDToBump())
                .alongWith(new WaitCommand(trajectories.getRedPrePlaceDToBump().getTotalTimeSeconds() - 1.5)
                        .andThen(new ArmToPoseCommand(
                                container.getArmSubsystem(), ArmPoseConstants.BUMP_LONG_L1_CUBE_BACK))));
        command.addCommands(score(container, GamePiece.CUBE, true));
        command.addCommands(
                followWhileIntakingWithNoWait(container, trajectories.getRedBumpToPrePlaceC(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container,
                trajectories.getRedPrePlaceCToEightOrange(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE,
                2,
                ArmPoseConstants.STOW));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getBumpTwoAndAHalfCubeThenBalance(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getBlueNineOrangeToPrePlaceD()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getBlueNineOrangeToPrePlaceD(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                false));
        command.addCommands(followThenScore(
                container,
                trajectories.getPrePlaceDToNineOrange(),
                GamePiece.CUBE,
                ArmPoseConstants.L1_CUBE_BACK,
                1.25,
                ArmPoseConstants.ARM_UP));
        command.addCommands(followWhileIntaking(
                container, trajectories.getNineOrangeToPrePlaceC(), GamePiece.CUBE, 1.5, ArmPoseConstants.STOW, false));
        command.addCommands(followAndDoChargingStationAndHome(container, trajectories.getPrePlaceCToChargingStation()));
        //        command.addCommands(resetDrivetrainPose(container, trajectories.getNineOrangeToPrePlaceD()));
        //        command.addCommands(follow(container, trajectories.getNineOrangeToPrePlaceD()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceDToEightOrange()));
        //        command.addCommands(follow(container, trajectories.getEightOrangeToPrePlaceC()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceCToEightOrange()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getBlueBumpTwoCube(RobotContainer container) {
        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(resetDrivetrainPose(container, trajectories.getNineOrangeToPrePlaceD()));
        //        command.addCommands(follow(container, trajectories.getNineOrangeToPrePlaceD()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceDToEightOrange()));
        //        return command;
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getBlueNineOrangeToPrePlaceD()));
        //        command.addCommands(followThenScore(
        //                container,
        //                trajectories.getPrePlaceDToEightOrange(),
        //                GamePiece.CUBE,
        //                ArmPoseConstants.L3_CUBE,
        //                1,
        //                ArmPoseConstants.ARM_UP));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getBlueNineOrangeToPrePlaceD(),
                GamePiece.CUBE,
                2.5,
                ArmPoseConstants.STOW,
                false));
        command.addCommands(follow(container, trajectories.getPrePlaceDToEightOrange()));
        command.addCommands(new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.L3_CUBE));
        command.addCommands(score(container, GamePiece.CUBE));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        command.addCommands(resetDrivetrainPose(container, trajectories.getNineOrangeToPrePlaceD()));
        //        command.addCommands(follow(container, trajectories.getNineOrangeToPrePlaceD()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceDToEightOrange()));
        //        return command;
    }

    public Command getRedBumpTwoCube(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getRedNineOrangeToPrePlaceD()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getRedNineOrangeToPrePlaceD(),
                GamePiece.CUBE,
                2.5,
                ArmPoseConstants.STOW,
                false));
        command.addCommands(follow(container, trajectories.getRedPrePlaceDTo8Orange()));
        command.addCommands(new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.L3_CUBE));
        command.addCommands(score(container, GamePiece.CUBE));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getChezyBlue3Bump(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getChezyBlueNineOrangeToPrePlaceD()));
        command.addCommands(
                followWhileIntaking(container, trajectories.getChezyBlueNineOrangeToPrePlaceD(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container,
                trajectories.getChezyBluePrePlaceDToEightOrange(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE));
        command.addCommands(
                followWhileIntaking(container, trajectories.getChezyBlueEightOrangeToPrePlaceC(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container,
                trajectories.getChezyBluePrePlaceCToEightOrange(),
                GamePiece.CUBE,
                ArmPoseConstants.L2_CUBE_BACK));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getChezyRed3Bump(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getChezyRedNineOrangeToPrePlaceD()));
        command.addCommands(
                followWhileIntaking(container, trajectories.getChezyRedNineOrangeToPrePlaceD(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getChezyRedPrePlaceDToEightOrange(), GamePiece.CUBE, ArmPoseConstants.L3_CUBE));
        command.addCommands(
                followWhileIntaking(container, trajectories.getChezyRedEightOrangeToPrePlaceC(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container,
                trajectories.getChezyRedPrePlaceCToEightOrange(),
                GamePiece.CUBE,
                ArmPoseConstants.L2_CUBE_BACK));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getChezyBlue2Bump(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getChezyBlueNineOrangeToPrePlaceD()));
        command.addCommands(
                followWhileIntaking(container, trajectories.getChezyBlueNineOrangeToPrePlaceD(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container,
                trajectories.getChezyBluePrePlaceDToEightOrange(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE));
        command.addCommands(new InstantCommand(() -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW)));
        command.addCommands(followAndHome(container, trajectories.getChezyBlueEightOrangeToExitCommunity()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getChezyRed2Bump(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getChezyRedNineOrangeToPrePlaceD()));
        command.addCommands(
                followWhileIntaking(container, trajectories.getChezyRedNineOrangeToPrePlaceD(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getChezyRedPrePlaceDToEightOrange(), GamePiece.CUBE, ArmPoseConstants.L3_CUBE));
        command.addCommands(new InstantCommand(() -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW)));
        command.addCommands(followAndHome(container, trajectories.getChezyRedEightOrangeToExitCommunity()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getRedLongIntakeNoBumpThreeCubeBalance(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getRedLongIntakeOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getRedLongIntakeOneWhiteToPrePlaceA(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getRedLongIntakePrePlaceAToOneWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.LONG_L1_CUBE_BACK,
                2.5,
                ArmPoseConstants.ARM_UP));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getRedLongIntakeOneWhiteToPrePlaceB(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getRedLongIntakePrePlaceBToTwoWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE));
        command.addCommands(
                followAndDoChargingStationAndHome(container, trajectories.getRedTwoWhiteToChargingStation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        //        commandGroup.addCommands(resetDrivetrainPose(container,
        // trajectories.getRedLongIntakeOneWhiteToPrePlaceA()));
        //        commandGroup.addCommands(follow(container, trajectories.getRedLongIntakeOneWhiteToPrePlaceA()));
        //        commandGroup.addCommands(follow(container, trajectories.getRedLongIntakePrePlaceAToTwoWhite()));
        //        commandGroup.addCommands(follow(container, trajectories.getRedLongIntakeTwoWhiteToPrePlaceB()));
        //        commandGroup.addCommands(follow(container, trajectories.getRedLongIntakePrePlaceBToTwoWhite()));
        //        return commandGroup;
    }

    public Command getBlueLongIntakeNoBumpThreeCubeBalance(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(
                getResetPoseAndPreloadScore(container, trajectories.getBlueLongIntakeOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getBlueLongIntakeOneWhiteToPrePlaceA(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getBlueLongIntakePrePlaceAToOneWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.LONG_L1_CUBE_BACK,
                2.5,
                ArmPoseConstants.ARM_UP));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getBlueLongIntakeOneWhiteToPrePlaceB(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getBlueLongIntakePrePlaceBToTwoWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE));
        command.addCommands(
                followAndDoChargingStationAndHome(container, trajectories.getBlueTwoWhiteToChargingStation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        //        commandGroup.addCommands(resetDrivetrainPose(container,
        // trajectories.getBlueLongIntakeOneWhiteToPrePlaceA()));
        //        commandGroup.addCommands(follow(container, trajectories.getBlueLongIntakeOneWhiteToPrePlaceA()));
        //        commandGroup.addCommands(follow(container, trajectories.getBlueLongIntakePrePlaceAToTwoWhite()));
        //        commandGroup.addCommands(follow(container, trajectories.getBlueLongIntakeTwoWhiteToPrePlaceB()));
        //        commandGroup.addCommands(follow(container, trajectories.getBlueLongIntakePrePlaceBToTwoWhite()));
        //        return commandGroup;
    }

    public Command getRedLongIntakeNoBumpThreeCubeSubstation(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getRedLongIntakeOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getRedLongIntakeOneWhiteToPrePlaceA(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getRedLongIntakePrePlaceAToOneWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.LONG_L1_CUBE_BACK,
                2.5,
                ArmPoseConstants.ARM_UP));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getRedLongIntakeOneWhiteToPrePlaceB(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getRedLongIntakePrePlaceBToTwoWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE));
        command.addCommands(followAndHome(container, trajectories.getRedTwoWhiteToSubstation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getBlueLongIntakeNoBumpThreeCubeSubstation(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(
                getResetPoseAndPreloadScore(container, trajectories.getBlueLongIntakeOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getBlueLongIntakeOneWhiteToPrePlaceA(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getBlueLongIntakePrePlaceAToOneWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.LONG_L1_CUBE_BACK,
                2.5,
                ArmPoseConstants.ARM_UP));
        command.addCommands(followWhileIntaking(
                container,
                trajectories.getBlueLongIntakeOneWhiteToPrePlaceB(),
                GamePiece.CUBE,
                1.5,
                ArmPoseConstants.STOW,
                true));
        command.addCommands(followThenScore(
                container,
                trajectories.getBlueLongIntakePrePlaceBToTwoWhite(),
                GamePiece.CUBE,
                ArmPoseConstants.L3_CUBE));
        command.addCommands(followAndHome(container, trajectories.getBlueTwoWhiteToSubstation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
    }

    public Command getNoBumpTwoCubeBalance(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new InstantCommand(() -> container.getIntakeSubsystem().setTargetPiece(GamePiece.CONE)));
        command.addCommands(getResetPoseAndPreloadScore(container, trajectories.getBlueOneWhiteToPrePlaceA()));
        command.addCommands(followWhileIntaking(container, trajectories.getBlueOneWhiteToPrePlaceA(), GamePiece.CUBE));
        command.addCommands(followThenScore(
                container, trajectories.getBluePrePlaceAToTwoWhite(), GamePiece.CUBE, ArmPoseConstants.L3_CUBE));
        //        command.addCommands(follow(container, trajectories.getTwoWhiteToChargingStation()));
        //        command.addCommands(new AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));
        command.addCommands(
                followAndDoChargingStationAndHome(container, trajectories.getRedTwoWhiteToChargingStation()));
        return command.finallyDo(interrupted -> container.getArmSubsystem().setTargetPose(ArmPoseConstants.STOW));
        //        SequentialCommandGroup command = new SequentialCommandGroup();
        //        command.addCommands(follow(container, trajectories.getOneWhiteToPrePlaceA()));
        //        command.addCommands(follow(container, trajectories.getPrePlaceAToTwoWhite()));
        //        command.addCommands(follow(container, trajectories.getTwoWhiteToChargingStation()));
        //        return command;
    }

    /**
     * @param container  RobotContainer to get subsystems
     * @param trajectory Trajectory to follow
     */
    private Command follow(RobotContainer container, PathPlannerTrajectory trajectory) {
        return new FollowPathCommand(container.getDrivetrainSubsystem(), trajectory);
    }

    private Command followWithTimeout(RobotContainer container, PathPlannerTrajectory trajectory, double timeout) {
        return new FollowPathCommand(container.getDrivetrainSubsystem(), trajectory).withTimeout(timeout);
    }

    /**
     * @param container   RobotContainer to get subsystems
     * @param trajectory  Trajectory to follow
     * @param targetPiece Piece to intake
     */
    private Command followWhileIntakingWithArmHoming(
            RobotContainer container, PathPlannerTrajectory trajectory, GamePiece targetPiece) {
        SequentialCommandGroup followWhileIntakingAndHomingCommand = new SequentialCommandGroup();
        Timer timer = new Timer();
        followWhileIntakingAndHomingCommand.addCommands(new InstantCommand((timer::restart)));
        followWhileIntakingAndHomingCommand.addCommands(
                new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.ARM_UP));
        followWhileIntakingAndHomingCommand.addCommands(
                new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.STOW));
        followWhileIntakingAndHomingCommand.addCommands(new SimultaneousHomeArmCommand(container.getArmSubsystem()));
        followWhileIntakingAndHomingCommand.addCommands(new InstantCommand(timer::stop));
        followWhileIntakingAndHomingCommand.addCommands(intake(container, targetPiece));
        return follow(container, trajectory)
                .raceWith(followWhileIntakingAndHomingCommand)
                .andThen(new StowArmCommand(container.getArmSubsystem()));
    }

    private Command followAndHome(RobotContainer container, PathPlannerTrajectory trajectory) {
        SequentialCommandGroup followAndHomeCommand = new SequentialCommandGroup();
        // followAndHomeCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.ARM_UP));
        followAndHomeCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.STOW));
        followAndHomeCommand.addCommands(new SimultaneousHomeArmCommand(container.getArmSubsystem()));
        return follow(container, trajectory).alongWith(followAndHomeCommand);
    }

    private Command followAndDoChargingStationAndHome(RobotContainer container, PathPlannerTrajectory trajectory) {
        SequentialCommandGroup homeArmCommand = new SequentialCommandGroup();
        homeArmCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.STOW));
        homeArmCommand.addCommands(new SimultaneousHomeArmCommand(container.getArmSubsystem()));

        SequentialCommandGroup chargingStationCommand = new SequentialCommandGroup();
        chargingStationCommand.addCommands(follow(container, trajectory));
        chargingStationCommand.addCommands(new AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));

        return chargingStationCommand.alongWith(homeArmCommand);
    }

    private Command followAndDoChargingStationAndHomeAndThenScore(
            RobotContainer container, PathPlannerTrajectory trajectory) {
        SequentialCommandGroup homeArmCommand = new SequentialCommandGroup();
        homeArmCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.STOW));
        homeArmCommand.addCommands(new SimultaneousHomeArmCommand(container.getArmSubsystem()));

        SequentialCommandGroup chargingStationCommand = new SequentialCommandGroup();
        chargingStationCommand.addCommands(follow(container, trajectory));
        chargingStationCommand.addCommands(score(container, ArmPoseConstants.L1_CUBE_BACK, GamePiece.CUBE));
        chargingStationCommand.addCommands(new AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));

        return chargingStationCommand.alongWith(homeArmCommand);
    }

    /**
     * @param container   RobotContainer to get subsystems
     * @param trajectory  Trajectory to follow
     * @param targetPiece Piece to intake
     */
    private Command followWhileIntaking(
            RobotContainer container,
            PathPlannerTrajectory trajectory,
            GamePiece targetPiece,
            double waitTime,
            ArmPositions travelState,
            boolean longIntake) {
        ArmPositions position = ArmPoseConstants.GROUND_CONE_FLAT;
        if (targetPiece == GamePiece.CUBE) {
            position = longIntake ? ArmPoseConstants.LONG_GROUND_CUBE : ArmPoseConstants.GROUND_CUBE;
        }

        return follow(container, trajectory)
                .alongWith(new InstantCommand(() -> container.getArmSubsystem().setTargetPose(travelState)))
                .alongWith(new WaitCommand(trajectory.getTotalTimeSeconds() - waitTime)
                        .andThen(new ArmToPoseCommand(container.getArmSubsystem(), position)))
                .raceWith(new SetIntakeCommand(
                        container.getIntakeSubsystem(),
                        () -> IntakeSubsystem.TargetIntakeStates.COLLECT,
                        () -> targetPiece))
                .andThen(new StowArmCommand(container.getArmSubsystem()));
    }

    private Command followWhileIntakingWithNoWait(
            RobotContainer container, PathPlannerTrajectory trajectory, GamePiece targetPiece) {
        return follow(container, trajectory)
                .alongWith(new ArmToPoseCommand(container.getArmSubsystem(), ArmPoseConstants.GROUND_CUBE))
                .raceWith(new SetIntakeCommand(
                        container.getIntakeSubsystem(),
                        () -> IntakeSubsystem.TargetIntakeStates.COLLECT,
                        () -> targetPiece));
    }

    private Command followWhileIntaking(
            RobotContainer container, PathPlannerTrajectory trajectory, GamePiece targetPiece) {
        return followWhileIntaking(container, trajectory, targetPiece, 2, ArmPoseConstants.ARM_UP, false);
    }

    /**
     * @param container    RobotContainer to get subsystems
     * @param trajectory   Trajectory to follow
     * @param scoringPiece Piece to score
     */
    private Command followThenScore(
            RobotContainer container, PathPlannerTrajectory trajectory, GamePiece scoringPiece, ArmPositions position) {
        return followThenScore(container, trajectory, scoringPiece, position, 2, ArmPoseConstants.ARM_UP, false);
    }

    private Command followThenScore(
            RobotContainer container,
            PathPlannerTrajectory trajectory,
            GamePiece scoringPiece,
            ArmPositions position,
            double waitTime,
            ArmPositions travelState) {
        return followThenScore(container, trajectory, scoringPiece, position, waitTime, travelState, false);
    }

    private Command followThenScore(
            RobotContainer container,
            PathPlannerTrajectory trajectory,
            GamePiece scoringPiece,
            ArmPositions position,
            double waitTime,
            ArmPositions travelState,
            boolean fastEject) {
        SequentialCommandGroup armMotionCommand = new SequentialCommandGroup();
        Timer timer = new Timer();
        armMotionCommand.addCommands(new InstantCommand(timer::restart)); // Restart timer
        armMotionCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(), travelState)); // Put the arm up
        armMotionCommand.addCommands(new InstantCommand(
                timer::stop)); // Stop the timer such that it now stores the amount of time the arm command took
        armMotionCommand.addCommands(new ProxyCommand(() -> new WaitCommand(trajectory.getTotalTimeSeconds()
                - timer.get()
                - waitTime))); // Wait for the remaining time in the path minus a one second wait time
        armMotionCommand.addCommands(new ArmToPoseCommand(
                container.getArmSubsystem(),
                position)); // Move the arm to the scoring pose as we are one second out from hitting the drivetrain

        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(new WaitUntilCommand(() -> isAtScoringPose(container, trajectory)));
        intakeCommand.addCommands(score(container, scoringPiece, fastEject));

        return follow(container, trajectory).alongWith(armMotionCommand).alongWith(intakeCommand);
    }

    /**
     * @param container   RobotContainer to get subsystems
     * @param targetPiece Piece to intake
     */
    private Command intake(RobotContainer container, GamePiece targetPiece) {
        ArmPositions position = ArmPoseConstants.GROUND_CONE_FLAT;
        if (targetPiece == GamePiece.CUBE) {
            // position = ArmPoseConstants.GROUND_CUBE;
            position = ArmPoseConstants.LONG_GROUND_CUBE;
        }
        return new ArmToPoseCommand(container.getArmSubsystem(), position)
                .alongWith(new SetIntakeCommand(
                        container.getIntakeSubsystem(),
                        () -> IntakeSubsystem.TargetIntakeStates.COLLECT,
                        () -> targetPiece));
    }

    /**
     * @param container RobotContainer to get subsystems
     */
    private Command score(RobotContainer container, GamePiece piece) {
        return score(container, piece, false);
    }

    private Command score(RobotContainer container, GamePiece piece, boolean fastEject) {
        return new InstantCommand(() -> {
                    container.getIntakeSubsystem().setTargetPiece(piece);
                    container.getIntakeSubsystem().setTargetState(IntakeSubsystem.TargetIntakeStates.HOLDING);
                })
                // .andThen(new ArmToPoseCommand(container.getArmSubsystem(), () -> position))
                .andThen(new SetIntakeCommand(
                                container.getIntakeSubsystem(),
                                () -> fastEject
                                        ? IntakeSubsystem.TargetIntakeStates.FAST_EJECT
                                        : IntakeSubsystem.TargetIntakeStates.EJECT,
                                () -> piece)
                        .withTimeout(piece == GamePiece.CONE ? 0.4 : 0.2));
    }

    private Command score(RobotContainer container, ArmPositions position, GamePiece piece) {
        return new InstantCommand(() -> {
                    container.getIntakeSubsystem().setTargetPiece(piece);
                    container.getIntakeSubsystem().setTargetState(IntakeSubsystem.TargetIntakeStates.HOLDING);
                })
                .andThen(new ArmToPoseCommand(container.getArmSubsystem(), () -> position))
                .andThen(new SetIntakeCommand(
                                container.getIntakeSubsystem(),
                                () -> IntakeSubsystem.TargetIntakeStates.EJECT,
                                () -> piece)
                        .withTimeout(piece == GamePiece.CONE ? 0.4 : 0.15));
    }

    public boolean isAtScoringPose(RobotContainer container, PathPlannerTrajectory trajectory) {
        DrivetrainSubsystem drivetrainSubsystem = container.getDrivetrainSubsystem();
        Pose2d drivePose = drivetrainSubsystem.getPose();
        PathPlannerTrajectory.PathPlannerState state =
                PathPlannerTrajectory.transformStateForAlliance(trajectory.getEndState(), DriverStation.getAlliance());
        Pose2d desiredPose = state.poseMeters;
        double driveX = drivePose.getX();
        double driveY = drivePose.getY();

        double desiredX = desiredPose.getX();
        double desiredY = desiredPose.getY();

        double xError = Math.abs(desiredX - driveX);
        double yError = Math.abs(desiredY - driveY);

        Logger.getInstance().recordOutput("DesiredPose", desiredPose);
        Logger.getInstance().recordOutput("IsUnderError", xError < EJECTION_DISTANCE && yError < EJECTION_DISTANCE);
        return xError < EJECTION_DISTANCE && yError < EJECTION_DISTANCE;
    }

    public Command resetDrivetrainPose(RobotContainer container, PathPlannerTrajectory trajectory) {
        PathPlannerTrajectory.PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(
                trajectory.getInitialState(), DriverStation.getAlliance());
        return new InstantCommand(() -> resetPose(
                new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation), container));
    }

    public void resetPose(Pose2d pose, RobotContainer container) {
        container.getDrivetrainSubsystem().resetPose(pose);
        //        container
        //                .getDrivetrainSubsystem()
        //                .resetPose(new Pose2d(
        //                        pose.getX(),
        //                        pose.getY(),
        //                        container.getDrivetrainSubsystem().getPose().getRotation()));
    }

    public static SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case DRIVE_STRAIGHT:
                return getDriveStraight(container);
            case BLUE_NO_BUMP_THREE_CUBE:
                return getNoBumpThreeCube(container);
            case RED_NO_BUMP_THREE_CUBE:
                return getRedNoBumpThreeCube(container);
            case BLUE_NO_BUMP_TWO_AND_A_HALF_CUBE_BALANCE:
                return getNoBumpTwoAndAHalfCubeBalance(container);
            case RED_NO_BUMP_TWO_AND_A_HALF_CUBE_BALANCE:
                return getRedNoBumpTwoAndAHalfCubeBalance(container);
            case BLUE_LONG_INTAKE_NO_BUMP_THREE_CUBE_BALANCE:
                return getBlueLongIntakeNoBumpThreeCubeBalance(container);
            case RED_LONG_INTAKE_NO_BUMP_THREE_CUBE_BALANCE:
                return getRedLongIntakeNoBumpThreeCubeBalance(container);
            case BLUE_LONG_INTAKE_NO_BUMP_THREE_CUBE_SUBSTATION:
                return getBlueLongIntakeNoBumpThreeCubeSubstation(container);
            case RED_LONG_INTAKE_NO_BUMP_THREE_CUBE_SUBSTATION:
                return getRedLongIntakeNoBumpThreeCubeSubstation(container);
            case RED_BUMP_THREE_CUBE:
                return getRedBumpThreeCube(container);
            case BLUE_BUMP_THREE_CUBE:
                return getBumpThreeCube(container);
            case BUMP_TWO_AND_A_HALF_CUBE:
                return getBumpTwoAndAHalfCubeThenBalance(container);
            case BLUE_BUMP_TWO_CUBE:
                return getBlueBumpTwoCube(container);
            case RED_BUMP_TWO_CUBE:
                return getRedBumpTwoCube(container);
            case BLUE_CHEZYBUMP_THREE_CUBE:
                return getChezyBlue3Bump(container);
            case RED_CHEZYBUMP_THREE_CUBE:
                return getChezyRed3Bump(container);
            case BLUE_CHEZYBUMP_TWO_CUBE:
                return getChezyBlue2Bump(container);
            case RED_CHEZYBUMP_TWO_CUBE:
                return getChezyRed2Bump(container);
            case NO_BUMP_TWO_CUBE_BALANCE:
                return getNoBumpTwoCubeBalance(container);
            case NO_BUMP_EXIT_COMMUNITY:
                return getNoBumpExitCommunity(container);
            case MIDDLE_UP_CUBE_BALANCE:
                return getUpCubeMiddleBalance(container);
            case MIDDLE_DOWN_CUBE_BALANCE:
                return getDownCubeMiddleBalance(container);
            case MIDDLE_BALANCE:
                return getMiddleBalance(container);
            case BUMP_EXIT_COMMUNITY:
                return getBumpExitCommunity(container);
            default:
                return new InstantCommand();
        }
    }
}
