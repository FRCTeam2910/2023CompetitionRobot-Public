package org.frcteam2910.c2023.util;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.*;
import org.frcteam2910.c2023.RobotContainer;

public class OperatorDashboard {
    RobotContainer container;

    private final GenericEntry[] rowButtons = new GenericEntry[3];
    private static final boolean[] rowBooleansArray = new boolean[3];

    private final GenericEntry[] nodeButtons = new GenericEntry[2];
    private final boolean[] nodeTypeBooleanArray = new boolean[2];

    private final GenericEntry[] shouldUseVision = new GenericEntry[1];
    private final GenericEntry[] shouldUseAutoScore = new GenericEntry[1];
    private final GenericEntry[] uprightGroundIntakeButton = new GenericEntry[1];
    private final GenericEntry[] singleCubeIntakeButton = new GenericEntry[1];
    private final GenericEntry[] shouldSupercharge = new GenericEntry[1];

    private final SimpleWidget rotationOffsetEntry;
    private final SimpleWidget translationOffsetEntry;

    private int scoringPositionOffset = 0;
    private double rotationOffset = 0.0;
    private double yOffset = 0.0;
    private boolean offsetChangePositive = true;

    public OperatorDashboard(RobotContainer container) {
        this.container = container;

        ShuffleboardTab version1 = Shuffleboard.getTab("Version 1 Grid");

        rowButtons[0] = version1.add("High Node?", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(0, 0)
                .withSize(3, 2)
                .getEntry();
        rowButtons[1] = version1.add("Middle Node?", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(0, 2)
                .withSize(3, 2)
                .getEntry();
        rowButtons[2] = version1.add("Low Node?", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(0, 4)
                .withSize(3, 2)
                .getEntry();
        version1.addString("Level: ", () -> String.valueOf(getSelectedGridLevel()))
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 6)
                .withSize(3, 1);

        nodeButtons[0] = version1.add("▲ Cone", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(3, 0)
                .withSize(3, 2)
                .getEntry();
        nodeButtons[1] = version1.add("◼ Cube", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(3, 2)
                .withSize(3, 2)
                .getEntry();
        uprightGroundIntakeButton[0] = version1.add("Upright Ground Intake?", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(6, 0)
                .withSize(3, 2)
                .getEntry();
        singleCubeIntakeButton[0] = version1.add("Single Sub Cube Intake?", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(6, 2)
                .withSize(3, 2)
                .getEntry();
        version1.addString("Game Piece: ", this::getSelectedGamePieceString)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(3, 4)
                .withSize(3, 1);

        //        version1.add("-", new InstantCommand(() -> {
        //                    scoringPositionOffset = MathUtil.clamp(scoringPositionOffset - 1, -8, 8);
        //                    offsetChangePositive = false;
        //                }))
        //                .withPosition(6, 2)
        //                .withSize(2, 2);
        //        version1.add("+", new InstantCommand(() -> {
        //                    scoringPositionOffset = MathUtil.clamp(scoringPositionOffset + 1, -8, 8);
        //                    offsetChangePositive = true;
        //                }))
        //                .withPosition(8, 2)
        //                .withSize(2, 2);

        version1.addNumber("Offset", () -> scoringPositionOffset)
                .withPosition(6, 4)
                .withSize(3, 2);

        rotationOffsetEntry = version1.add("Auto Align Rotation Offset", 0.0)
                .withSize(5, 2)
                .withPosition(9, 2)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -10.0, "max", 10.0, "block increment", 1.0));

        translationOffsetEntry = version1.add("Auto Align Y Offset", 0.0)
                .withSize(5, 2)
                .withPosition(14, 2)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -5.0, "max", 5.0, "block increment", 0.5));

        //        version1.add("Zero Gyro", new InstantCommand(container.getDrivetrainSubsystem()::resetPose))
        //                .withPosition(18, 0)
        //                .withSize(3, 2);
        //        version1.add("Home Arm", new SimultaneousHomeArmCommand(container.getArmSubsystem()))
        //                .withPosition(18, 2)
        //                .withSize(3, 2);

        shouldUseVision[0] = version1.add("Use vision?", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(9, 4)
                .withSize(3, 2)
                .getEntry();

        shouldUseAutoScore[0] = version1.add("Use auto-eject?", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(12, 4)
                .withSize(3, 2)
                .getEntry();

        shouldSupercharge[0] = version1.add("Supercharge?", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(3, 5)
                .withSize(3, 2)
                .getEntry();
    }

    public void update() {
        rotationOffset = rotationOffsetEntry.getEntry().getDouble(rotationOffset);
        yOffset = translationOffsetEntry.getEntry().getDouble(yOffset);
        // update for row buttons
        for (int i = 0; i < 3; i++) {
            if (rowButtons[i].getBoolean(false) != rowBooleansArray[i] & rowButtons[i].getBoolean(false)) {
                rowButtons[0].setBoolean(false);
                rowBooleansArray[0] = false;
                rowButtons[1].setBoolean(false);
                rowBooleansArray[1] = false;
                rowButtons[2].setBoolean(false);
                rowBooleansArray[2] = false;
                rowButtons[i].setBoolean(true);
                rowBooleansArray[i] = true;
            } else if (rowButtons[i].getBoolean(false) != rowBooleansArray[i]) {
                rowBooleansArray[i] = false;
            }
        }

        // update for node buttons
        for (int i = 0; i < 2; i++) {
            if (nodeButtons[i].getBoolean(false) != nodeTypeBooleanArray[i] & nodeButtons[i].getBoolean(false)) {
                nodeButtons[0].setBoolean(false);
                nodeTypeBooleanArray[0] = false;
                nodeButtons[1].setBoolean(false);
                nodeTypeBooleanArray[1] = false;
                nodeButtons[i].setBoolean(true);
                nodeTypeBooleanArray[i] = true;
            } else if (nodeButtons[i].getBoolean(false) != nodeTypeBooleanArray[i]) {
                nodeTypeBooleanArray[i] = false;
            }
        }
        container.getDrivetrainSubsystem().setShouldUseVisionData(getShouldUseVision());
    }

    public void setSelectedGamePiece(GamePiece piece) {
        nodeButtons[piece == GamePiece.CONE ? 0 : 1].setBoolean(true);
    }

    public void setSelectedScoringLevel(GridLevel level) {
        switch (level) {
            case HIGH:
                rowButtons[0].setBoolean(true);
                break;
            case MIDDLE:
                rowButtons[1].setBoolean(true);
                break;
            case LOW:
                rowButtons[2].setBoolean(true);
        }
    }

    public GamePiece getSelectedGamePiece() {
        for (int i = 0; i < 2; i++) {
            if (nodeTypeBooleanArray[i]) {
                if (i == 0) {
                    return GamePiece.CONE;
                } else {
                    return GamePiece.CUBE;
                }
            }
        }
        return GamePiece.CONE;
    }

    public String getSelectedGamePieceString() {
        for (int i = 0; i < 2; i++) {
            if (nodeTypeBooleanArray[i]) {
                if (i == 0) {
                    String isUpright = getShouldUprightGroundIntake() ? "Upright" : "Flat";
                    return "Cone " + isUpright;
                } else {
                    String isSingleSub = getSingleSubCube() ? " Single Substation" : "";
                    return "Cube" + isSingleSub;
                }
            }
        }
        return "";
    }

    public GridLevel getSelectedGridLevel() {
        for (int i = 0; i < 3; i++) {
            if (OperatorDashboard.rowBooleansArray[i]) {
                if (i == 2) {
                    return GridLevel.LOW;
                } else if (i == 1) {
                    return GridLevel.MIDDLE;
                } else {
                    return GridLevel.HIGH;
                }
            }
        }
        return GridLevel.HIGH;
    }

    public void setScoringPositionOffset(int scoringPositionOffset) {
        this.scoringPositionOffset = scoringPositionOffset;
    }

    public int getScoringPositionOffset() {
        return scoringPositionOffset;
    }

    public double getRotationOffset() {
        return rotationOffset;
    }

    public double getTranslationOffset() {
        return yOffset;
    }

    public void changeRotationOffset(double offsetChange) {
        rotationOffset = MathUtil.clamp(rotationOffset + offsetChange, -10, 10);
        rotationOffsetEntry.getEntry().set(NetworkTableValue.makeDouble(rotationOffset));
    }

    public void changeTranslationOffset(double offsetChange) {
        yOffset = MathUtil.clamp(yOffset + offsetChange, -5.0, 5.0);
        translationOffsetEntry.getEntry().set(NetworkTableValue.makeDouble(yOffset));
    }

    public void changeScoringPositionOffset(int offsetChange) {
        scoringPositionOffset = MathUtil.clamp(scoringPositionOffset + offsetChange, -8, 8);
    }

    public boolean isOffsetChangePositive() {
        return offsetChangePositive;
    }

    public boolean getShouldUseVision() {
        return shouldUseVision[0].getBoolean(true);
    }

    public void setShouldUseAutoScore(boolean useAutoScore) {
        shouldUseAutoScore[0].setBoolean(useAutoScore);
    }

    public boolean getShouldUseAutoScore() {
        return shouldUseAutoScore[0].getBoolean(false);
    }

    public void setGroundCone(boolean groundCone) {
        uprightGroundIntakeButton[0].setBoolean(groundCone);
    }

    public boolean getShouldUprightGroundIntake() {
        return uprightGroundIntakeButton[0].getBoolean(false);
    }

    public void setSingleSubCube(boolean singleSubCube) {
        singleCubeIntakeButton[0].setBoolean(singleSubCube);
    }

    public boolean getSingleSubCube() {
        return singleCubeIntakeButton[0].getBoolean(false);
    }

    public void setShouldSupercharge(boolean supercharge) {
        shouldSupercharge[0].setBoolean(supercharge);
    }

    public boolean getShouldSupercharge() {
        return shouldSupercharge[0].getBoolean(false);
    }

    public void toggleGroundCone() {
        uprightGroundIntakeButton[0].setBoolean(!uprightGroundIntakeButton[0].getBoolean(false));
    }
}
