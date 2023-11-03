package org.frcteam2910.c2023.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;

import com.pathplanner.lib.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class PathTrajectories {
    //    private final PathConstraints constraints = new PathConstraints(Units.feetToMeters(19),
    // Units.feetToMeters(8));
    private final PathConstraints constraints = new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(6));

    public PathPlannerTrajectory getDriveStraightPath() {
        return driveStraightPath;
    }

    public PathPlannerTrajectory getRedOneWhiteToPrePlaceA() {
        return redOneWhiteToPrePlaceA;
    }

    public PathPlannerTrajectory getRedPrePlaceAToTwoWhite() {
        return redPrePlaceAToTwoWhite;
    }

    public PathPlannerTrajectory getRedTwoWhiteToPrePlaceB() {
        return redTwoWhiteToPrePlaceB;
    }

    public PathPlannerTrajectory getRedPrePlaceBToTwoWhite() {
        return redPrePlaceBToTwoWhite;
    }

    private final PathPlannerTrajectory redOneWhiteToPrePlaceA;
    private final PathPlannerTrajectory redPrePlaceAToTwoWhite;
    private final PathPlannerTrajectory redTwoWhiteToPrePlaceB;
    private final PathPlannerTrajectory redPrePlaceBToTwoWhite;

    public PathPlannerTrajectory getRedPrePlaceBToChargingStation() {
        return redPrePlaceBToChargingStation;
    }

    private final PathPlannerTrajectory redPrePlaceBToChargingStation;

    private final PathPlannerTrajectory driveStraightPath;

    // Special paths - exit community, charging station
    private final PathPlannerTrajectory bumpExitCommunity;
    private final PathPlannerTrajectory noBumpExitCommunity;
    private final PathPlannerTrajectory redTwoWhiteToChargingStation;
    private final PathPlannerTrajectory blueTwoWhiteToChargingStation;
    private final PathPlannerTrajectory twoWhiteToSubstation;

    public PathPlannerTrajectory getRedTwoWhiteToSubstation() {
        return redTwoWhiteToSubstation;
    }

    public PathPlannerTrajectory getBlueTwoWhiteToSubstation() {
        return blueTwoWhiteToSubstation;
    }

    private final PathPlannerTrajectory redTwoWhiteToSubstation;
    private final PathPlannerTrajectory blueTwoWhiteToSubstation;
    private final PathPlannerTrajectory bumpOut;
    private final PathPlannerTrajectory bumpReturn;
    private final PathPlannerTrajectory bumpOutToPrePlaceD;
    private final PathPlannerTrajectory bumpReturnToEightOrange;
    private final PathPlannerTrajectory fiveBlueToChargingStation;

    private final PathPlannerTrajectory prePlaceCToChargingStation;
    private final PathPlannerTrajectory bluePrePlaceBToChargingStation;

    private final PathPlannerTrajectory middlePrePlaceCToChargingStation;
    private final PathPlannerTrajectory middlePrePlaceBToChargingStation;
    private final PathPlannerTrajectory middleBalance;

    // Scoring poses, sorted from high to low
    private final PathPlannerTrajectory blueOneWhiteToPrePlaceA;
    private final PathPlannerTrajectory blueTwoWhiteToPrePlaceB;
    private final PathPlannerTrajectory fourBlueToPrePlaceB;
    private final PathPlannerTrajectory sixGreenToPrePlaceC;
    private final PathPlannerTrajectory blueNineOrangeToPrePlaceD;

    public PathPlannerTrajectory getRedPrePlaceDTo8Orange() {
        return redPrePlaceDTo8Orange;
    }

    private final PathPlannerTrajectory redPrePlaceDTo8Orange;

    public PathPlannerTrajectory getBlueThreeBumpNineOrangeToPrePlaceD() {
        return blueThreeBumpNineOrangeToPrePlaceD;
    }

    private final PathPlannerTrajectory blueThreeBumpNineOrangeToPrePlaceD;

    public PathPlannerTrajectory getBluePrePlaceCToEightORange() {
        return bluePrePlaceCToEightORange;
    }

    private final PathPlannerTrajectory bluePrePlaceCToEightORange;
    private final PathPlannerTrajectory eightOrangeToPrePlaceC;
    private final PathPlannerTrajectory nineOrangeToPrePlaceC;
    private final PathPlannerTrajectory nineOrangeToBumpOut;

    //  Intake poses, sorted from high to low
    private final PathPlannerTrajectory bluePrePlaceAToTwoWhite;
    private final PathPlannerTrajectory bluePrePlaceBToTwoWhite;
    private final PathPlannerTrajectory prePlaceBToFiveBlue;
    private final PathPlannerTrajectory prePlaceCToFiveBlue;
    private final PathPlannerTrajectory prePlaceCToEightOrange;
    private final PathPlannerTrajectory prePlaceCToBumpReturn;
    private final PathPlannerTrajectory bluePrePlaceDToEightOrange;
    private final PathPlannerTrajectory prePlaceDToNineOrange;

    private final PathPlannerTrajectory redLongIntakeOneWhiteToPrePlaceA;
    private final PathPlannerTrajectory redLongIntakeOneWhiteToPrePlaceB;
    private final PathPlannerTrajectory redLongIntakePrePlaceAToOneWhite;
    private final PathPlannerTrajectory redLongIntakePrePlaceBToTwoWhite;

    private final PathPlannerTrajectory blueLongIntakeOneWhiteToPrePlaceA;
    private final PathPlannerTrajectory blueLongIntakeOneWhiteToPrePlaceB;
    private final PathPlannerTrajectory blueLongIntakePrePlaceAToOneWhite;
    private final PathPlannerTrajectory blueLongIntakePrePlaceBToTwoWhite;

    public PathPlannerTrajectory getChezyBlueNineOrangeToPrePlaceD() {
        return chezyBlueNineOrangeToPrePlaceD;
    }

    public PathPlannerTrajectory getChezyRedNineOrangeToPrePlaceD() {
        return chezyRedNineOrangeToPrePlaceD;
    }

    public PathPlannerTrajectory getChezyBluePrePlaceDToEightOrange() {
        return chezyBluePrePlaceDToEightOrange;
    }

    public PathPlannerTrajectory getChezyRedPrePlaceDToEightOrange() {
        return chezyRedPrePlaceDToEightOrange;
    }

    public PathPlannerTrajectory getChezyBlueEightOrangeToPrePlaceC() {
        return chezyBlueEightOrangeToPrePlaceC;
    }

    public PathPlannerTrajectory getChezyRedEightOrangeToPrePlaceC() {
        return chezyRedEightOrangeToPrePlaceC;
    }

    public PathPlannerTrajectory getChezyBluePrePlaceCToEightOrange() {
        return chezyBluePrePlaceCToEightOrange;
    }

    public PathPlannerTrajectory getChezyRedPrePlaceCToEightOrange() {
        return chezyRedPrePlaceCToEightOrange;
    }

    private final PathPlannerTrajectory chezyBlueNineOrangeToPrePlaceD;
    private final PathPlannerTrajectory chezyRedNineOrangeToPrePlaceD;
    private final PathPlannerTrajectory chezyBluePrePlaceDToEightOrange;
    private final PathPlannerTrajectory chezyRedPrePlaceDToEightOrange;
    private final PathPlannerTrajectory chezyBlueEightOrangeToPrePlaceC;
    private final PathPlannerTrajectory chezyRedEightOrangeToPrePlaceC;
    private final PathPlannerTrajectory chezyBluePrePlaceCToEightOrange;
    private final PathPlannerTrajectory chezyRedPrePlaceCToEightOrange;

    public PathPlannerTrajectory getChezyRedEightOrangeToExitCommunity() {
        return chezyRedEightOrangeToExitCommunity;
    }

    private final PathPlannerTrajectory chezyRedEightOrangeToExitCommunity;

    public PathPlannerTrajectory getChezyBlueEightOrangeToExitCommunity() {
        return chezyBlueEightOrangeToExitCommunity;
    }

    private final PathPlannerTrajectory chezyBlueEightOrangeToExitCommunity;

    public PathPlannerTrajectory getRedNineOrangeToPrePlaceD() {
        return redNineOrangeToPrePlaceD;
    }

    public PathPlannerTrajectory getRedPrePlaceDToBump() {
        return redPrePlaceDToBump;
    }

    public PathPlannerTrajectory getRedBumpToPrePlaceC() {
        return redBumpToPrePlaceC;
    }

    public PathPlannerTrajectory getRedPrePlaceCToEightOrange() {
        return redPrePlaceCToEightOrange;
    }

    private final PathPlannerTrajectory redNineOrangeToPrePlaceD;
    private final PathPlannerTrajectory redPrePlaceDToBump;
    private final PathPlannerTrajectory redBumpToPrePlaceC;
    private final PathPlannerTrajectory redPrePlaceCToEightOrange;

    private final PathPlannerTrajectory blueBumpToPrePlaceC;
    private final PathPlannerTrajectory bluePrePlaceDToBump;

    public PathTrajectories() {
        redOneWhiteToPrePlaceA = PathPlanner.loadPath(
                "Red1WhiteToPrePlaceA", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));
        redPrePlaceAToTwoWhite = PathPlanner.loadPath("RedPrePlaceATo2White", constraints);
        redTwoWhiteToPrePlaceB = PathPlanner.loadPath("Red2WhiteToPrePlaceB", constraints);
        redPrePlaceBToTwoWhite = PathPlanner.loadPath("RedPrePlaceBTo2White", constraints);
        redPrePlaceBToChargingStation = PathPlanner.loadPath("RedPrePlaceBToChargingStation", constraints);

        redTwoWhiteToSubstation = PathPlanner.loadPath(
                "Red2WhiteToSubstation", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));
        blueTwoWhiteToSubstation = PathPlanner.loadPath(
                "Blue2WhiteToSubstation", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));

        redNineOrangeToPrePlaceD = PathPlanner.loadPath(
                "Red9OrangeToPrePlaceD", new PathConstraints(Units.feetToMeters(7), Units.feetToMeters(4)));
        redPrePlaceDToBump = PathPlanner.loadPath("RedPrePlaceDToBump", constraints);
        redBumpToPrePlaceC = PathPlanner.loadPath(
                "RedBumpToPrePlaceC", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(5)));
        redPrePlaceCToEightOrange = PathPlanner.loadPath(
                "RedPrePlaceCTo8Orange", new PathConstraints(Units.feetToMeters(7), Units.feetToMeters(7)));

        redPrePlaceDTo8Orange = PathPlanner.loadPath(
                "RedPrePlaceDTo8Orange", new PathConstraints(Units.feetToMeters(5), Units.feetToMeters(3)));

        // Special paths - exit community, charging station
        bumpExitCommunity = PathPlanner.loadPath("9OrangeToExitCommunity", constraints);
        noBumpExitCommunity = PathPlanner.loadPath("1WhiteToExitCommunity", constraints);
        redTwoWhiteToChargingStation = PathPlanner.loadPath("Red2WhiteToChargingStation", constraints);
        blueTwoWhiteToChargingStation = PathPlanner.loadPath("Blue2WhiteToChargingStation", constraints);
        fiveBlueToChargingStation = PathPlanner.loadPath("5BlueToChargingStation", constraints);
        twoWhiteToSubstation = PathPlanner.loadPath("2WhiteToSubstation", constraints);
        middleBalance = PathPlanner.loadPath("5BlueToChargingStation", constraints);
        middlePrePlaceBToChargingStation = PathPlanner.loadPath("MiddlePrePlaceBToChargingStation", constraints);
        middlePrePlaceCToChargingStation = PathPlanner.loadPath("MiddlePrePlaceCToChargingStation", constraints);
        bluePrePlaceBToChargingStation = PathPlanner.loadPath("BluePrePlaceBToChargingStation", constraints);
        prePlaceCToChargingStation = PathPlanner.loadPath("PrePlaceCToChargingStation", constraints);
        bumpReturn =
                PathPlanner.loadPath("BumpReturn", new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(3)));
        bumpOut = PathPlanner.loadPath("BumpOut", new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(3)));
        bumpOutToPrePlaceD = PathPlanner.loadPath("BumpOutToPrePlaceD", constraints);
        bumpReturnToEightOrange = PathPlanner.loadPath("BumpReturnTo8Orange", constraints);
        driveStraightPath = PathPlanner.loadPath(
                "DriveStraightPath", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(6)));

        blueOneWhiteToPrePlaceA = PathPlanner.loadPath(
                "Blue1WhiteToPrePlaceA", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));
        blueTwoWhiteToPrePlaceB = PathPlanner.loadPath("Blue2WhiteToPrePlaceB", constraints);
        fourBlueToPrePlaceB = PathPlanner.loadPath(
                "4BlueToPrePlaceB", new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(3)));
        sixGreenToPrePlaceC = PathPlanner.loadPath(
                "6GreenToPrePlaceC", new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(3)));
        blueNineOrangeToPrePlaceD = PathPlanner.loadPath(
                "Blue9OrangeToPrePlaceD", new PathConstraints(Units.feetToMeters(5), Units.feetToMeters(3)));
        blueThreeBumpNineOrangeToPrePlaceD = PathPlanner.loadPath(
                "BlueThreeBump9OrangeToPrePlaceD", new PathConstraints(Units.feetToMeters(7), Units.feetToMeters(4)));
        bluePrePlaceCToEightORange = PathPlanner.loadPath(
                "BluePrePlaceCTo8Orange", new PathConstraints(Units.feetToMeters(7), Units.feetToMeters(6)));
        eightOrangeToPrePlaceC = PathPlanner.loadPath("8OrangeToPrePlaceC", constraints);
        nineOrangeToPrePlaceC = PathPlanner.loadPath("9OrangeToPrePlaceC", constraints);
        nineOrangeToBumpOut = PathPlanner.loadPath("9OrangeToBumpOut", constraints);

        bluePrePlaceAToTwoWhite = PathPlanner.loadPath(
                "BluePrePlaceATo2White", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(6)));
        bluePrePlaceBToTwoWhite = PathPlanner.loadPath("BluePrePlaceBTo2White", constraints);
        prePlaceBToFiveBlue = PathPlanner.loadPath("PrePlaceBTo5Blue", constraints);
        prePlaceCToEightOrange = PathPlanner.loadPath(
                "PrePlaceCTo8Orange", new PathConstraints(Units.feetToMeters(5), Units.feetToMeters(3)));
        prePlaceCToFiveBlue = PathPlanner.loadPath("PrePlaceCTo5Blue", constraints);
        prePlaceCToBumpReturn = PathPlanner.loadPath("PrePlaceCToBumpReturn", constraints);
        bluePrePlaceDToEightOrange = PathPlanner.loadPath(
                "BluePrePlaceDTo8Orange", new PathConstraints(Units.feetToMeters(5), Units.feetToMeters(3)));
        prePlaceDToNineOrange = PathPlanner.loadPath("BluePrePlaceDTo9Orange", constraints);
        bluePrePlaceDToBump = PathPlanner.loadPath("BluePrePlaceDToBump", constraints);
        blueBumpToPrePlaceC = PathPlanner.loadPath(
                "BlueBumpToPrePlaceC", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(5)));

        redLongIntakeOneWhiteToPrePlaceA = PathPlanner.loadPath(
                "RedLongIntake1WhiteToPrePlaceA", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));
        redLongIntakeOneWhiteToPrePlaceB = PathPlanner.loadPath(
                "RedLongIntake1WhiteToPrePlaceB", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(7)));

        redLongIntakePrePlaceBToTwoWhite = PathPlanner.loadPath(
                "RedLongIntakePrePlaceBTo2White", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(7)));
        redLongIntakePrePlaceAToOneWhite = PathPlanner.loadPath(
                "RedLongIntakePrePlaceATo1White", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));

        blueLongIntakeOneWhiteToPrePlaceA = PathPlanner.loadPath(
                "BlueLongIntake1WhiteToPrePlaceA", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));
        blueLongIntakeOneWhiteToPrePlaceB = PathPlanner.loadPath(
                "BlueLongIntake1WhiteToPrePlaceB", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(7)));

        blueLongIntakePrePlaceBToTwoWhite = PathPlanner.loadPath(
                "BlueLongIntakePrePlaceBTo2White", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(7)));
        blueLongIntakePrePlaceAToOneWhite = PathPlanner.loadPath(
                "BlueLongIntakePrePlaceATo1White", new PathConstraints(Units.feetToMeters(19), Units.feetToMeters(9)));
        chezyBlueNineOrangeToPrePlaceD = PathPlanner.loadPath("ChezyBlue9OrangeToPrePlaceD", constraints);
        chezyRedNineOrangeToPrePlaceD = PathPlanner.loadPath("ChezyRed9OrangeToPrePlaceD", constraints);
        chezyBluePrePlaceDToEightOrange = PathPlanner.loadPath("ChezyBluePrePlaceDTo8Orange", constraints);
        chezyRedPrePlaceDToEightOrange = PathPlanner.loadPath("ChezyRedPrePlaceDTo8Orange", constraints);
        chezyBlueEightOrangeToPrePlaceC = PathPlanner.loadPath("ChezyBlue8OrangeToPrePlaceC", constraints);
        chezyRedEightOrangeToPrePlaceC = PathPlanner.loadPath("ChezyRed8OrangeToPrePlaceC", constraints);
        chezyBluePrePlaceCToEightOrange = PathPlanner.loadPath("ChezyBluePrePlaceCTo8Orange", constraints);
        chezyRedPrePlaceCToEightOrange = PathPlanner.loadPath("ChezyRedPrePlaceCTo8Orange", constraints);
        chezyRedEightOrangeToExitCommunity = PathPlanner.loadPath("ChezyRed8OrangeToExitCommunity", constraints);
        chezyBlueEightOrangeToExitCommunity = PathPlanner.loadPath("ChezyBlue8OrangeToExitCommunity", constraints);
    }

    //  Getters for robot path files
    public PathPlannerTrajectory getBumpExitCommunity() {
        return bumpExitCommunity;
    }

    public PathPlannerTrajectory getNoBumpExitCommunity() {
        return noBumpExitCommunity;
    }

    public PathPlannerTrajectory getRedTwoWhiteToChargingStation() {
        return redTwoWhiteToChargingStation;
    }

    public PathPlannerTrajectory getBlueTwoWhiteToChargingStation() {
        return blueTwoWhiteToChargingStation;
    }

    public PathPlannerTrajectory getFiveBlueToChargingStation() {
        return fiveBlueToChargingStation;
    }

    public PathPlannerTrajectory getTwoWhiteToSubstation() {
        return twoWhiteToSubstation;
    }

    public PathPlannerTrajectory getMiddleBalance() {
        return middleBalance;
    }

    public PathPlannerTrajectory getMiddlePrePlaceCToChargingStation() {
        return middlePrePlaceCToChargingStation;
    }

    public PathPlannerTrajectory getMiddlePrePlaceBToChargingStation() {
        return middlePrePlaceBToChargingStation;
    }

    public PathPlannerTrajectory getBluePrePlaceBToChargingStation() {
        return bluePrePlaceBToChargingStation;
    }

    public PathPlannerTrajectory getPrePlaceCToChargingStation() {
        return prePlaceCToChargingStation;
    }

    public PathPlannerTrajectory getBluePrePlaceDToBump() {
        return bluePrePlaceDToBump;
    }

    public PathPlannerTrajectory getBlueBumpToPrePlaceC() {
        return blueBumpToPrePlaceC;
    }

    public PathPlannerTrajectory getPrePlaceBToFiveBlue() {
        return prePlaceBToFiveBlue;
    }

    public PathPlannerTrajectory getBlueOneWhiteToPrePlaceA() {
        return blueOneWhiteToPrePlaceA;
    }

    public PathPlannerTrajectory getBlueTwoWhiteToPrePlaceB() {
        return blueTwoWhiteToPrePlaceB;
    }

    public PathPlannerTrajectory getFourBlueToPrePlaceB() {
        return fourBlueToPrePlaceB;
    }

    public PathPlannerTrajectory getSixGreenToPrePlaceC() {
        return sixGreenToPrePlaceC;
    }

    public PathPlannerTrajectory getEightOrangeToPrePlaceC() {
        return eightOrangeToPrePlaceC;
    }

    public PathPlannerTrajectory getBlueNineOrangeToPrePlaceD() {
        return blueNineOrangeToPrePlaceD;
    }

    public PathPlannerTrajectory getNineOrangeToPrePlaceC() {
        return nineOrangeToPrePlaceC;
    }

    public PathPlannerTrajectory getBluePrePlaceAToTwoWhite() {
        return bluePrePlaceAToTwoWhite;
    }

    public PathPlannerTrajectory getBluePrePlaceBToTwoWhite() {
        return bluePrePlaceBToTwoWhite;
    }

    public PathPlannerTrajectory getPrePlaceCToEightOrange() {
        return prePlaceCToEightOrange;
    }

    public PathPlannerTrajectory getPrePlaceCToFiveBlue() {
        return prePlaceCToFiveBlue;
    }

    public PathPlannerTrajectory getPrePlaceDToEightOrange() {
        return bluePrePlaceDToEightOrange;
    }

    public PathPlannerTrajectory getPrePlaceDToNineOrange() {
        return prePlaceDToNineOrange;
    }

    public PathPlannerTrajectory getBlueLongIntakeOneWhiteToPrePlaceA() {
        return blueLongIntakeOneWhiteToPrePlaceA;
    }

    public PathPlannerTrajectory getBlueLongIntakeOneWhiteToPrePlaceB() {
        return blueLongIntakeOneWhiteToPrePlaceB;
    }

    public PathPlannerTrajectory getBlueLongIntakePrePlaceAToOneWhite() {
        return blueLongIntakePrePlaceAToOneWhite;
    }

    public PathPlannerTrajectory getBlueLongIntakePrePlaceBToTwoWhite() {
        return blueLongIntakePrePlaceBToTwoWhite;
    }

    public PathPlannerTrajectory getRedLongIntakeOneWhiteToPrePlaceA() {
        return redLongIntakeOneWhiteToPrePlaceA;
    }

    public PathPlannerTrajectory getRedLongIntakeOneWhiteToPrePlaceB() {
        return redLongIntakeOneWhiteToPrePlaceB;
    }

    public PathPlannerTrajectory getRedLongIntakePrePlaceAToOneWhite() {
        return redLongIntakePrePlaceAToOneWhite;
    }

    public PathPlannerTrajectory getRedLongIntakePrePlaceBToTwoWhite() {
        return redLongIntakePrePlaceBToTwoWhite;
    }

    public PathPlannerTrajectory getNineOrangeToBumpOut() {
        return nineOrangeToBumpOut;
    }

    public PathPlannerTrajectory getBumpOut() {
        return bumpOut;
    }

    public PathPlannerTrajectory getBumpOutToPrePlaceD() {
        return bumpOutToPrePlaceD;
    }

    public PathPlannerTrajectory getBumpReturn() {
        return bumpReturn;
    }

    public PathPlannerTrajectory getPrePlaceCToBumpReturn() {
        return prePlaceCToBumpReturn;
    }

    public PathPlannerTrajectory getBumpReturnToEightOrange() {
        return bumpReturnToEightOrange;
    }

    private static PathPlannerTrajectory transformPath(
            String name, PathConstraints constraints, Translation2d translation) {
        List<PathPoint> points = trajectoryToPathPoints(name, false, -1);
        for (PathPoint point : points) {
            point.position.plus(translation);
        }
        return PathPlanner.generatePath(constraints, points);
    }

    private static PathPoint waypointToPathPoint(Waypoint waypoint) {
        PathPoint point =
                new PathPoint(waypoint.anchorPoint, new Rotation2d(), waypoint.holonomicRotation, waypoint.velOverride);
        if (waypoint.nextControl != null && waypoint.prevControl != null) {
            point = new PathPoint(
                    waypoint.anchorPoint,
                    (waypoint.nextControl.minus(waypoint.anchorPoint)).getAngle(),
                    waypoint.holonomicRotation,
                    waypoint.velOverride);
            point.withNextControlLength(
                    Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
            point.withPrevControlLength(
                    Math.abs((waypoint.prevControl.minus(waypoint.anchorPoint).getNorm())));
        } else if (waypoint.nextControl != null) {
            point = new PathPoint(
                    waypoint.anchorPoint,
                    (waypoint.nextControl.minus(waypoint.anchorPoint)).getAngle(),
                    waypoint.holonomicRotation,
                    waypoint.velOverride);
            point.withNextControlLength(
                    Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
            point.withPrevControlLength(
                    Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
        } else if (waypoint.prevControl != null) {
            point = new PathPoint(
                    waypoint.anchorPoint,
                    new Rotation2d((waypoint.prevControl.minus(waypoint.anchorPoint))
                                    .getAngle()
                                    .getRadians()
                            + Math.PI),
                    waypoint.holonomicRotation,
                    waypoint.velOverride);
            point.withPrevControlLength(
                    Math.abs((waypoint.prevControl.minus(waypoint.anchorPoint).getNorm())));
            point.withNextControlLength(
                    Math.abs((waypoint.prevControl.minus(waypoint.anchorPoint).getNorm())));
        }
        //        if (reverse) {
        //            if (waypoint.nextControl != null && waypoint.prevControl != null) {
        //                point.withNextControlLength(
        //                        Math.abs((waypoint.prevControl.minus(waypoint.anchorPoint).getNorm())));
        //                point.withPrevControlLength(
        //                        Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
        //            } else if (waypoint.nextControl != null) {
        //                point.withPrevControlLength(
        //                        Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
        //                point.withNextControlLength(
        //                        Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
        //            } else if (waypoint.prevControl != null) {
        //                point.withPrevControlLength(
        //                        Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
        //                point.withNextControlLength(
        //                        Math.abs((waypoint.nextControl.minus(waypoint.anchorPoint).getNorm())));
        //
        //            }
        //        }
        return point;
    }

    public static PathPlannerTrajectory combinePathFiles(List<String> names, PathConstraints constraints) {
        return combinePathFiles(names, constraints, false);
    }

    public static PathPlannerTrajectory combinePathFiles(
            List<String> names, PathConstraints constraints, boolean reversed) {
        return combinePathFiles(names, constraints, reversed, -1);
    }

    /**
     * Custom modifications of method from PathPlannerLib - initially called loadPath
     **/
    public static PathPlannerTrajectory combinePathFiles(
            List<String> names, PathConstraints constraints, boolean reversed, double endVelocity) {
        List<PathPoint> points = new ArrayList<>();
        boolean firstPath = true;

        for (String name : names) {
            try (BufferedReader br = new BufferedReader(
                    new FileReader(new File(Filesystem.getDeployDirectory(), "pathplanner/" + name + ".path")))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                List<Waypoint> waypoints = getWaypointsFromJson(json);
                // New code - convert Waypoints to PathPoints
                if (!firstPath) {
                    if (!waypoints.isEmpty()) {
                        waypoints.remove(0);
                    }
                }

                for (Waypoint waypoint : waypoints) {
                    if (reversed) {
                        Translation2d nextControl = waypoint.nextControl;
                        waypoint.nextControl = waypoint.prevControl;
                        waypoint.prevControl = nextControl;
                    }

                    points.add(waypointToPathPoint(waypoint));
                }
                firstPath = false;
                // List<PathPlannerTrajectory.EventMarker> markers = getMarkersFromJson(json);

            } catch (Exception e) {
                throw new RuntimeException("Failed to read path: " + name, e);
            }
        }

        if (reversed) {
            Collections.reverse(points);
        }

        points.set(
                points.size() - 1,
                PathPlannerWorkaround.setPathPointVelocity(points.get(points.size() - 1), endVelocity));

        // Return the new PathTrajectory
        return PathPlanner.generatePath(constraints, points);
    }

    public static List<PathPoint> trajectoryToPathPoints(String name, boolean reversed, double endVelocity) {
        List<PathPoint> points = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(
                new FileReader(new File(Filesystem.getDeployDirectory(), "pathplanner/" + name + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            List<Waypoint> waypoints = getWaypointsFromJson(json);
            // New code - convert Waypoints to PathPoints

            for (Waypoint waypoint : waypoints) {
                if (reversed) {
                    Translation2d nextControl = waypoint.nextControl;
                    waypoint.nextControl = waypoint.prevControl;
                    waypoint.prevControl = nextControl;
                }

                points.add(waypointToPathPoint(waypoint));
            }
            // List<PathPlannerTrajectory.EventMarker> markers = getMarkersFromJson(json);

        } catch (Exception e) {
            throw new RuntimeException("Failed to read path: " + name, e);
        }
        if (reversed) {
            Collections.reverse(points);
        }

        points.set(
                points.size() - 1,
                PathPlannerWorkaround.setPathPointVelocity(points.get(points.size() - 1), endVelocity));

        // Return the new PathTrajectory
        return points;
    }

    /**
     * Copied from PathPlannerLib as it had protected access
     **/
    protected static class Waypoint {
        protected Translation2d anchorPoint;
        protected Translation2d prevControl;
        protected Translation2d nextControl;
        protected double velOverride;
        protected Rotation2d holonomicRotation;
        protected boolean isReversal;
        protected boolean isStopPoint;
        protected PathPlannerTrajectory.StopEvent stopEvent;

        protected Waypoint(
                Translation2d anchorPoint,
                Translation2d prevControl,
                Translation2d nextControl,
                double velOverride,
                Rotation2d holonomicRotation,
                boolean isReversal,
                boolean isStopPoint,
                PathPlannerTrajectory.StopEvent stopEvent) {
            this.anchorPoint = anchorPoint;
            this.prevControl = prevControl;
            this.nextControl = nextControl;
            this.velOverride = velOverride;
            this.holonomicRotation = holonomicRotation;
            this.isReversal = isReversal;
            this.isStopPoint = isStopPoint;
            this.stopEvent = stopEvent;
        }
    }

    /**
     * Copied from PathPlannerLib as it had private access
     **/
    private static List<Waypoint> getWaypointsFromJson(JSONObject json) {
        JSONArray jsonWaypoints = (JSONArray) json.get("waypoints");

        List<Waypoint> waypoints = new ArrayList<>();

        for (Object waypoint : jsonWaypoints) {
            JSONObject jsonWaypoint = (JSONObject) waypoint;

            JSONObject jsonAnchor = (JSONObject) jsonWaypoint.get("anchorPoint");
            Translation2d anchorPoint = new Translation2d(
                    ((Number) jsonAnchor.get("x")).doubleValue(), ((Number) jsonAnchor.get("y")).doubleValue());

            JSONObject jsonPrevControl = (JSONObject) jsonWaypoint.get("prevControl");
            Translation2d prevControl = null;
            if (jsonPrevControl != null) {
                prevControl = new Translation2d(
                        ((Number) jsonPrevControl.get("x")).doubleValue(),
                        ((Number) jsonPrevControl.get("y")).doubleValue());
            }

            JSONObject jsonNextControl = (JSONObject) jsonWaypoint.get("nextControl");
            Translation2d nextControl = null;
            if (jsonNextControl != null) {
                nextControl = new Translation2d(
                        ((Number) jsonNextControl.get("x")).doubleValue(),
                        ((Number) jsonNextControl.get("y")).doubleValue());
            }

            Rotation2d holonomicAngle = null;
            if (jsonWaypoint.get("holonomicAngle") != null) {
                holonomicAngle = Rotation2d.fromDegrees(((Number) jsonWaypoint.get("holonomicAngle")).doubleValue());
            }
            boolean isReversal = (boolean) jsonWaypoint.get("isReversal");
            Object isStopPointObj = jsonWaypoint.get("isStopPoint");
            boolean isStopPoint = false;
            if (isStopPointObj != null) isStopPoint = (boolean) isStopPointObj;
            double velOverride = -1;
            if (jsonWaypoint.get("velOverride") != null) {
                velOverride = ((Number) jsonWaypoint.get("velOverride")).doubleValue();
            }

            PathPlannerTrajectory.StopEvent stopEvent = new PathPlannerTrajectory.StopEvent();
            if (jsonWaypoint.get("stopEvent") != null) {
                List<String> names = new ArrayList<>();
                PathPlannerTrajectory.StopEvent.ExecutionBehavior executionBehavior =
                        PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL;
                PathPlannerTrajectory.StopEvent.WaitBehavior waitBehavior =
                        PathPlannerTrajectory.StopEvent.WaitBehavior.NONE;
                double waitTime = 0;

                JSONObject stopEventJson = (JSONObject) jsonWaypoint.get("stopEvent");
                if (stopEventJson.get("names") != null) {
                    JSONArray namesArray = (JSONArray) stopEventJson.get("names");
                    for (Object name : namesArray) {
                        names.add(name.toString());
                    }
                }
                if (stopEventJson.get("executionBehavior") != null) {
                    PathPlannerTrajectory.StopEvent.ExecutionBehavior behavior =
                            PathPlannerTrajectory.StopEvent.ExecutionBehavior.fromValue(
                                    stopEventJson.get("executionBehavior").toString());

                    if (behavior != null) {
                        executionBehavior = behavior;
                    }
                }
                if (stopEventJson.get("waitBehavior") != null) {
                    PathPlannerTrajectory.StopEvent.WaitBehavior behavior =
                            PathPlannerTrajectory.StopEvent.WaitBehavior.fromValue(
                                    stopEventJson.get("waitBehavior").toString());

                    if (behavior != null) {
                        waitBehavior = behavior;
                    }
                }
                if (stopEventJson.get("waitTime") != null) {
                    waitTime = ((Number) stopEventJson.get("waitTime")).doubleValue();
                }

                stopEvent = new PathPlannerTrajectory.StopEvent(names, executionBehavior, waitBehavior, waitTime);
            }

            waypoints.add(new Waypoint(
                    anchorPoint,
                    prevControl,
                    nextControl,
                    velOverride,
                    holonomicAngle,
                    isReversal,
                    isStopPoint,
                    stopEvent));
        }

        return waypoints;
    }

    //    public class JesterPathPlannerTrajectory extends PathPlannerTrajectory{
    //        public JesterPathPlannerTrajectory(
    //                List<PathTrajectories.Waypoint> pathPoints,
    //                List<PathPlannerTrajectory.EventMarker> markers,
    //                PathConstraints constraints,
    //                boolean reversed,
    //                boolean fromGUI
    //        ) {
    //            super(pathPoints, markers, constraints, reversed, fromGUI));
    //        }
    //    }
    //    protected PathPlannerTrajectory(
    //            List<Waypoint> pathPoints,
    //            List<PathPlannerTrajectory.EventMarker> markers,
    //            PathConstraints constraints,
    //            boolean reversed,
    //            boolean fromGUI) {
    //        super(generatePath(pathPoints, constraints.maxVelocity, constraints.maxAcceleration, reversed));
    //
    //        this.markers = markers;
    //        this.calculateMarkerTimes(pathPoints);
    //        this.startStopEvent = pathPoints.get(0).stopEvent;
    //        this.endStopEvent = pathPoints.get(pathPoints.size() - 1).stopEvent;
    //        this.fromGUI = fromGUI;
    //    }
}
