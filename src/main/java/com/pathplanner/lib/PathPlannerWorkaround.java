package com.pathplanner.lib;

public class PathPlannerWorkaround {
    public static PathPoint setPathPointVelocity(PathPoint point, double velocity) {
        PathPoint newPoint = new PathPoint(point.position, point.heading, point.holonomicRotation, velocity);
        newPoint.withNextControlLength(point.nextControlLength);
        newPoint.withPrevControlLength(point.prevControlLength);
        return newPoint;
    }
}
