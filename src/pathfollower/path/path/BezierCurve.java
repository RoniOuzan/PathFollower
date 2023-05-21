package pathfollower.path.path;

import pathfollower.math.geometry.Pose2d;
import pathfollower.math.geometry.Rotation2d;
import pathfollower.math.geometry.Translation2d;
import pathfollower.path.Path;
import pathfollower.path.util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@SuppressWarnings(value = "unused")
public class BezierCurve extends Path {

    public BezierCurve(Constants constants, List<Translation2d> waypoints) {
        super(constants, 0.0001,
                waypoints.parallelStream().map(t -> new Waypoint(t, 0, 0)).toList());
    }

    public BezierCurve(Constants constants, Translation2d... waypoints) {
        this(constants, new ArrayList<>(Arrays.asList(waypoints)));
    }

    public Translation2d[] getBezierPoint(double t, int times) {
        List<Double> pointsX = waypoints.stream().map(Translation2d::getX).toList();
        List<Double> pointsY = waypoints.stream().map(Translation2d::getY).toList();
        for (int j = 0; j < times; j++) {
            List<Double> newX = new ArrayList<>();
            List<Double> newY = new ArrayList<>();
            for (int i = 0; i < pointsX.size() - 1; i++) {
                newX.add(pointsX.get(i) + (t * (pointsX.get(i + 1) - pointsX.get(i))));
                newY.add(pointsY.get(i) + (t * (pointsY.get(i + 1) - pointsY.get(i))));
            }
            pointsX = newX;
            pointsY = newY;
        }
        Translation2d[] points = new Translation2d[pointsX.size()];
        for (int i = 0; i < points.length; i++) {
            points[i] = new Translation2d(pointsX.get(i), pointsY.get(i));
        }
        return points;
    }

    @Override
    public double getX(double t) {
        List<Double> pointsX = waypoints.stream().map(Translation2d::getX).toList();
        while (pointsX.size() > 1) {
            List<Double> newX = new ArrayList<>();
            for (int i = 0; i < pointsX.size() - 1; i++) {
                newX.add(pointsX.get(i) + (t * (pointsX.get(i + 1) - pointsX.get(i))));
            }
            pointsX = newX;
        }
        return pointsX.get(0);
    }

    @Override
    public double getY(double t) {
        List<Double> pointsY = waypoints.stream().map(Translation2d::getY).toList();
        while (pointsY.size() > 1) {
            List<Double> newY = new ArrayList<>();
            for (int i = 0; i < pointsY.size() - 1; i++) {
                newY.add(pointsY.get(i) + (t * (pointsY.get(i + 1) - pointsY.get(i))));
            }
            pointsY = newY;
        }
        return pointsY.get(0);
    }
}
