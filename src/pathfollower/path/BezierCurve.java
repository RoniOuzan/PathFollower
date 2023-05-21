package pathfollower.path;

import pathfollower.math.geometry.Pose2d;
import pathfollower.math.geometry.Rotation2d;
import pathfollower.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

@SuppressWarnings(value = "unused")
public class BezierCurve extends Path {

    public BezierCurve(Constants constants, List<Translation2d> waypoints) {
        super(constants, 0.0001, waypoints);
    }

    public BezierCurve(Constants constants, Translation2d... waypoints) {
        this(constants, new ArrayList<>(Arrays.asList(waypoints)));
    }

    @Override
    public State getClosestPoint(Pose2d robotPose) {
        double minT = 0;
        double minDistance = Double.MAX_VALUE;
        Pose2d output = this.getPosition(0);
        for (double t = this.differentBetweenTs; t <= 1; t += this.differentBetweenTs) {
            Pose2d pose = this.getPosition(t);
            double distance = robotPose.getTranslation().getDistance(pose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                output = pose;
                minT = t;
            }
        }

        return new State(output, minT);
    }

    @Override
    public Pose2d getVelocity(State state, Pose2d robot, double velocity, double omega) {
        Translation2d vector = new Translation2d(1 - constants.errorCorrectorPower(), this.getAngle(state.t()))
                .plus(state.pose().getTranslation().minus(robot.getTranslation()).times(constants.errorCorrectorPower()));

        double curvature = 1 / Math.abs(this.getCurvatureRadius(state.t()));
        velocity = Math.min(constants.maxVel() - Math.min(curvature, 3.5), velocity);

        return new Pose2d(new Translation2d(velocity, vector.getAngle()), Rotation2d.fromDegrees(omega));
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
