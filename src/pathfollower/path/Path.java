package pathfollower.path;

import pathfollower.math.geometry.Pose2d;
import pathfollower.math.geometry.Rotation2d;
import pathfollower.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

public abstract class Path {
    private final double dx;

    private final List<Translation2d> waypoints;
    private final double differentBetweenTs;

    private final Constants constants;

    public Path(Constants constants, double dx, List<Translation2d> waypoints) {
        this.constants = constants;
        this.dx = dx;
        this.waypoints = waypoints;
        this.differentBetweenTs = 0.01 / waypoints.size();
    }

    public Path(Constants constants, double dx, Translation2d... waypoints) {
        this(constants, dx, new ArrayList<>(Arrays.asList(waypoints)));
    }

    public BezierCurve.State getClosestPoint(Pose2d robotPose) {
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

        return new BezierCurve.State(output, minT);
    }

    public abstract Pose2d getVelocity(State state, Pose2d robot, double velocity, double omega);

    public abstract double getX(double t);

    public abstract double getY(double t);

    public Translation2d getLocation(double t) {
        return new Translation2d(getX(t), getY(t));
    }

    public Rotation2d getAngle(double t) {
        Translation2d angle = this.getLocation(t + this.dx).minus(this.getLocation(t - this.dx));
        return new Rotation2d(angle.getX(), angle.getY());
    }

    public Pose2d getPosition(double t) {
        return new Pose2d(this.getLocation(t), this.getAngle(t));
    }

    private double calculateDerivative(double x, Function<Double, Double> function) {
        return (function.apply(x + this.dx) - function.apply(x - this.dx)) / (2 * this.dx);
    }


    public record State(Pose2d pose, double t) {}


    public record Constants(double maxVel, double maxAccel, double errorCorrectorPower) {}

}
