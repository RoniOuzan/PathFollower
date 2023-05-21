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

    protected final List<Translation2d> waypoints;
    protected final double differentBetweenTs;

    protected final Constants constants;

    public Path(Constants constants, double dx, List<Translation2d> waypoints) {
        this.constants = constants;
        this.dx = dx;
        this.waypoints = waypoints;
        this.differentBetweenTs = 0.01 / waypoints.size();
    }

    public Path(Constants constants, double dx, Translation2d... waypoints) {
        this(constants, dx, new ArrayList<>(Arrays.asList(waypoints)));
    }

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

    public double getXDerivative(double t) {
        return this.calculateDerivative(t, this::getX);
    }

    public double getYDerivative(double t) {
        return this.calculateDerivative(t, this::getY);
    }

    public double getXSecondDerivative(double t) {
        return this.calculateDerivative(t, this::getXDerivative);
    }

    public double getYSecondDerivative(double t) {
        return this.calculateDerivative(t, this::getYDerivative);
    }

    protected double calculateDerivative(double x, Function<Double, Double> function) {
        return (function.apply(x + this.dx) - function.apply(x - this.dx)) / (2 * this.dx);
    }

    public double getDistance(double t1, double t2) {
        return calculateIntegral(t1, t2, t -> Math.hypot(this.getXDerivative(t), this.getYDerivative(t)));
    }

    public double getDistance(double t) {
        return this.getDistance(0, t);
    }

    public double calculateIntegral(double a, double b, Function<Double, Double> function) {
        double sum = 0;
        for (double t = a; t < b; t += this.dx) {
            sum += (function.apply(t - this.dx) + function.apply(t + this.dx)) * (0.5 * this.dx);
        }
        return sum;
    }

    public double getCurvatureRadius(double t) {
        double d1x = this.getXDerivative(t);
        double d1y = this.getYDerivative(t);
        double d2x = this.getXSecondDerivative(t);
        double d2y = this.getYSecondDerivative(t);

        return Math.pow((d1x * d1x) + (d1y * d1y), 1.5) / ((d1x * d2y) - (d1y * d2x));
    }

    public Translation2d getFinalPoint() {
        return this.waypoints.get(this.waypoints.size() - 1);
    }

    public Translation2d getStartPoint() {
        return this.waypoints.get(0);
    }

    public List<Translation2d> getWaypoints() {
        return waypoints;
    }

    public void setWaypoint(int index, Translation2d waypoint) {
        this.waypoints.set(index, waypoint);
    }

    public double getDifferentBetweenTs() {
        return differentBetweenTs;
    }

    public double getPathLength() {
        return this.getDistance(1);
    }

    public Constants getConstants() {
        return constants;
    }

    public record State(Pose2d pose, double t) {}


    public record Constants(double maxVel, double maxAccel, double errorCorrectorPower) {}

}
