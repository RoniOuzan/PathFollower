package pathfollower.path.path;

import pathfollower.math.MathUtil;
import pathfollower.math.geometry.Pose2d;
import pathfollower.math.geometry.Rotation2d;
import pathfollower.math.geometry.Translation2d;
import pathfollower.path.Path;
import pathfollower.path.util.Waypoint;

import java.util.List;

public class Quintic extends Path {
    public Quintic(Constants constants, List<Waypoint> waypoints) {
        super(constants, 0.0001, waypoints);
    }

    public Quintic(Constants constants, Waypoint... waypoints) {
        super(constants, 0.0001, waypoints);
    }

    @Override
    public Translation2d getLocation(double t) {
        t = Math.max(this.dx / 2, Math.min(1 - (this.dx / 2), t));

        double size = this.getWaypoints().size();
        int waypointIndex = (int) (t * (size - 1));
        t = (size - 1) * (t - (waypointIndex / (size - 1))); // Makes t between 0 and 1

        Waypoint waypoint1 = this.getWaypoint(waypointIndex);
        Waypoint waypoint2 = this.getWaypoint(waypointIndex + 1);

        double distance = waypoint1.getDistance(waypoint2);

        Translation2d point1 = waypoint1;
        Translation2d point2 = waypoint2;

        Rotation2d angle1 = Rotation2d.fromDegrees(waypoint1.getMovementAngle());
        Rotation2d angle2 = Rotation2d.fromDegrees(waypoint1.getMovementAngle() + 180);
        while (point1.getDistance(point2) <= 3 * this.dx) {
            angle1 = Rotation2d.fromDegrees(maxAccel(angle(point1, point2), angle1.getDegrees(), 5));
            angle2 = Rotation2d.fromDegrees(maxAccel(angle(point2, point2), angle2.getDegrees(), 5));

            point1.plus(new Translation2d(distance * 0.01, angle1));
            point2.plus(new Translation2d(distance * 0.01, angle2));
        }

        if (t > 0.5)
            return point2;
        return point1;
    }

    private double angle(Translation2d point1, Translation2d point2) {
        return Math.toDegrees(Math.atan2(point2.getY() - point1.getY(), point2.getX() - point1.getX()));
    }

    private double maxAccel(double vel, double lastVel, double maxAccel) {
        if (vel - lastVel > maxAccel) {
            return vel + maxAccel;
        } else if (lastVel - vel > maxAccel) {
            return vel - maxAccel;
        }

        return vel;
    }

    @Override
    public double getX(double t) {
        return this.getLocation(t).getX();
    }

    @Override
    public double getY(double t) {
        return this.getLocation(t).getY();
    }
}
