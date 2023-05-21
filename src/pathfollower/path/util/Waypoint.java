package pathfollower.path.util;

import pathfollower.math.geometry.Rotation2d;
import pathfollower.math.geometry.Translation2d;

public class Waypoint extends Translation2d {
    private final double heading;
    private final double movementAngle;

    public Waypoint(double x, double y, double heading, double movementAngle) {
        super(x, y);
        this.heading = heading;
        this.movementAngle = movementAngle;
    }

    public Waypoint(double distance, Rotation2d angle, double heading, double movementAngle) {
        super(distance, angle);
        this.heading = heading;
        this.movementAngle = movementAngle;
    }

    public Waypoint(Translation2d translation2d, double heading, double movementAngle) {
        super(translation2d.getX(), translation2d.getY());
        this.heading = heading;
        this.movementAngle = movementAngle;
    }

    public double getHeading() {
        return heading;
    }

    public double getMovementAngle() {
        return movementAngle;
    }
}
