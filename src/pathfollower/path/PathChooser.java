package pathfollower.path;

import pathfollower.math.geometry.Translation2d;
import pathfollower.path.path.BezierCurve;
import pathfollower.path.path.Quintic;
import pathfollower.path.util.Waypoint;

public enum PathChooser {
    BEZIER_CURVE(new BezierCurve(new Path.Constants(4.5, 4.5, 0.5),
            new Translation2d(2, -3),
            new Translation2d(-5, 2),
            new Translation2d(-2, 1),
            new Translation2d(-6, -2),
            new Translation2d(3, 3),
            new Translation2d(7, -1))),
    QUINTIC(new Quintic(new Path.Constants(4.5, 4.5, 0.5),
            new Waypoint(0, 0, 0, 0),
            new Waypoint(1, 0, 0, 0)
            ))
    ;

    private final Path path;

    PathChooser(Path path) {
        this.path = path;
    }

    public Path getPath() {
        return path;
    }
}
