package pathfollower.path;

import pathfollower.path.pid.PIDPreset;
import pathfollower.gui.Frame;
import pathfollower.gui.types.draw.DrawCentered;
import pathfollower.gui.types.field.ZeroCenter;
import pathfollower.math.MathUtil;
import pathfollower.math.geometry.Dimension2d;
import pathfollower.math.geometry.Pose2d;
import pathfollower.math.geometry.Rotation2d;
import pathfollower.math.geometry.Translation2d;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;

public class PathFollowerGUI extends Frame implements ZeroCenter, DrawCentered {
    private static final boolean IS_CHARGED_UP_FIELD = false;

    private static final double DEFAULT_MAX_VALUE = 10;
    private static final Dimension2d DIMENSION = new Dimension2d(1713, 837);
    private static final double PIXELS_IN_ONE_UNIT = convertMaxValueToPixels(DEFAULT_MAX_VALUE);

    private static final double FPS = 20;
    private static final double ROBOT_WIDTH = 0.91;
    private static final double BUMPER_WIDTH = 0.08;

    private final Path path;
    private final Robot robot;

    private final Follower follower;

    private double maxValue = DEFAULT_MAX_VALUE;

    public PathFollowerGUI() {
        super("Bezier Curve", DIMENSION, PIXELS_IN_ONE_UNIT);

        this.path = new BezierCurve(new BezierCurve.Constants(4.5, 4.5, 0.5),
                new Translation2d(2, -3),
                new Translation2d(-5, 2),
                new Translation2d(-2, 1),
                new Translation2d(-6, -2),
                new Translation2d(3, 3),
                new Translation2d(7, -1)
        );
        this.robot = new Robot(new Pose2d(this.path.getStartPoint(), Rotation2d.fromDegrees(0)),
                new Robot.Constants(5, 1 / FPS));

        this.follower = new Follower(this.path, this.robot,
                new Follower.Constants(0, 270,
                        new PIDPreset(3, 0, 0, 1, 10),
                        new PIDPreset(2, 0, 0, 5, 10)));

        this.follower.start();
        this.start();
    }

    public void drawBackground() {
        if (IS_CHARGED_UP_FIELD)
            this.drawImage(new ImageIcon("src/pathfollower/path/Field.png").getImage(), 0, 0, DIMENSION.getX(), DIMENSION.getY());
        else
            this.drawGrid();

//        for (int i = 0; i < this.path.getWaypoints().size() - 1; i++) {
//            Color color = new Color(0, (int) (255 * (i / (this.path.getWaypoints().size() - 1d))), 0);
//            Translation2d[] bezierPoints = this.path.getBezierPoint(this.follower.getState().t(), i);
//            this.drawConnectedPoints(color, bezierPoints);
//            for (Translation2d bezierPoint : bezierPoints) {
//                this.fillPoint(bezierPoint.getX(), bezierPoint.getY(), convertPixelsToUnits(4), color);
//            }
//        }

        for (double t = this.path.getDifferentBetweenTs(); t < 1; t += this.path.getDifferentBetweenTs()) {
            this.drawRobotPose(this.path.getLocation(t));
        }

        for (Translation2d waypoint : this.path.getWaypoints()) {
            this.drawWaypoint(waypoint);
        }
    }

    public void displayRobot() {
        BezierCurve.State state = this.follower.getState();
        Pose2d robot = this.robot.getPosition();

        Translation2d setpoint = this.robot.getPosition().getTranslation()
                .plus(new Translation2d(this.follower.getPidController().getSetpoint().position,
                        this.robot.getVelocity().getTranslation().getAngle()));
        this.fillPoint(setpoint.getX(), setpoint.getY(), convertPixelsToUnits(5), Color.GREEN);

        double curvatureRadius = this.path.getCurvatureRadius(state.t());
        Translation2d curvature = state.pose().getTranslation()
                .plus(new Translation2d(
                        curvatureRadius,
                        this.path.getAngle(state.t()).plus(Rotation2d.fromDegrees(curvatureRadius > 0 ? -90 : 90))
                ));
        this.drawPoint(curvature.getX(), curvature.getY(), Math.abs(curvatureRadius), Color.BLUE);

        double bumperRadius = Math.hypot(ROBOT_WIDTH / 2, ROBOT_WIDTH / 2);
        Translation2d[] bumperEdges = new Translation2d[4];
        for (int i = 0; i < bumperEdges.length; i++) {
            double radians = Math.toRadians(45 + (90 * i)) + robot.getRotation().getRadians();
            bumperEdges[i] = robot.getTranslation().plus(new Translation2d(bumperRadius * Math.cos(radians), bumperRadius * Math.sin(radians)));
        }
        this.fillPolygon(Color.RED, bumperEdges);

        double robotRadius = Math.hypot((ROBOT_WIDTH / 2) - BUMPER_WIDTH, (ROBOT_WIDTH / 2) - BUMPER_WIDTH);
        Translation2d[] robotEdges = new Translation2d[4];
        for (int i = 0; i < robotEdges.length; i++) {
            double radians = Math.toRadians(45 + (90 * i)) + robot.getRotation().getRadians();
            robotEdges[i] = robot.getTranslation().plus(new Translation2d(robotRadius * Math.cos(radians), robotRadius * Math.sin(radians)));
        }
        this.fillPolygon(Color.GRAY, robotEdges);
    }

    public void writeValues() {
        String[] texts = {
                "T: " + MathUtil.limitDot(this.follower.getState().t(), 4),
                "Pose: (" + MathUtil.limitDot(this.robot.getPosition().getTranslation().getX(), 3) + ", "
                        + MathUtil.limitDot(this.robot.getPosition().getTranslation().getY(), 3) + ")",
                "Heading: " + MathUtil.limitDot(this.robot.getPosition().getRotation().getDegrees(), 3) + " Deg",
                "Vector: (" + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getX(), 3) + ", "
                        + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getY(), 3) + ")",
                "Velocity: " + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getNorm(), 3) + "m/s",
                "Accelration: " + MathUtil.limitDot(this.robot.getAcceleration(), 3) + "m/s",
                "Omega Velocity: " + MathUtil.limitDot(this.robot.getVelocity().getRotation().getDegrees(), 3) + " deg/s",
                "Distance: " + MathUtil.limitDot(this.path.getDistance(0, this.follower.getState().t()), 3) + " / " + MathUtil.limitDot(this.path.getPathLength(), 3),
                "Curvature Radius: " + MathUtil.limitDot(this.path.getCurvatureRadius(this.follower.getState().t()), 3)
        };

        double size = convertPixelsToUnits(20);
        double space = convertPixelsToUnits(10);
        for (int i = 0; i < texts.length; i++) {
            this.write(-this.maxValue + convertPixelsToUnits(5), (this.maxValue * ((double) DIMENSION.getY() / DIMENSION.getX()) - ((size + space) * (i + 1))), texts[i], size, Color.BLACK);
        }
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        Translation2d mouseLocation = this.getMouseTranslation(e);

        for (int i = this.path.getWaypoints().size() - 1; i >= 0; i--) {
            if (this.path.getWaypoints().get(i).getDistance(mouseLocation) <= convertPixelsToUnits(50)) {
                this.path.setWaypoint(i, mouseLocation);
                break;
            }
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyChar() == 'r' || e.getKeyChar() == 'R') {
            this.follower.reset();
        } else if (e.getKeyChar() == 't' || e.getKeyChar() == 'T') {
            this.follower.setRunning(!this.follower.isRunning());
        }
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        this.maxValue += e.getPreciseWheelRotation();
    }

    public void start() {
        double t = 0;
        int direction = 1;

        while (true) {
            this.setPixelsInOneUnit(convertMaxValueToPixels(this.maxValue));

            this.clearFrame();
            this.drawBackground();
            this.follower.update();
            this.displayRobot();
            this.writeValues();
            this.repaint();

            t += this.path.getDifferentBetweenTs() * direction;
            if (t > 1) {
                t = 1;
                direction = -1;
            } else if (t < 0) {
                t = 0;
                direction = 1;
            }
            try {
                Thread.sleep((long) (1000 / FPS));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void drawRobotPose(Translation2d pose) {
        this.fillPoint(pose.getX(), pose.getY(), convertPixelsToUnits(2), Color.RED);
        this.drawPoint(pose.getX(), pose.getY(), convertPixelsToUnits(2), new Color(20, 20, 20));
    }

    private void drawWaypoint(Translation2d waypoint) {
        this.fillPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(6), Color.BLUE);
        this.drawPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(7), new Color(200, 200, 200));
    }

    private void drawGrid() {
        for (double i = (int) -Math.floor(this.maxValue); i <= this.maxValue; i += 0.25) {
            this.drawThinLine(i, this.getDimensionWithUnits().getY() / -2, i, this.getDimensionWithUnits().getY() / 2,
                    Math.floor(i * 10) / 10d % 1 == 0 ? new Color(60, 60, 60) : new Color(230, 230, 230));
        }

        for (double i = (int) -Math.floor(this.maxValue); i <= this.maxValue; i += 0.25) {
            this.drawThinLine(this.getDimensionWithUnits().getX() / -2, i, this.getDimensionWithUnits().getX() / 2, i,
                    Math.floor(i * 10) / 10d % 1 == 0 ? new Color(60, 60, 60) : new Color(230, 230, 230));
        }

        this.drawLine(0, this.getDimensionWithUnits().getY() / -2, 0, this.getDimensionWithUnits().getY() / 2, convertPixelsToUnits(5), Color.BLACK);
        this.drawLine(this.getDimensionWithUnits().getX() / -2, 0, this.getDimensionWithUnits().getX() / 2, 0, convertPixelsToUnits(5), Color.BLACK);

        double textSize = convertPixelsToUnits(20);
        for (int i = 0; i <= this.maxValue; i += this.maxValue / 10) {
            this.write(0, i - (textSize / 2), " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(0, -i - (textSize / 2), -i + "", textSize, Color.BLACK);
        }
        for (int i = 0; i <= this.maxValue; i += this.maxValue / 10 ) {
            this.write(i - (textSize / 2), -textSize, " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(-i - (textSize / 2), -textSize, -i + "", textSize, Color.BLACK);
        }
    }

    private static double convertMaxValueToPixels(double maxValue) {
        return (DIMENSION.getX() / maxValue) / 2;
    }
}
