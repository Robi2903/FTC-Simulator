package InternalFiles;

import javax.swing.*;
import java.util.*;

public class MecanumRobot {
    protected boolean odoquery;
    protected boolean gyroquery;
    protected boolean posequery;
    protected boolean centric;

    protected static boolean leftreciever;
    protected static boolean rightreciever;
    protected static boolean strafereciever;
    protected static boolean gyroreciever;
    protected static boolean posereciever;

    protected PID_Controller PID_X;
    protected PID_Controller PID_Y;
    protected PID_Controller PID_Z;

    private float kp = 0.03f;
    private float ki = 0;
    private float kd = 0.0025f;

    private float kpr = 0.6f;
    private float kir = 0;
    private float kdr = 0.02f;

    protected ClientSNCH msngr;
    private boolean powerSent = false;

    private Queue<Double> speedHistory = new LinkedList<>();
    private static final int SPEED_HISTORY_SIZE = 5;
    private double lastX = 0;
    private double lastY = 0;
    private long lastUpdateTime = 0;
    private static final double SPEED_THRESHOLD = 0.1; // Reduced from 0.3
    private static final double MIN_SPEED_FOR_DETECTION = 0.05; // Minimum speed to consider obstacle detection
    private boolean isFirstUpdate = true;

    private boolean firstp = true; // Added variable
    private boolean firstg = true; // Added variable

    public MecanumRobot(Telemetry telemetry) {
        msngr = new ClientSNCH(telemetry);
        odoquery = false;
        gyroquery = false;
        centric = false;

        PID_X = new PID_Controller(kp, ki, kd);
        PID_Y = new PID_Controller(kp, ki, kd);
        PID_Z = new PID_Controller(kpr, kir, kdr);

        leftreciever = false;
        rightreciever = false;
        strafereciever = false;
        gyroreciever = false;
    }

    private void updateSpeedFromPose(double x, double y) {
        long currentTime = System.currentTimeMillis();

        if (isFirstUpdate) {
            lastX = x;
            lastY = y;
            lastUpdateTime = currentTime;
            isFirstUpdate = false;
            return;
        }

        // Calculate time difference in seconds
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
        if (deltaTime == 0) return;

        // Calculate distance moved
        double deltaX = x - lastX;
        double deltaY = y - lastY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Calculate speed in units per second
        double currentSpeed = distance / deltaTime;

        // Update speed history
        speedHistory.offer(currentSpeed);
        if (speedHistory.size() > SPEED_HISTORY_SIZE) {
            speedHistory.poll();
        }

        // Update last positions and time
        lastX = x;
        lastY = y;
        lastUpdateTime = currentTime;
    }

    public boolean isObstacleDetected() {
        // If we don't have enough speed history, or this is the first update
        if (speedHistory.size() < SPEED_HISTORY_SIZE || isFirstUpdate) {
            return false;
        }

        // Calculate average speed from history
        double avgSpeed = speedHistory.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(0.0);

        // Get the most recent speed
        double currentSpeed = speedHistory.peek();

        // Only detect obstacles if we're actually trying to move
        if (avgSpeed < MIN_SPEED_FOR_DETECTION) {
            return false;
        }

        // Calculate the speed drop percentage
        double speedDropPercentage = (avgSpeed - currentSpeed) / avgSpeed;

        System.out.println("Average Speed: " + avgSpeed +
                " Current Speed: " + currentSpeed +
                " Speed Drop %: " + speedDropPercentage);

        return speedDropPercentage > SPEED_THRESHOLD;
    }

    public void setGoToPointPIDCoeff(float kpTrans, float kiTrans, float kdTrans, float kpRot, float kiRot, float kdRot) {
        kp = kpTrans;
        ki = kiTrans;
        kd = kdTrans;

        kpr = kpRot;
        kir = kiRot;
        kdr = kdRot;

        PID_X = new PID_Controller(kp, ki, kd);
        PID_Y = new PID_Controller(kp, ki, kd);
        PID_Z = new PID_Controller(kpr, kir, kdr);
    }

    public double clip(double value, double min, double max) {
        if (value >= max) {
            return max;
        } else if (value <= min) {
            return min;
        } else {
            return value;
        }
    }

    private void sendPower(double ul, double bl, double ur, double br) {
        powerSent = true;
        if (odoquery && !Form.stopper) {
            msngr.StartClient("O,");
            odoquery = false;
        } else if (gyroquery && !Form.stopper) {
            msngr.StartClient("G,");
            gyroquery = false;
        } else if (posequery && !Form.stopper) {
            msngr.StartClient("P,");
            posequery = false;
        } else {
            msngr.StartClient("rp" + clip(ul, -1.0f, 1.0f) + "|" + clip(ur, -1.0f, 1.0f) + "|" + clip(bl, -1.0f, 1.0f) + "|" + clip(br, -1.0f, 1.0f) + ",");
        }
    }

    public void setPower(double UpLeft, double BackLeft, double UpRight, double BackRight) {
        centric = false;
        if (!Double.isNaN(UpLeft) &&
                !Double.isNaN(BackLeft) &&
                !Double.isNaN(UpRight) &&
                !Double.isNaN(BackRight) &&
                Double.isFinite(UpLeft) &&
                Double.isFinite(BackLeft) &&
                Double.isFinite(UpRight) &&
                Double.isFinite(BackRight)) {
            sendPower(UpLeft, BackLeft, UpRight, BackRight);
        }
    }

    public void setPower(double forward, double strafe, double turn) {
        double ul = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double ur = forward - strafe - turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(ul), Math.abs(bl)),
                Math.max(Math.abs(ur), Math.abs(br))
        ));

        setPower(ul/max, bl/max, ur/max, br/max);
    }

    public void goToPoint(Pose2d target, Pose2d current, double speedMultiplier, double turnMultiplier) {
        // Calculate difference in position
        double dx = target.x - current.x;
        double dy = target.y - current.y;

        // Simple proportional control - no PID for now
        double forward = dy * 0.5 * speedMultiplier;  // Forward/backward movement
        double strafe = dx * 0.5 * speedMultiplier;   // Left/right movement

        // Calculate turn power based on heading difference
        double headingError = normalizeAngle(target.heading - current.heading);
        double turn = headingError * 0.5 * turnMultiplier;

        // Ensure minimum power
        if (Math.abs(dx) > 0.1 || Math.abs(dy) > 0.1) {
            if (Math.abs(forward) < 0.3) forward = Math.signum(forward) * 0.3;
            if (Math.abs(strafe) < 0.3) strafe = Math.signum(strafe) * 0.3;
        }

        // Calculate wheel powers
        double frontLeft = forward + strafe + turn;
        double backLeft = forward - strafe + turn;
        double frontRight = forward - strafe - turn;
        double backRight = forward + strafe - turn;

        // Apply powers directly
        setPower(frontLeft, backLeft, frontRight, backRight);

        // Debug
        System.out.println("Target: " + target.x + "," + target.y);
        System.out.println("Powers FL:" + frontLeft + " BL:" + backLeft +
                " FR:" + frontRight + " BR:" + backRight);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public Vector3 getPose() {
        if (!powerSent) {
            setPower(0, 0, 0);
            powerSent = false;
        }

        if (firstp) {
            posequery = true;
            posereciever = true;
            firstp = false;
        }

        // Update speed based on position change
        updateSpeedFromPose(msngr.pose.x, msngr.pose.y);

        return new Vector3(msngr.pose.x, msngr.pose.y);
    }

    // Returns the heading in degrees
    public double getHeadingDegrees() {
        if (!powerSent) {
            setPower(0, 0, 0);
            powerSent = false;
        }

        if (firstg) {
            gyroquery = true;
            gyroreciever = true;
            firstg = false;
        }
        return msngr.gyro;
    }

    // Returns the heading in radians
    public double getHeading() {
        if (!powerSent) {
            setPower(0, 0, 0);
            powerSent = false;
        }

        if (firstg) {
            gyroquery = true;
            gyroreciever = true;
            firstg = false;
        }
        return Math.toRadians(msngr.gyro);
    }

    public void setMovement(double forward, double strafe) {
        setPower(forward, strafe, 0);
    }
}