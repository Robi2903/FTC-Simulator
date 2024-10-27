package YourCode;

import InternalFiles.OpMode;
import InternalFiles.Pose2d;
import InternalFiles.RegisterOpMode;
import InternalFiles.Vector3;
import java.util.*;

@RegisterOpMode
public class MovementOpMode extends OpMode {
    private final Pose2d FINAL_TARGET = new Pose2d(-10, 55, Math.PI / 2); // Initialize here
    private Pose2d intermediateTarget;
    private boolean startedMoving;
    private boolean hasReachedFinalTarget;
    private static final double POSITION_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE = Math.PI / 36;
    private static final double OBSTACLE_AVOIDANCE_DISTANCE = 10.0; // Distance to keep from obstacles
    private static final double PATHFINDING_STEP_SIZE = 5.0; // Step size for exploring new positions
    private List<Pose2d> currentPath; // Store the planned path
    private Pose2d lastPose; // Store the last known position

    @Override
    public void init() {
        intermediateTarget = FINAL_TARGET;
        startedMoving = false;
        hasReachedFinalTarget = false;
        currentPath = new ArrayList<>();
        lastPose = null;
        telemetry.addData("Initial Target Point", String.format("x:%.2f, y:%.2f, h:%.2f",
                FINAL_TARGET.x, FINAL_TARGET.y, FINAL_TARGET.heading));
        telemetry.addData("Status", "Initialized - Waiting to start");
    }

    @Override
    public void loop() {
        Vector3 currentVector = Robot.getPose();
        Pose2d currentPose = new Pose2d(currentVector.x, currentVector.y, Robot.getHeading());

        telemetry.addData("Current Position", String.format("x:%.2f, y:%.2f, h:%.2f",
                currentPose.x, currentPose.y, currentPose.heading));
        telemetry.addData("Final Target", String.format("x:%.2f, y:%.2f, h:%.2f",
                FINAL_TARGET.x, FINAL_TARGET.y, FINAL_TARGET.heading));

        // Check for obstacles based on movement
        if (lastPose != null) {
            double distanceMoved = distance(currentPose, lastPose);
            if (distanceMoved < POSITION_TOLERANCE) {
                telemetry.addData("Obstacle Detected", true);
            } else {
                telemetry.addData("Obstacle Detected", false);
            }
        }
        lastPose = currentPose;

        // If we haven't started moving or we detect an obstacle, plan a new path
        if (!startedMoving || Robot.isObstacleDetected()) {
            List<Pose2d> newPath = findPathToTarget(currentPose, FINAL_TARGET);
            if (!newPath.isEmpty()) {
                currentPath = newPath;
                intermediateTarget = currentPath.get(0);
                startedMoving = true;
                telemetry.addData("New Path", "Path found and set");
            } else {
                telemetry.addData("New Path", "No path found");
            }
        }

        // Check if we've reached the intermediate target
        double distanceToIntermediate = distance(currentPose, intermediateTarget);
        if (distanceToIntermediate < POSITION_TOLERANCE && !currentPath.isEmpty()) {
            currentPath.remove(0); // Remove reached waypoint
            if (!currentPath.isEmpty()) {
                intermediateTarget = currentPath.get(0);
            }
        }

        // Check if we've reached the final target
        double distanceToFinal = distance(currentPose, FINAL_TARGET);
        double headingDifference = Math.abs(normalizeAngle(currentPose.heading - FINAL_TARGET.heading));

        telemetry.addData("Distance to Final", String.format("%.2f", distanceToFinal));
        telemetry.addData("Distance to Intermediate", String.format("%.2f", distanceToIntermediate));

        if (distanceToFinal < POSITION_TOLERANCE && headingDifference < HEADING_TOLERANCE) {
            hasReachedFinalTarget = true;
            telemetry.addData("Status", "Reached final target");
            Robot.setPower(0, 0, 0); // Stop the robot
        } else if (!hasReachedFinalTarget) {
            Robot.goToPoint(intermediateTarget, currentPose, 1.0, 1.0); // Move towards the intermediate target
        }
    }

    private double calculateSpeed(Pose2d currentPose) {
        // Implement your logic to calculate the current speed of the robot
        // This is a placeholder implementation
        return Math.sqrt(currentPose.x * currentPose.x + currentPose.y * currentPose.y);
    }

    private List<Pose2d> findPathToTarget(Pose2d start, Pose2d goal) {
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        Set<String> closedSet = new HashSet<>();
        Map<String, Node> nodeMap = new HashMap<>();

        Node startNode = new Node(start, null);
        startNode.g = 0;
        startNode.h = distance(start, goal);
        openSet.add(startNode);
        nodeMap.put(nodeKey(start), startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (distance(current.pose, goal) < POSITION_TOLERANCE) {
                return reconstructPath(current);
            }

            closedSet.add(nodeKey(current.pose));

            // Generate neighbors in 8 directions
            double[] directions = {0, Math.PI/4, Math.PI/2, 3*Math.PI/4, Math.PI,
                    5*Math.PI/4, 3*Math.PI/2, 7*Math.PI/4};

            for (double direction : directions) {
                Pose2d neighborPose = new Pose2d(
                        current.pose.x + PATHFINDING_STEP_SIZE * Math.cos(direction),
                        current.pose.y + PATHFINDING_STEP_SIZE * Math.sin(direction),
                        0);

                if (!isValidPosition(neighborPose) || closedSet.contains(nodeKey(neighborPose))) {
                    continue;
                }

                double tentativeG = current.g + distance(current.pose, neighborPose);
                Node neighborNode = nodeMap.getOrDefault(nodeKey(neighborPose), new Node(neighborPose, current));

                if (tentativeG < neighborNode.g) {
                    neighborNode.parent = current;
                    neighborNode.g = tentativeG;
                    neighborNode.h = distance(neighborPose, goal);
                    openSet.add(neighborNode);
                    nodeMap.put(nodeKey(neighborPose), neighborNode);
                }
            }
        }

        return new ArrayList<>(); // Return empty path if no path found
    }

    private boolean isValidPosition(Pose2d pose) {
        // Check if position is within field boundaries
        if (pose.x < -100 || pose.x > 100 || pose.y < -100 || pose.y > 100) {
            return false;
        }

        // Simulate checking for obstacles around the position
        Vector3 currentPos = Robot.getPose();
        double distanceToCheck = distance(new Pose2d(currentPos.x, currentPos.y, 0), pose);

        if (distanceToCheck < OBSTACLE_AVOIDANCE_DISTANCE) {
            return !Robot.isObstacleDetected();
        }

        return true;
    }

    private List<Pose2d> reconstructPath(Node endNode) {
        List<Pose2d> path = new ArrayList<>();
        Node current = endNode;
        while (current != null) {
            path.add(0, current.pose);
            current = current.parent;
        }
        return path;
    }

    private String nodeKey(Pose2d pose) {
        return String.format("%.1f:%.1f", pose.x, pose.y);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return Math.abs(angle);
    }

    private double distance(Pose2d p1, Pose2d p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    private class Node implements Comparable<Node> {
        Pose2d pose;
        Node parent;
        double g; // Cost from start
        double h; // Heuristic (estimated cost to goal)

        Node(Pose2d pose, Node parent) {
            this.pose = pose;
            this.parent = parent;
        }

        double f() {
            return g + h;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.f(), other.f());
        }
    }
}