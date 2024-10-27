package YourCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.util.*;

public class MyOpMode2 {
    private static final double POSITION_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE = Math.toRadians(5.0);
    private static final boolean DEBUG = true;
    private static final double MIN_STEP_SIZE = 2.0;
    private static final double MAX_STEP_SIZE = 15.0;
    private static final double PRUNING_TOLERANCE = 5.0;
    private static final double OBSTACLE_WEIGHT = 1.5;

    public List<Pose2d> getMultiPointPath(Pose2d start, List<Pose2d> goals, int stepsPerSegment) {
        if (goals.isEmpty()) return new ArrayList<>();

        // Optimize goal order using nearest neighbor with look-ahead
        List<Pose2d> optimizedGoals = optimizeGoalOrder(start, goals);

        if (DEBUG) {
            System.out.println("Generating optimized path through " + optimizedGoals.size() + " points");
        }

        // Generate complete path through optimized points
        List<Pose2d> completePath = new ArrayList<>();
        completePath.add(start);

        Pose2d currentPos = start;
        for (Pose2d goal : optimizedGoals) {
            List<Pose2d> segmentPath = generateOptimizedPathSegment(currentPos, goal, stepsPerSegment);
            completePath.addAll(segmentPath.subList(completePath.isEmpty() ? 0 : 1, segmentPath.size()));
            currentPos = goal;
        }

        // Apply path optimization techniques
        completePath = prunePath(completePath);
        completePath = smoothPath(completePath);

        return completePath;
    }

    private List<Pose2d> optimizeGoalOrder(Pose2d start, List<Pose2d> goals) {
        List<Pose2d> optimizedGoals = new ArrayList<>(goals);
        List<Pose2d> result = new ArrayList<>();
        Pose2d currentPos = start;

        while (!optimizedGoals.isEmpty()) {
            int bestIndex = 0;
            double bestScore = Double.MAX_VALUE;

            // Look ahead up to 3 points to find better global path
            for (int i = 0; i < optimizedGoals.size(); i++) {
                double score = calculatePathScore(currentPos, optimizedGoals, i, 3);
                if (score < bestScore) {
                    bestScore = score;
                    bestIndex = i;
                }
            }

            result.add(optimizedGoals.get(bestIndex));
            currentPos = optimizedGoals.get(bestIndex);
            optimizedGoals.remove(bestIndex);
        }

        return result;
    }

    private double calculatePathScore(Pose2d start, List<Pose2d> points, int startIndex, int lookAhead) {
        double score = 0;
        Pose2d current = start;

        for (int i = 0; i < lookAhead && startIndex + i < points.size(); i++) {
            Pose2d next = points.get(startIndex + i);
            score += distance(current, next) * (1 + i * 0.1); // Penalize longer look-ahead slightly
            current = next;
        }

        return score;
    }

    private List<Pose2d> generateOptimizedPathSegment(Pose2d start, Pose2d goal, int numberOfSteps) {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(start);

        double totalDistance = distance(start, goal);
        Vector2d controlPoint = calculateOptimalControlPoint(start, goal);

        for (int i = 1; i < numberOfSteps - 1; i++) {
            double t = (double) i / (numberOfSteps - 1);

            // Dynamic step size based on distance to goal and local complexity
            double dynamicStepSize = calculateDynamicStepSize(t, totalDistance, start, goal);
            t = adjustStepParameter(t, dynamicStepSize);

            // Enhanced Bezier curve calculation with dynamic control point
            Vector2d point = calculateBezierPoint(t, start, controlPoint, goal);
            double heading = calculateOptimalHeading(t, start, controlPoint, goal);

            waypoints.add(new Pose2d(point.getX(), point.getY(), heading));
        }

        waypoints.add(goal);
        return waypoints;
    }

    private Vector2d calculateOptimalControlPoint(Pose2d start, Pose2d goal) {
        double distanceWeight = Math.min(1.0, distance(start, goal) / 100.0);
        double midX = start.getX() + (goal.getX() - start.getX()) * 0.5;
        double midY = start.getY() + (goal.getY() - start.getY()) * 0.5;

        // Calculate perpendicular offset based on path characteristics
        double perpDistance = Math.min(30, Math.abs(goal.getY() - start.getY()) * 0.5);
        double offsetX = -(goal.getY() - start.getY()) * distanceWeight;
        double offsetY = (goal.getX() - start.getX()) * distanceWeight;

        double magnitude = Math.hypot(offsetX, offsetY);
        if (magnitude > 0) {
            offsetX = offsetX / magnitude * perpDistance;
            offsetY = offsetY / magnitude * perpDistance;
        }

        return new Vector2d(midX + offsetX, midY + offsetY);
    }

    private double calculateDynamicStepSize(double t, double totalDistance, Pose2d start, Pose2d goal) {
        // Smaller steps near start/end points and in areas of high curvature
        double baseStepSize = Math.min(MAX_STEP_SIZE, Math.max(MIN_STEP_SIZE, totalDistance / 20.0));
        double endpointFactor = 4.0 * t * (1 - t); // Smaller steps near endpoints
        return baseStepSize * (0.5 + endpointFactor);
    }

    private double adjustStepParameter(double t, double stepSize) {
        // Adjust the interpolation parameter based on step size
        return Math.pow(t, 1.0 + stepSize / MAX_STEP_SIZE);
    }

    private Vector2d calculateBezierPoint(double t, Pose2d start, Vector2d control, Pose2d goal) {
        double x = Math.pow(1 - t, 2) * start.getX() +
                2 * (1 - t) * t * control.getX() +
                Math.pow(t, 2) * goal.getX();

        double y = Math.pow(1 - t, 2) * start.getY() +
                2 * (1 - t) * t * control.getY() +
                Math.pow(t, 2) * goal.getY();

        return new Vector2d(x, y);
    }

    private double calculateOptimalHeading(double t, Pose2d start, Vector2d control, Pose2d goal) {
        // Calculate tangent vector to the curve at point t
        double dx = 2 * (1 - t) * (control.getX() - start.getX()) +
                2 * t * (goal.getX() - control.getX());
        double dy = 2 * (1 - t) * (control.getY() - start.getY()) +
                2 * t * (goal.getY() - control.getY());

        return Math.atan2(dy, dx);
    }

    private List<Pose2d> prunePath(List<Pose2d> path) {
        if (path.size() < 3) return path;

        List<Pose2d> prunedPath = new ArrayList<>();
        prunedPath.add(path.get(0));

        int i = 1;
        while (i < path.size() - 1) {
            Pose2d current = path.get(i);
            Pose2d next = path.get(i + 1);

            // Check if point is necessary based on curvature and distance
            if (isSignificantPoint(prunedPath.get(prunedPath.size() - 1), current, next)) {
                prunedPath.add(current);
            }
            i++;
        }

        prunedPath.add(path.get(path.size() - 1));
        return prunedPath;
    }

    private boolean isSignificantPoint(Pose2d prev, Pose2d current, Pose2d next) {
        // Calculate angle between segments
        double angle1 = Math.atan2(current.getY() - prev.getY(), current.getX() - prev.getX());
        double angle2 = Math.atan2(next.getY() - current.getY(), next.getX() - current.getX());
        double angleDiff = Math.abs(normalizeAngle(angle2 - angle1));

        // Calculate distances
        double d1 = distance(prev, current);
        double d2 = distance(current, next);
        double d3 = distance(prev, next);

        // Point is significant if:
        // 1. There's a significant direction change
        // 2. The point is not too close to its neighbors
        // 3. The point is not collinear with its neighbors (within tolerance)
        return angleDiff > Math.PI / 6 ||
                Math.abs(d1 + d2 - d3) > PRUNING_TOLERANCE ||
                d1 > MAX_STEP_SIZE * 2 ||
                d2 > MAX_STEP_SIZE * 2;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double distance(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private List<Pose2d> smoothPath(List<Pose2d> path) {
        if (path.size() < 3) return path;

        List<Pose2d> smoothedPath = new ArrayList<>();
        smoothedPath.add(path.get(0));

        for (int i = 1; i < path.size() - 1; i++) {
            Pose2d prev = path.get(i - 1);
            Pose2d curr = path.get(i);
            Pose2d next = path.get(i + 1);

            // Enhanced smoothing using weighted averaging
            Vector2d smoothedPoint = smoothPoint(prev, curr, next);
            double smoothedHeading = smoothHeading(prev, curr, next);

            smoothedPath.add(new Pose2d(smoothedPoint.getX(), smoothedPoint.getY(), smoothedHeading));
        }

        smoothedPath.add(path.get(path.size() - 1));
        return smoothedPath;
    }

    private Vector2d smoothPoint(Pose2d prev, Pose2d curr, Pose2d next) {
        // Weight factors for smoothing
        double w1 = 0.25; // previous point weight
        double w2 = 0.5;  // current point weight
        double w3 = 0.25; // next point weight

        double x = w1 * prev.getX() + w2 * curr.getX() + w3 * next.getX();
        double y = w1 * prev.getY() + w2 * curr.getY() + w3 * next.getY();

        return new Vector2d(x, y);
    }

    private double smoothHeading(Pose2d prev, Pose2d curr, Pose2d next) {
        double angle1 = Math.atan2(curr.getY() - prev.getY(), curr.getX() - prev.getX());
        double angle2 = Math.atan2(next.getY() - curr.getY(), next.getX() - curr.getX());

        // Weighted average of the two angles
        double weightedAngle = angle1 * 0.5 + angle2 * 0.5;
        return normalizeAngle(weightedAngle);
    }
}