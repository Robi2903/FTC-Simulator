package YourCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;
import java.util.List;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
        List<Pose2d> goals = Arrays.asList(
                new Pose2d(-50, 42, Math.toRadians(-45)),
                new Pose2d(69, 10, Math.toRadians(0)),
                new Pose2d(29, 3, Math.toRadians(180)),
                new Pose2d(-20, 9, Math.toRadians(-90))
        );

        MyOpMode2 myOpMode = new MyOpMode2();
        int stepsPerSegment = 30;
        List<Pose2d> path = myOpMode.getMultiPointPath(start, goals, stepsPerSegment);

        if (path.isEmpty()) {
            System.err.println("Failed to generate path. Check the pathfinding parameters.");
            return;
        }

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        45.0,
                        30.0,
                        Math.toRadians(180.0),
                        Math.toRadians(90.0),
                        15.0
                )
                .followTrajectorySequence(drive -> {
                    var builder = drive.trajectorySequenceBuilder(start);

                    for (int i = 1; i < path.size(); i++) {
                        Pose2d currentPose = path.get(i);

                        double curvature = 0;
                        if (i > 1) {
                            Vector2d prev = new Vector2d(path.get(i-2).getX(), path.get(i-2).getY());
                            Vector2d curr = new Vector2d(path.get(i-1).getX(), path.get(i-1).getY());
                            Vector2d next = new Vector2d(currentPose.getX(), currentPose.getY());

                            double dx1 = curr.getX() - prev.getX();
                            double dy1 = curr.getY() - prev.getY();
                            double dx2 = next.getX() - curr.getX();
                            double dy2 = next.getY() - curr.getY();

                            double t = Math.sqrt((dx1*dx1 + dy1*dy1) * (dx2*dx2 + dy2*dy2));
                            if (t > 1e-6) {
                                curvature = Math.abs(dx1*dy2 - dy1*dx2) / t;
                            }
                        }

                        double speedMultiplier = 1.0 / (1.0 + 2.0 * curvature);

                        if (i == 1) {
                            builder.splineTo(
                                    new Vector2d(currentPose.getX(), currentPose.getY()),
                                    currentPose.getHeading()
                            ).setVelConstraint((vel, accel, angVel, trackWidth) ->
                                    Math.min(45 * speedMultiplier, vel)
                            );
                        } else {
                            builder.splineToSplineHeading(
                                    currentPose,
                                    currentPose.getHeading()
                            ).setVelConstraint((vel, accel, angVel, trackWidth) ->
                                    Math.min(45 * speedMultiplier, vel)
                            );
                        }
                    }

                    return builder.build();
                });

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}