package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class TrajectoryFactory {
//TODO: Make it so all of the intaking spike mark paths (spike mark to end) are at a 0.58 dt mutliplier
    // Poses - all positions on the field
    public static Pose goalStartPos = new Pose(26.0, 130.0, Math.toRadians(-40));
    public static Pose farStartPos = new Pose(56.0, 8.0, Math.toRadians(180));
    public static Pose farParkPos = new Pose(36.0, 8.0, Math.toRadians(180));
    public static Pose scorePos = new Pose(30.0, 119.0, Math.toRadians(-42));

    // Spike Mark 1 positions
    public static Pose spikeMark1PosPre = new Pose(41.6+1.4 , 83.6, Math.toRadians(180));
    public static Pose spikeMark1PosOuter = new Pose(26.6-6.6 , 83.6, Math.toRadians(180));

    // Spike Mark 2 positions
    public static Pose spikeMark2PosPre = new Pose(41.6+1.4 , 60.0, Math.toRadians(180));
    public static Pose spikeMark2PosOuter = new Pose(26.6-6.6 , 60.0, Math.toRadians(180));

    // Spike Mark 3 positions
    public static Pose spikeMark3PosPre = new Pose(40.6+2.4 , 35.6, Math.toRadians(180));
    public static Pose spikeMark3PosOuter = new Pose(25.6-5.6 , 35.6, Math.toRadians(180));

    public static Pose outOfTheWayPos = new Pose(45.0, 128.0, Math.toRadians(180));

    // Path chains
    public static PathChain goalToScore;
    public static PathChain scoreToSpikeMark1;
    public static PathChain spikeMark1ToEnd;
    public static PathChain spikeMark1EndToScore;
    public static PathChain scoreToSpikeMark2;
    public static PathChain spikeMark2ToEnd;
    public static PathChain spikeMark2EndToScore;
    public static PathChain scoreToSpikeMark3;
    public static PathChain spikeMark3ToEnd;
    public static PathChain spikeMark3EndToScore;
    public static PathChain scoreToOutOfTheWay;
    public static PathChain farStartToPark;

    public static void buildTrajectories(Follower follower, boolean isRedAlliance) {
        // Set Limelight AprilTag ID based on alliance
        if (isRedAlliance) {
            org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.ID = 20;  // Red alliance tag
            buildRedTrajectories(follower);
        } else {
            org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.ID = 24;  // Blue alliance tag
            buildBlueTrajectories(follower);
        }
    }

    private static void buildBlueTrajectories(Follower follower) {
        // Goal start directly to score
        goalToScore = follower.pathBuilder()
                .addPath(new BezierLine(goalStartPos, scorePos))
                .setLinearHeadingInterpolation(goalStartPos.getHeading(), scorePos.getHeading())
                .build();

        // Score to spike mark 1 pre-position
        scoreToSpikeMark1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePos,
                        new Pose(spikeMark1PosPre.getX() + 25, spikeMark1PosPre.getY(), spikeMark1PosPre.getHeading()),
                        spikeMark1PosPre
                ))
                .setLinearHeadingInterpolation(scorePos.getHeading(), spikeMark1PosPre.getHeading())
                .build();

        // Spike mark 1 pre to outer (end position)
        spikeMark1ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark1PosPre, spikeMark1PosOuter))
                .setLinearHeadingInterpolation(spikeMark1PosPre.getHeading(), spikeMark1PosOuter.getHeading())
                .build();

        // Spike mark 1 end back to score
        spikeMark1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark1PosOuter, scorePos))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.getHeading(), scorePos.getHeading())
                .build();

        // Score to spike mark 2 pre-position
        scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePos,
                        new Pose(spikeMark2PosPre.getX() + 25, spikeMark2PosPre.getY(), spikeMark2PosPre.getHeading()),
                        spikeMark2PosPre
                ))
                .setLinearHeadingInterpolation(scorePos.getHeading(), spikeMark2PosPre.getHeading())
                .build();

        // Spike mark 2 pre to outer (end position)
        spikeMark2ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark2PosPre, spikeMark2PosOuter))
                .setLinearHeadingInterpolation(spikeMark2PosPre.getHeading(), spikeMark2PosOuter.getHeading())
                .build();

        // Spike mark 2 end back to score
        spikeMark2EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark2PosOuter, scorePos))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.getHeading(), scorePos.getHeading())
                .build();

        // Score to spike mark 3 pre-position
        scoreToSpikeMark3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePos, spikeMark3PosPre))
                .setLinearHeadingInterpolation(scorePos.getHeading(), spikeMark3PosPre.getHeading())
                .build();

        // Spike mark 3 pre to outer (end position)
        spikeMark3ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark3PosPre, spikeMark3PosOuter))
                .setLinearHeadingInterpolation(spikeMark3PosPre.getHeading(), spikeMark3PosOuter.getHeading())
                .build();

        // Spike mark 3 end back to score
        spikeMark3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark3PosOuter, scorePos))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.getHeading(), scorePos.getHeading())
                .build();

        // Score to out of the way
        scoreToOutOfTheWay = follower.pathBuilder()
                .addPath(new BezierLine(scorePos, outOfTheWayPos))
                .setLinearHeadingInterpolation(scorePos.getHeading(), outOfTheWayPos.getHeading())
                .build();

        // Far start to park
        farStartToPark = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farStartPos,
                        new Pose(42.0, 15.0, Math.toRadians(180)),
                        farParkPos
                ))
                .setLinearHeadingInterpolation(farStartPos.getHeading(), farParkPos.getHeading())
                .build();
    }

    private static void buildRedTrajectories(Follower follower) {
        // Goal start directly to score (mirrored)
        goalToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(goalStartPos),
                        mirrorPose(scorePos)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(goalStartPos).getHeading(),
                        mirrorPose(scorePos).getHeading()
                )
                .build();

        // Score to spike mark 1 (mirrored)
        scoreToSpikeMark1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        mirrorPose(scorePos),
                        new Pose(mirrorX(spikeMark1PosPre.getX() + 30), spikeMark1PosPre.getY(), mirrorHeading(spikeMark1PosPre.getHeading())),
                        mirrorPose(spikeMark1PosPre)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(scorePos).getHeading(),
                        mirrorPose(spikeMark1PosPre).getHeading()
                )
                .build();

        // Spike mark 1 to end (mirrored)
        spikeMark1ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(spikeMark1PosPre),
                        mirrorPose(spikeMark1PosOuter)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(spikeMark1PosPre).getHeading(),
                        mirrorPose(spikeMark1PosOuter).getHeading()
                )
                .build();

        // Spike mark 1 end to score (mirrored)
        spikeMark1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(spikeMark1PosOuter),
                        mirrorPose(scorePos)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(spikeMark1PosOuter).getHeading(),
                        mirrorPose(scorePos).getHeading()
                )
                .build();

        // Score to spike mark 2 (mirrored)
        scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        mirrorPose(scorePos),
                        new Pose(mirrorX(spikeMark2PosPre.getX() + 25), spikeMark2PosPre.getY(), mirrorHeading(spikeMark2PosPre.getHeading())),
                        mirrorPose(spikeMark2PosPre)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(scorePos).getHeading(),
                        mirrorPose(spikeMark2PosPre).getHeading()
                )
                .build();

        // Spike mark 2 to end (mirrored)
        spikeMark2ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(spikeMark2PosPre),
                        mirrorPose(spikeMark2PosOuter)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(spikeMark2PosPre).getHeading(),
                        mirrorPose(spikeMark2PosOuter).getHeading()
                )
                .build();

        // Spike mark 2 end to score (mirrored)
        spikeMark2EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(spikeMark2PosOuter),
                        mirrorPose(scorePos)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(spikeMark2PosOuter).getHeading(),
                        mirrorPose(scorePos).getHeading()
                )
                .build();

        // Score to spike mark 3 (mirrored)
        scoreToSpikeMark3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(scorePos),
                        mirrorPose(spikeMark3PosPre)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(scorePos).getHeading(),
                        mirrorPose(spikeMark3PosPre).getHeading()
                )
                .build();

        // Spike mark 3 to end (mirrored)
        spikeMark3ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(spikeMark3PosPre),
                        mirrorPose(spikeMark3PosOuter)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(spikeMark3PosPre).getHeading(),
                        mirrorPose(spikeMark3PosOuter).getHeading()
                )
                .build();

        // Spike mark 3 end to score (mirrored)
        spikeMark3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(spikeMark3PosOuter),
                        mirrorPose(scorePos)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(spikeMark3PosOuter).getHeading(),
                        mirrorPose(scorePos).getHeading()
                )
                .build();

        // Score to out of the way (mirrored)
        scoreToOutOfTheWay = follower.pathBuilder()
                .addPath(new BezierLine(
                        mirrorPose(scorePos),
                        mirrorPose(outOfTheWayPos)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(scorePos).getHeading(),
                        mirrorPose(outOfTheWayPos).getHeading()
                )
                .build();

        // Far start to park (mirrored)
        farStartToPark = follower.pathBuilder()
                .addPath(new BezierCurve(
                        mirrorPose(farStartPos),
                        new Pose(mirrorX(42.0), 15.0, mirrorHeading(Math.toRadians(180))),
                        mirrorPose(farParkPos)
                ))
                .setLinearHeadingInterpolation(
                        mirrorPose(farStartPos).getHeading(),
                        mirrorPose(farParkPos).getHeading()
                )
                .build();
    }

    // Helper methods for mirroring
    private static Pose mirrorPose(Pose pose) {
        return new Pose(
                144 - pose.getX(),
                pose.getY(),
                Math.PI - pose.getHeading()
        );
    }

    private static double mirrorX(double x) {
        return 144 - x;
    }

    private static double mirrorHeading(double heading) {
        return Math.PI - heading;
    }
}