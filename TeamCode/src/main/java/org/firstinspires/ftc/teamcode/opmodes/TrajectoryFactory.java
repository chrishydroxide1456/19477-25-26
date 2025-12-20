package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class TrajectoryFactory {
    //TODO: Make it so all of the intaking spike mark paths (spike mark to end) are at a 0.58 dt multiplier

    // Poses - all positions on the field
    public static Pose goalStartPos = new Pose(26.0, 130.0, Math.toRadians(-40));
    public static Pose farStartPos = new Pose(56.0, 8.0, Math.toRadians(180));
    public static Pose farParkPos = new Pose(36.0, 8.0, Math.toRadians(180));
    //public static Pose scorePos = new Pose(45, 105, Math.toRadians(-47));
    public static Pose scorePos = new Pose(57, 88, Math.toRadians(-48.5));

    // Spike Mark 1 positions
    public static Pose spikeMark1PosPre = new Pose(41.6 + 6, 83.6, Math.toRadians(180));
    public static Pose spikeMark1PosOuter = new Pose(26.6 - 6.3, 83.6, Math.toRadians(180));

    // Spike Mark 2 positions
    public static Pose spikeMark2PosPre = new Pose(41.6 + 6, 60.0, Math.toRadians(180));
    public static Pose spikeMark2PosOuter = new Pose(26.6 - 7.5, 60.0, Math.toRadians(180));

    // Spike Mark 3 positions
    public static Pose spikeMark3PosPre = new Pose(42.6 + 10, 35, Math.toRadians(180));
    public static Pose spikeMark3PosOuter = new Pose(25.6 -11.5, 35, Math.toRadians(180));

    public static Pose scorePosFinal = new Pose(60, 90, Math.toRadians(-49.5));

    public static Pose outOfTheWayPos = new Pose(25.0, 70.0, Math.toRadians(180));

    public static Pose farscorePos = new Pose(58.9, 20.1, Math.toRadians(115));

    public static Pose HumanArtifacts = new Pose(13.5, 17.3, Math.toRadians(16));

    public static Pose HumanZone = new Pose(11.8, 12.3, Math.toRadians(16));

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

    public static PathChain farStartToScore;
    public static PathChain farScoreToHuman;
    public static PathChain HumanCollect;
    public static PathChain HumanTofarScore;
    public static PathChain farscoreToSpikeMark3;
    public static PathChain spikeMark3EndTofarscore;
    public static PathChain farscoreToPark;

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
                .addPath(new BezierLine(spikeMark2PosOuter, scorePosFinal))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.getHeading(), scorePosFinal.getHeading())
                .build();

        // Score to spike mark 3 pre-position
        scoreToSpikeMark3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePosFinal, spikeMark3PosPre))
                .setLinearHeadingInterpolation(scorePosFinal.getHeading(), spikeMark3PosPre.getHeading())
                .build();

        // Spike mark 3 pre to outer (end position)
        spikeMark3ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark3PosPre, spikeMark3PosOuter))
                .setLinearHeadingInterpolation(spikeMark3PosPre.getHeading(), spikeMark3PosOuter.getHeading())
                .build();

        // Spike mark 3 end back to score
        spikeMark3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(spikeMark3PosOuter, scorePosFinal))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.getHeading(), scorePosFinal.getHeading())
                .build();

        // Score to out of the way
        scoreToOutOfTheWay = follower.pathBuilder()
                .addPath(new BezierLine(scorePosFinal, outOfTheWayPos))
                .setLinearHeadingInterpolation(scorePosFinal.getHeading(), outOfTheWayPos.getHeading())
                .build();
        farStartToScore = follower.pathBuilder()
                .addPath(new BezierCurve(farStartPos, farscorePos))
                .setLinearHeadingInterpolation(farStartPos.getHeading(), farscorePos.getHeading())
                .build();
        farscoreToSpikeMark3 = follower.pathBuilder()
                .addPath(new BezierCurve(farscorePos, new Pose(spikeMark3PosPre.getX() + 25, spikeMark3PosPre.getY(), spikeMark3PosPre.getHeading()),
                        spikeMark3PosPre, spikeMark3PosPre))
                .setLinearHeadingInterpolation(farscorePos.getHeading(), spikeMark1PosPre.getHeading())
                .build();

        farScoreToHuman = follower.pathBuilder()
                .addPath(new BezierCurve(farscorePos, HumanArtifacts))
                .setLinearHeadingInterpolation(farscorePos.getHeading(), HumanArtifacts.getHeading())
                .build();

        HumanCollect = follower.pathBuilder()
                .addPath(new BezierCurve(HumanArtifacts, HumanZone))
                .setLinearHeadingInterpolation(HumanArtifacts.getHeading(), HumanZone.getHeading())
                .build();

        HumanTofarScore = follower.pathBuilder()
                .addPath(new BezierCurve(HumanZone, farscorePos))
                .setLinearHeadingInterpolation(HumanZone.getHeading(), farscorePos.getHeading())
                .build();
        spikeMark3EndTofarscore = follower.pathBuilder()
                .addPath(new BezierCurve(spikeMark3PosOuter, farscorePos))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.getHeading(), farscorePos.getHeading())
                .build();
        farscoreToPark = follower.pathBuilder()
                .addPath(new BezierCurve(farscorePos, new Pose(56.0, 36.0, Math.toRadians(180)), farParkPos))
                .setLinearHeadingInterpolation(farscorePos.getHeading(), farParkPos.getHeading())
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
        // Goal start directly to score (mirrored using built-in mirror() method)
        farscoreToPark = follower.pathBuilder()
                .addPath(new BezierCurve(farscorePos.mirror(), new Pose(56.0, 36.0, Math.toRadians(180)).mirror(), farParkPos.mirror()))
                .setLinearHeadingInterpolation(farscorePos.mirror().getHeading(), farParkPos.mirror().getHeading())
                .build();

        spikeMark3EndTofarscore = follower.pathBuilder()
                .addPath(new BezierCurve(spikeMark3PosOuter.mirror(), farscorePos.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.mirror().getHeading(), farscorePos.mirror().getHeading())
                .build();

        farscoreToSpikeMark3 = follower.pathBuilder() //come back here
                .addPath(new BezierCurve(farscorePos.mirror(), new Pose(spikeMark3PosPre.getX() + 30
                        , spikeMark3PosPre.getY(), spikeMark3PosPre.getHeading()).mirror(),
                        spikeMark3PosPre.mirror()))
                .setLinearHeadingInterpolation(farscorePos.mirror().getHeading(), spikeMark3PosPre.mirror().getHeading())
                .build();

        farScoreToHuman = follower.pathBuilder()
                .addPath(new BezierCurve(farscorePos.mirror(), HumanArtifacts.mirror()))
                .setLinearHeadingInterpolation(farscorePos.mirror().getHeading(), HumanArtifacts.mirror().getHeading())
                .build();

        HumanCollect = follower.pathBuilder()
                .addPath(new BezierCurve(HumanArtifacts.mirror(), HumanZone.mirror()))
                .setLinearHeadingInterpolation(HumanArtifacts.mirror().getHeading(), HumanZone.mirror().getHeading())
                .build();

        HumanTofarScore = follower.pathBuilder()
                .addPath(new BezierCurve(HumanZone.mirror(), farscorePos.mirror()))
                .setLinearHeadingInterpolation(HumanZone.mirror().getHeading(), farscorePos.mirror().getHeading())
                .build();

        farStartToScore = follower.pathBuilder()
                .addPath(new BezierCurve(farStartPos.mirror(), farscorePos.mirror()))
                .setLinearHeadingInterpolation(farStartPos.mirror().getHeading(), farscorePos.mirror().getHeading())
                .build();

        goalToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        goalStartPos.mirror(),
                        scorePos.mirror()
                ))
                .setLinearHeadingInterpolation(
                        goalStartPos.mirror().getHeading(),
                        scorePos.mirror().getHeading()
                )
                .build();

        // Score to spike mark 1 (mirrored)
        scoreToSpikeMark1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePos.mirror(),
                        new Pose(spikeMark1PosPre.getX() + 25, spikeMark1PosPre.getY(), spikeMark1PosPre.getHeading()).mirror(),
                        spikeMark1PosPre.mirror()
                ))
                .setLinearHeadingInterpolation(
                        scorePos.mirror().getHeading(),
                        spikeMark1PosPre.mirror().getHeading()
                )
                .build();

        // Spike mark 1 to end (mirrored)
        spikeMark1ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        spikeMark1PosPre.mirror(),
                        spikeMark1PosOuter.mirror()
                ))
                .setLinearHeadingInterpolation(
                        spikeMark1PosPre.mirror().getHeading(),
                        spikeMark1PosOuter.mirror().getHeading()
                )
                .build();

        // Spike mark 1 end to score (mirrored)
        spikeMark1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        spikeMark1PosOuter.mirror(),
                        scorePos.mirror()
                ))
                .setLinearHeadingInterpolation(
                        spikeMark1PosOuter.mirror().getHeading(),
                        scorePos.mirror().getHeading()
                )
                .build();

        // Score to spike mark 2 (mirrored)
        scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePos.mirror(),
                        new Pose(spikeMark2PosPre.getX() + 25, spikeMark2PosPre.getY(), spikeMark2PosPre.getHeading()).mirror(),
                        spikeMark2PosPre.mirror()
                ))
                .setLinearHeadingInterpolation(
                        scorePos.mirror().getHeading(),
                        spikeMark2PosPre.mirror().getHeading()
                )
                .build();

        // Spike mark 2 to end (mirrored)
        spikeMark2ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        spikeMark2PosPre.mirror(),
                        spikeMark2PosOuter.mirror()
                ))
                .setLinearHeadingInterpolation(
                        spikeMark2PosPre.mirror().getHeading(),
                        spikeMark2PosOuter.mirror().getHeading()
                )
                .build();

        // Spike mark 2 end to score (mirrored)
        spikeMark2EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        spikeMark2PosOuter.mirror(),
                        scorePosFinal.mirror()
                ))
                .setLinearHeadingInterpolation(
                        spikeMark2PosOuter.mirror().getHeading(),
                        scorePosFinal.mirror().getHeading()
                )
                .build();

        // Score to spike mark 3 (mirrored)
        scoreToSpikeMark3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scorePosFinal.mirror(),
                        spikeMark3PosPre.mirror()
                ))
                .setLinearHeadingInterpolation(
                        scorePosFinal.mirror().getHeading(),
                        spikeMark3PosPre.mirror().getHeading()
                )
                .build();

        // Spike mark 3 to end (mirrored)
        spikeMark3ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        spikeMark3PosPre.mirror(),
                        spikeMark3PosOuter.mirror()
                ))
                .setLinearHeadingInterpolation(
                        spikeMark3PosPre.mirror().getHeading(),
                        spikeMark3PosOuter.mirror().getHeading()
                )
                .build();

        // Spike mark 3 end to score (mirrored)
        spikeMark3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        spikeMark3PosOuter.mirror(),
                        scorePosFinal.mirror()
                ))
                .setLinearHeadingInterpolation(
                        spikeMark3PosOuter.mirror().getHeading(),
                        scorePosFinal.mirror().getHeading()
                )
                .build();

        // Score to out of the way (mirrored)
        scoreToOutOfTheWay = follower.pathBuilder()
                .addPath(new BezierLine(
                        scorePosFinal.mirror(),
                        outOfTheWayPos.mirror()
                ))
                .setLinearHeadingInterpolation(
                        scorePosFinal.mirror().getHeading(),
                        outOfTheWayPos.mirror().getHeading()
                )
                .build();

        // Far start to park (mirrored)
        farStartToPark = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farStartPos.mirror(),
                        new Pose(42.0, 15.0, Math.toRadians(180)).mirror(),
                        farParkPos.mirror()
                ))
                .setLinearHeadingInterpolation(
                        farStartPos.mirror().getHeading(),
                        farParkPos.mirror().getHeading()
                )
                .build();
    }
}