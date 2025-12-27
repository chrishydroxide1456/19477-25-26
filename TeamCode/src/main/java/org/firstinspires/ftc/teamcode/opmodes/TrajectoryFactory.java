package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class TrajectoryFactory {
    //TODO: Make it so all of the intaking spike mark paths (spike mark to end) are at a 0.58 dt multiplier

    // Poses - all positions on the field
    public static Pose goalStartPos = new Pose(26.0, 130.0, Math.toRadians(-40));
    public static Pose farStartPos = new Pose(56.0, 8.0, Math.toRadians(270));
    public static Pose farParkPos = new Pose(37.0, 13.0, Math.toRadians(180));
    //public static Pose scorePos = new Pose(57, 88, Math.toRadians(-48.5));
    public static Pose scorePos = new Pose(60, 90, Math.toRadians(-49.5));

    public static Pose gatePos = new Pose(20.8, 74.0, Math.toRadians(180));

    // Spike Mark 1 positions
    public static Pose spikeMark1PosPre = new Pose(41.6 + 6, 83.6, Math.toRadians(180));
    public static Pose spikeMark1PosOuter = new Pose(26.6 - 7, 83.6, Math.toRadians(180));

    // Spike Mark 2 positions
    public static Pose spikeMark2PosPre = new Pose(41.6 + 6, 60.0, Math.toRadians(180));
    public static Pose spikeMark2PosOuter = new Pose(26.6 - 9.2, 60.0, Math.toRadians(180));

    // Spike Mark 3 positions
    public static Pose spikeMark3PosPre = new Pose(42.6 + 6, 35, Math.toRadians(180));
    public static Pose spikeMark3PosOuter = new Pose(25.6 -13.5, 35, Math.toRadians(180));

    // Blue-adjusted Spike Mark positions (Y shifted -6)
    public static Pose spikemark1prepos = new Pose(spikeMark1PosPre.getX(), spikeMark1PosPre.getY() + 10, spikeMark1PosPre.getHeading());
    public static Pose spikemark1outer = new Pose(spikeMark1PosOuter.getX()+2.2, spikeMark1PosOuter.getY() +10, spikeMark1PosOuter.getHeading());

    public static Pose spikemark2prepos = new Pose(spikeMark2PosPre.getX(), spikeMark2PosPre.getY() +11, spikeMark2PosPre.getHeading());
    public static Pose spikemark2outer = new Pose(spikeMark2PosOuter.getX()+3, spikeMark2PosOuter.getY() +11, spikeMark2PosOuter.getHeading());

    public static Pose spikemark3prepos = new Pose(spikeMark3PosPre.getX(), spikeMark3PosPre.getY() + 12, spikeMark3PosPre.getHeading());
    public static Pose spikemark3outer = new Pose(spikeMark3PosOuter.getX()+3, spikeMark3PosOuter.getY() + 12, spikeMark3PosOuter.getHeading());

    // Blue-adjusted Gate position (Y shifted -6)
    public static Pose gatePosBlue = new Pose(gatePos.getX(), gatePos.getY() + 11, gatePos.getHeading());

    public static Pose scorePosFinal = new Pose(60, 90, Math.toRadians(-49.5));

    public static Pose outOfTheWayPos = new Pose(27.0, 70.0, Math.toRadians(180));

    public static Pose farscorePos = new Pose(58.9, 18.0, Math.toRadians(115.5-180));

    public static Pose HumanArtifacts = new Pose(8.54, 23, Math.toRadians(270));

    public static Pose HumanZone = new Pose(6, 8.54, Math.toRadians(270));

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
    public static PathChain spikeMark1EndToGate;
    public static PathChain gateToScore;

    public static PathChain farStartToScore;
    public static PathChain farScoreToHuman;
    public static PathChain HumanCollect;
    public static PathChain HumanTofarScore;
    public static PathChain farscoreToSpikeMark1;
    public static PathChain spikeMark1EndTofarscore;
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
                        spikemark1prepos
                ))
                .setLinearHeadingInterpolation(scorePos.getHeading(), spikemark1prepos.getHeading())
                .build();

        // Spike mark 1 pre to outer (end position)
        spikeMark1ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(spikemark1prepos, spikemark1outer))
                .setLinearHeadingInterpolation(spikemark1prepos.getHeading(), spikemark1outer.getHeading())
                .build();

        // Spike mark 1 end back to score
        spikeMark1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(spikemark1outer, scorePos))
                .setLinearHeadingInterpolation(spikemark1outer.getHeading(), scorePos.getHeading())
                .build();

        spikeMark1EndToGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        spikemark1outer,
                        new Pose(gatePosBlue.getX() + 25, gatePosBlue.getY(), gatePosBlue.getHeading()),
                        gatePosBlue
                ))
                .setLinearHeadingInterpolation(spikemark1outer.getHeading(), gatePosBlue.getHeading())
                .build();

        gateToScore = follower.pathBuilder()
                .addPath(new BezierLine(gatePosBlue, scorePos))
                .setLinearHeadingInterpolation(gatePosBlue.getHeading(), scorePos.getHeading())
                .build();

        // Score to spike mark 2 pre-position
        scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePos,
                        new Pose(spikemark2prepos.getX() + 25, spikemark2prepos.getY(), spikemark2prepos.getHeading()),
                        spikemark2prepos
                ))
                .setLinearHeadingInterpolation(scorePos.getHeading(), spikemark2prepos.getHeading())
                .build();

        // Spike mark 2 pre to outer (end position)
        spikeMark2ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(spikemark2prepos, spikemark2outer))
                .setLinearHeadingInterpolation(spikemark2prepos.getHeading(), spikemark2outer.getHeading())
                .build();

        // Spike mark 2 end back to score
        spikeMark2EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(spikemark2outer, scorePosFinal))
                .setLinearHeadingInterpolation(spikemark2outer.getHeading(), scorePosFinal.getHeading())
                .build();

        // Score to spike mark 3 pre-position
        scoreToSpikeMark3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePosFinal, spikemark3prepos))
                .setLinearHeadingInterpolation(scorePosFinal.getHeading(), spikemark3prepos.getHeading())
                .build();

        // Spike mark 3 pre to outer (end position)
        spikeMark3ToEnd = follower.pathBuilder()
                .addPath(new BezierLine(spikemark3prepos, spikemark3outer))
                .setLinearHeadingInterpolation(spikemark3prepos.getHeading(), spikemark3outer.getHeading())
                .build();

        // Spike mark 3 end back to score
        spikeMark3EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(spikemark3outer, scorePosFinal))
                .setLinearHeadingInterpolation(spikemark3outer.getHeading(), scorePosFinal.getHeading())
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

        farscoreToSpikeMark1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        farscorePos,
                        new Pose(spikemark1prepos.getX() + 30, spikemark1prepos.getY(), spikemark1prepos.getHeading()),
                        spikemark1prepos,
                        spikemark1prepos
                ))
                .setLinearHeadingInterpolation(farscorePos.getHeading(), spikemark1prepos.getHeading())
                .build();

        farScoreToHuman = follower.pathBuilder()
                .addPath(new BezierCurve(farscorePos, HumanArtifacts))
                .setLinearHeadingInterpolation(farscorePos.getHeading(), HumanArtifacts.getHeading())
                .build();

        HumanCollect = follower.pathBuilder()
                .addPath(new BezierCurve(HumanArtifacts, HumanZone))
                .setLinearHeadingInterpolation(HumanArtifacts.getHeading(), HumanZone.getHeading()).setVelocityConstraint(20.0)
                .build();

        HumanTofarScore = follower.pathBuilder()
                .addPath(new BezierCurve(HumanZone, farscorePos))
                .setLinearHeadingInterpolation(HumanZone.getHeading(), farscorePos.getHeading())
                .build();

        spikeMark1EndTofarscore = follower.pathBuilder()
                .addPath(new BezierCurve(spikemark1outer, farscorePos))
                .setLinearHeadingInterpolation(spikemark1outer.getHeading(), farscorePos.getHeading())
                .build();

        farscoreToPark = follower.pathBuilder()
                .addPath(new BezierCurve(farscorePos, farParkPos))
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

        spikeMark1EndTofarscore = follower.pathBuilder()
                .addPath(new BezierCurve(spikeMark3PosOuter.mirror(), farscorePos.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.mirror().getHeading(), farscorePos.mirror().getHeading())
                .build();

        farscoreToSpikeMark1 = follower.pathBuilder() //come back here
                .addPath(new BezierCurve(
                        farscorePos.mirror(),
                        new Pose(spikeMark3PosPre.getX() + 30, spikeMark3PosPre.getY(), spikeMark3PosPre.getHeading()).mirror(),
                        spikeMark3PosPre.mirror()
                ))
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

        spikeMark1EndToGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        spikeMark1PosOuter.mirror(),
                        new Pose(gatePos.getX() + 25, gatePos.getY(), gatePos.getHeading()).mirror(),
                        gatePos.mirror()
                ))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.mirror().getHeading(), gatePos.mirror().getHeading())
                .build();

        gateToScore = follower.pathBuilder()
                .addPath(new BezierLine(
                        gatePos.mirror(),
                        scorePos.mirror()
                ))
                .setLinearHeadingInterpolation(
                        gatePos.mirror().getHeading(),
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
                .addPath(new BezierCurve(
                        scorePosFinal.mirror(),
                        new Pose(spikeMark3PosPre.getX() + 25, spikeMark3PosPre.getY(), spikeMark3PosPre.getHeading()).mirror(),
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
