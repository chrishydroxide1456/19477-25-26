package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.onbotjava.OnBotJavaManager.initialize;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmodes.TrajectoryFactory;

@Deprecated
public class AutoRoutines {

    private final Routines routines;
    private final Intake intake;


    public AutoRoutines(Routines routines, Intake intake) {
        this.routines = routines;
        this.intake = intake;
    }

    /**
     * 3-ball autonomous: Score preload only
     */
    Command mypath = new LambdaCommand()
            .setStart(() -> follower.followPath(TrajectoryFactory.goalToScore))
            .setIsDone(() -> follower.isBusy());

    public Command threeBallAuto() {
        return new SequentialGroup(
                // Drive from goal start directly to score position
                new ParallelRaceGroup(
                        mypath,
                        intake.keeping  // Keep intake holding while driving
                ),

                // Align and shoot
                routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto()
        );
    }


    /**
     * 6-ball autonomous: Score preload + first row of samples
     */
    public Command sixBallAuto() {
        return new SequentialGroup(
                //region First volley (preload)
                // Drive from goal start directly to score position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.goalToScore, true),
                        intake.keeping
                ),

                // Align and shoot first volley
                routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                //region Second volley (spike mark 1)
                // Drive to spike mark 1 and start intaking
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                        routines.inSequence()  // Turn on intake
                ),

                // Drive through all three balls to end position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1ToEnd, true),
                        intake.on  // Keep intake running
                ),

                // Drive back to score with intake keeping
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1EndToScore, true),
                        intake.keeping
                ),

                // Align and shoot second volley
                routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                // Park
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
                        intake.keeping
                ),

                routines.stopinSequence()
        );
    }

    public Command farnineBallAuto() {
        return new SequentialGroup(
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.farStartToScore, true), //far start pos to far shooting pos
                        intake.keeping
                ),
                routines.fullOuttakeSequenceAuto(),
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.farScoreToHuman, true),
                        routines.inSequence()
                ),
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.HumanCollect, true),
                        intake.on
                ),
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.HumanTofarScore, true),
                        intake.keeping
                ),
                routines.fullOuttakeSequenceAuto(),
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.farscoreToSpikeMark1, true),
                        routines.inSequence()
                ),
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1ToEnd, true),
                        intake.on
                ),
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1EndTofarscore, true),
                        intake.keeping
                ),
                routines.fullOuttakeSequenceAuto(),
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.farscoreToPark, true),
                        intake.keeping
                ),
                routines.stopinSequence()

        );
    }

    /**
     * 9-ball autonomous: Score preload + two rows of samples
     */
    public Command nineBallAuto() {
        return new SequentialGroup(
                //region First volley (preload)
                // Drive from goal start directly to score position

                new ParallelRaceGroup(
                        mypath,
                        intake.keeping
                ),

                // Align and shoot first volley
                //routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                //region Second volley (spike mark 1)
                // Drive to spike mark 1 and start intaking
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                        routines.inSequence()
                ),

                // Drive through all three balls to end position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1ToEnd, true),
                        intake.on
                ),

                // Drive back to score with intake keeping
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1EndToScore, true),
                        intake.keeping
                ),

                // Align and shoot second volley
                //routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                //region Third volley (spike mark 2)
                // Drive to spike mark 2 and start intaking
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                        routines.inSequence()
                ),

                // Drive through all three balls to end position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark2ToEnd, true),
                        intake.on
                ),

                // Drive back to score with intake keeping
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark2EndToScore, true),
                        intake.keeping
                ),

                // Align and shoot third volley
                //routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                // Park
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
                        intake.keeping
                ),

                routines.stopinSequence()
        );
    }

    /**
     * 12-ball autonomous: Score preload + all three rows of samples
     */
    public Command twelveBallAuto() {
        return new SequentialGroup(
                //region First volley (preload)
                // Drive from goal start directly to score position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.goalToScore, true),
                        intake.keeping
                ),

                // Align and shoot first volley
                routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                //region Second volley (spike mark 1)
                // Drive to spike mark 1 and start intaking
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                        routines.inSequence()
                ),

                // Drive through all three balls to end position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1ToEnd, true),
                        intake.on
                ),

                // Drive back to score with intake keeping
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark1EndToScore, true),
                        intake.keeping
                ),

                // Align and shoot second volley
                routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                //region Third volley (spike mark 2)
                // Drive to spike mark 2 and start intaking
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                        routines.inSequence()
                ),

                // Drive through all three balls to end position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark2ToEnd, true),
                        intake.on
                ),

                // Drive back to score with intake keeping
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark2EndToScore, true),
                        intake.keeping
                ),

                // Align and shoot third volley
                routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                //region Fourth volley (spike mark 3)
                // Drive to spike mark 3 and start intaking
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToSpikeMark3, true),
                        routines.inSequence()
                ),

                // Drive through all three balls to end position
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark3ToEnd, true),
                        intake.on
                ),

                // Drive back to score with intake keeping
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.spikeMark3EndToScore, true),
                        intake.keeping
                ),

                // Align and shoot fourth volley
                routines.autoAlignOnly(),
                routines.fullOuttakeSequenceAuto(),
                //endregion

                // Park
                new ParallelRaceGroup(
                        new FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
                        intake.keeping
                ),

                routines.stopinSequence()
        );
    }

    /**
     * Simple park autonomous from far start position
     */
    public Command farParkAuto() {
        return new SequentialGroup(
                new FollowPath(TrajectoryFactory.farStartToPark, true)
        );
    }
}