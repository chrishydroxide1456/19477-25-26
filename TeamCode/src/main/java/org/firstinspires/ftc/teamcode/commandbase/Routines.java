package org.firstinspires.ftc.teamcode.commandbase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

public class Routines {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;

    public Routines(Intake intake, Outtake outtake, LL ll, Drive drive) {
        this.intake = intake;
        this.outtake = outtake;
        this.ll = ll;
        this.drive = drive;
    }

    /**
     * Test Outtake sequence: spins flywheel, opens gate, runs intake, then stops everything
     */
    public Command testoutSequence() {
        return new ParallelGroup(
                // Spin flywheel in background
                new Command() {
                    @Override
                    public void start() { LL.targetVel = 933; }
                    @Override
                    public void update() {}
                    @Override
                    public boolean isDone() { return false; }
                },

                // Open gate after 0.75s
                new SequentialGroup(
                        new Delay(0.75),
                        Outtake.INSTANCE.open
                ),

                // Start intake after 1.25s
                new SequentialGroup(
                        new Delay(1.25),
                        Intake.INSTANCE.onmoving
                ),

                // Stop everything after 3.25s
                new SequentialGroup(
                        new Delay(3.25),
                        new ParallelGroup(
                                new Command() {
                                    @Override
                                    public void start() { LL.targetVel = 0.0; }
                                    @Override public boolean isDone() { return true; }
                                },
                                Outtake.INSTANCE.close,
                                Intake.INSTANCE.off
                        )
                )
        );
    }

    public Command inSequence() {
        drive.setMulti(1.0);
        return new ParallelGroup(
                Intake.INSTANCE.on,
                Outtake.INSTANCE.reverse
        );
    }

    public Command stopinSequence() {
        drive.setMulti(0.58);
        return new ParallelGroup(
                Intake.INSTANCE.keeping,
                Outtake.INSTANCE.testoff
        );
    }

    public Command stopoutSequence() {
        return new ParallelGroup(
                Outtake.INSTANCE.off
        );
    }

    public Command outtaketest1() {
        return new ParallelGroup(
                Outtake.INSTANCE.teston2000,
                Outtake.INSTANCE.open
        );
    }

    public Command outtaketest2() {
        return new ParallelGroup(
                Outtake.INSTANCE.teston4000
        );
    }
}
