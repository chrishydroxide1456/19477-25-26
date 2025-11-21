package org.firstinspires.ftc.teamcode.commandbase;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;

/**
 * Command routines that combine multiple subsystem actions
 * Following the pattern from AtomicRobotics Decode repository
 */
public class Routines {

    private static final Intake intake = Intake.INSTANCE;
    private static final Outtake outtake = Outtake.INSTANCE;
    private static final LL limelight = LL.INSTANCE;
    private static final Drive drive = Drive.INSTANCE;

    /**
     * Routine to intake a game element
     * Starts intake and waits briefly
     */
    public static Command intakeSequence() {
        return new SequentialGroup(
                intake.start,
                new Delay(0.5),
                intake.slowOut
        );
    }

    /**
     * Routine to score a game element
     * Spins up outtake, opens gate, then stops
     */
    public static Command scoreSequence() {
        return new SequentialGroup(
                outtake.on,
                new Delay(1.0),  // Wait for outtake to spin up
                outtake.open,
                new Delay(0.5),  // Wait for element to exit
                outtake.close,
                outtake.off
        );
    }

    /**
     * Routine to adjust heading using Limelight
     * Updates targeting and waits briefly
     */
    public static Command limelightAdjust() {
        return new InstantCommand(() -> limelight.adjust());
    }

    /**
     * Stop all mechanisms
     */
    public static Command stopAll() {
        return new ParallelGroup(
                intake.stop,
                outtake.off,
                new InstantCommand(() -> drive.stop())
        );
    }

    /**
     * Reverse intake to eject game element
     */
    public static Command ejectSequence() {
        return new SequentialGroup(
                intake.reverse,
                new Delay(0.5),
                intake.stop
        );
    }

}
