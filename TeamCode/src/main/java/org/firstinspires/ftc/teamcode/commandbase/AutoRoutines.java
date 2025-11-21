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
 * Autonomous command routines
 * Following the pattern from AtomicRobotics Decode repository
 */
public class AutoRoutines {

    private static final Intake intake = Intake.INSTANCE;
    private static final Outtake outtake = Outtake.INSTANCE;
    private static final LL limelight = LL.INSTANCE;
    private static final Drive drive = Drive.INSTANCE;

    /**
     * Basic autonomous routine - score preload and park
     */
    public static Command basicAutoRoutine() {
        return new SequentialGroup(
                // Initialize
                new InstantCommand(() -> System.out.println("Starting autonomous...")),
                
                // Drive forward
                new InstantCommand(() -> drive.driveManual(0.3, 0, 0)),
                new Delay(1.0),
                new InstantCommand(() -> drive.stop()),
                
                // Score preload
                Routines.scoreSequence(),
                
                // Back up
                new InstantCommand(() -> drive.driveManual(-0.3, 0, 0)),
                new Delay(0.5),
                new InstantCommand(() -> drive.stop()),
                
                // Park
                new InstantCommand(() -> drive.driveManual(0, 0.3, 0)),
                new Delay(1.0),
                new InstantCommand(() -> drive.stop()),
                
                new InstantCommand(() -> System.out.println("Autonomous complete!"))
        );
    }

    /**
     * Score preload specimen
     */
    public static Command scorePreloadRoutine() {
        return new SequentialGroup(
                // Drive to scoring position
                new InstantCommand(() -> drive.driveManual(0.3, 0, 0)),
                new Delay(1.5),
                new InstantCommand(() -> drive.stop()),
                
                // Score
                Routines.scoreSequence(),
                
                // Back away
                new InstantCommand(() -> drive.driveManual(-0.2, 0, 0)),
                new Delay(0.5),
                new InstantCommand(() -> drive.stop())
        );
    }

    /**
     * Park in observation zone
     */
    public static Command parkRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> drive.driveManual(0, 0.3, 0)),
                new Delay(1.0),
                new InstantCommand(() -> drive.stop())
        );
    }

    /**
     * Example multi-element autonomous
     */
    public static Command multiElementAuto() {
        return new SequentialGroup(
                // Score preload
                scorePreloadRoutine(),
                
                // Drive to sample
                new InstantCommand(() -> drive.driveManual(-0.3, 0, 0)),
                new Delay(1.0),
                new InstantCommand(() -> drive.stop()),
                
                // Intake sample
                Routines.intakeSequence(),
                
                // Drive back to score
                new InstantCommand(() -> drive.driveManual(0.3, 0, 0)),
                new Delay(1.0),
                new InstantCommand(() -> drive.stop()),
                
                // Score sample
                Routines.scoreSequence(),
                
                // Park
                parkRoutine()
        );
    }

}
