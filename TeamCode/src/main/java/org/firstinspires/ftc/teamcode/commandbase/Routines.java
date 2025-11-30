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

    private final Intake intake;
    private final Outtake outtake;
    private final LL ll;
    private final Drive drive;

    public Routines(Intake intake, Outtake outtake, LL ll, Drive drive) {
        this.intake = intake;
        this.outtake = outtake;
        this.ll = ll;
        this.drive = drive;
    }

    // Auto-align + shoot sequence
    public Command testoutSequence() {
        return new ParallelGroup(
                robotAdjust,

                new SequentialGroup(
                        new Delay(0.75),
                        Outtake.INSTANCE.open
                ),

                new SequentialGroup(
                        new Delay(1.25),
                        Intake.INSTANCE.onmoving
                ),

                new SequentialGroup(
                        new Delay(3.25),
                        new ParallelGroup(
                                Intake.INSTANCE.off,
                                Outtake.INSTANCE.close
                        )
                )
        );
    }

    public Command inSequence() {
        drive.setMulti(0.58);
        return new ParallelGroup(
                Intake.INSTANCE.on,
                // you can add Outtake reverse here later if needed
                Intake.INSTANCE.on // placeholder; adjust as desired
        );
    }

    public Command stopinSequence() {
        drive.setMulti(1.0);
        return new ParallelGroup(
                Intake.INSTANCE.off
        );
    }

    public final Command robotAdjust = new Command() {
        @Override
        public void start() {
            drive.startAutoAlign();
        }

        @Override
        public void update() {
            // LL.adjust and outtake.periodic are called from TeleOp onUpdate
        }

        @Override
        public boolean isDone() { return false; }

        @Override
        public void stop(boolean interrupted) {
            drive.stopAutoAlign();
        }
    };
}
