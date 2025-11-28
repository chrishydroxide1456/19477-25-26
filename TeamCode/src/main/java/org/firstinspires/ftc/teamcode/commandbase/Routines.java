package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.headingAdjust;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

public class Routines {

//    private Intake intake = Intake.INSTANCE;
//    private Outtake outtake = Outtake.INSTANCE;
//    private LL ll = LL.INSTANCE;
//    private Drive drive = Drive.INSTANCE;

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

    public Command outSequence() { //currently is a little sketch for if robot isn't stationary when starting this sequence because adjusting only runs at the beginning
        return new ParallelGroup(
                robotadjust,
                outtake.on,
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(3.0),
                                outtake.open
                        ),
                        new SequentialGroup(
                                new Delay(6.0),
                                new ParallelGroup(
                                        outtake.close,
                                        outtake.off
                                        //need a way to turn adjust off while letting it continue while moving/shooting
                                )
                        )
                )
        );
    }
    public Command robotadjust = new Command() {

        @Override
        public void start() {
            ll.adjust();
            //drive.autodrivecmd.start();
            drive.autodrive(headingAdjust);
        }

        @Override
        public boolean isDone() {
            //return headingAdjust < 0.04; //like 2.3 degrees
            return false;
        }

        @Override
        public void update() {
            ll.adjust();
            //drive.autodrivecmd.start();
            drive.autodrive(headingAdjust);
        }

    };

    public Command inSequence() {
            return new ParallelGroup(
                    intake.on,
                    outtake.reverse
            );
    }

    public Command inSequencemoving() {
        return new ParallelGroup(
                intake.onmoving,
                outtake.reverse
        );
    }

    public Command outtaketest1() {
        return new ParallelGroup(
                outtake.teston2000,
                new Delay(3.0),
                outtake.testoff
        );
    }

    public Command outtaketest2() {
        return new ParallelGroup(
                outtake.teston4000,
                new Delay(3.0),
                outtake.testoff
        );
    }

}
