package org.firstinspires.ftc.teamcode.commandbase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.*;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.*;

public class Routines {
    private final Intake intake;
    private final Outtake outtake;
    private final LL ll;
    private final Drive drive;

    // Timing constants
    private static final long FLYWHEEL_SPINUP_MS = 750;
    private static final long INTAKE_REVERSE_START_MS = 500;
    private static final long GATE_OPEN_MS = 750;
    private static final long INTAKE_FORWARD_START_MS = 1250;
    private static final long SEQUENCE_DURATION_MS = 3250;
    private static final double HEADING_TOLERANCE = 0.7;
    private static final long ALIGN_TIMEOUT_MS = 1500;

    public Routines(Intake intake, Outtake outtake, LL ll, Drive drive) {
        this.intake = intake;
        this.outtake = outtake;
        this.ll = ll;
        this.drive = drive;
    }

    public Command autoAlignOnly() {
        return new Command() {
            private long startTime;

            @Override
            public void start() {
                drive.startAutoAlign();
                startTime = System.currentTimeMillis();
            }

            @Override
            public boolean isDone() {
                long elapsed = System.currentTimeMillis() - startTime;
                boolean aligned = Math.abs(LL.headingAdjust) <= HEADING_TOLERANCE;
                boolean timeout = elapsed > ALIGN_TIMEOUT_MS;

                return (aligned && LL.tagVisible) || timeout;
            }

            @Override
            public void stop(boolean interrupted) {
                drive.stopAutoAlign();
            }
        };
    }

    public Command testoutSequence() {
        return new Command() {
            private long sequenceStart = 0;
            private boolean gateOpened = false;
            private boolean intakeReversed = false;
            private boolean intakeForwarded = false;

            @Override
            public void start() {
                if (!LL.tagVisible) {
                    return;
                }

                // Stop spin servos immediately
                outtake.spinServo1.setPower(0);
                outtake.spinServo2.setPower(0);

                Outtake.shooting = true;
                sequenceStart = System.currentTimeMillis();
                gateOpened = false;
                intakeReversed = false;
                intakeForwarded = false;
            }

            @Override
            public void update() {
                long elapsed = System.currentTimeMillis() - sequenceStart;

                // Reverse intake at 500ms AND spin servos backward
                if (elapsed > INTAKE_REVERSE_START_MS && !intakeReversed) {
                    Intake.INSTANCE.revmoving.schedule();
                    outtake.spinServo1.setPower(-0.5);
                    outtake.spinServo2.setPower(-0.5);
                    intakeReversed = true;
                }

                // Open gate at 750ms
                if (elapsed > GATE_OPEN_MS && !gateOpened) {
                    Outtake.INSTANCE.open.schedule();
                    gateOpened = true;
                }

                // Start intake forward at 1250ms AND spin servos forward
                if (elapsed > INTAKE_FORWARD_START_MS && !intakeForwarded) {
                    Intake.INSTANCE.onmoving.schedule();
                    outtake.spinServo1.setPower(0.7);
                    outtake.spinServo2.setPower(0.7);
                    intakeForwarded = true;
                }

                // Complete sequence at 3250ms
                if (elapsed > SEQUENCE_DURATION_MS) {
                    Outtake.INSTANCE.close.schedule();
                    Intake.INSTANCE.off.schedule();
                    outtake.spinServo1.setPower(0);
                    outtake.spinServo2.setPower(0);
                    Outtake.shooting = false;
                }
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() - sequenceStart > SEQUENCE_DURATION_MS;
            }

            @Override
            public void stop(boolean interrupted) {
                Outtake.shooting = false;
                Outtake.INSTANCE.close.schedule();
                Intake.INSTANCE.off.schedule();
                outtake.spinServo1.setPower(0);
                outtake.spinServo2.setPower(0);
            }
        };
    }

    public Command fullOuttakeSequence() {
        return testoutSequence();
    }

    public Command inSequence() {
        return new Command() {
            private long startTime = 0;
            private boolean servosStarted = false;

            @Override
            public void start() {
                drive.setMulti(0.58);
                intake.on.schedule();
                startTime = System.currentTimeMillis();
                servosStarted = false;
            }

            @Override
            public void update() {
                long elapsed = System.currentTimeMillis() - startTime;

                // Start spin servos after 500ms delay
                if (elapsed > 500 && !servosStarted) {
                    outtake.spinServo1.setPower(-0.5);
                    outtake.spinServo2.setPower(-0.5);
                    servosStarted = true;
                }
            }

            @Override
            public boolean isDone() {
                return false;  // Changed to false so update() keeps running
            }
        };
    }

    public Command stopinSequence() {
        return new Command() {
            @Override
            public void start() {
                drive.setMulti(1.0);
                intake.keeping.schedule();
                // Stop spin servos
                outtake.spinServo1.setPower(-0.2);
                outtake.spinServo2.setPower(-0.2);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        };
    }
}