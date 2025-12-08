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
    private static final long INTAKE_REVERSE_START_MS = 1300;
    private static final long INTAKE_STOP_MS = 1500;          // NEW: Stop 150ms after reversing starts
    private static final long GATE_OPEN_MS = 1300;
    private static final long INTAKE_FORWARD_START_MS = 1700;  // 250ms pause (1700 - 1450 = 250ms)
    private static final long SEQUENCE_DURATION_MS = 3250;
    private static final double HEADING_TOLERANCE = 0.7;
    private static final long ALIGN_TIMEOUT_MS = 3000;

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
            private boolean intakeStopped = false;      // NEW: Track when we stop
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
                intakeStopped = false;      // NEW
                intakeForwarded = false;
            }

            @Override
            public void update() {
                long elapsed = System.currentTimeMillis() - sequenceStart;

                // Reverse intake at 1300ms AND spin servos backward
                if (elapsed > INTAKE_REVERSE_START_MS && !intakeReversed) {
                    Intake.INSTANCE.revmoving.schedule();
                    outtake.spinServo1.setPower(-1.0);
                    outtake.spinServo2.setPower(-1.0);
                    intakeReversed = true;
                }

                // NEW: Stop everything at 1450ms (150ms of reversing)
                if (elapsed > INTAKE_STOP_MS && !intakeStopped) {
                    Intake.INSTANCE.off.schedule();
                    outtake.spinServo1.setPower(0);
                    outtake.spinServo2.setPower(0);
                    intakeStopped = true;
                }

                // Open gate at 1300ms
                if (elapsed > GATE_OPEN_MS && !gateOpened) {
                    Outtake.INSTANCE.open.schedule();
                    gateOpened = true;
                }

                // Start intake forward at 1700ms (250ms pause after stopping)
                if (elapsed > INTAKE_FORWARD_START_MS && !intakeForwarded) {
                    Intake.INSTANCE.onmoving.schedule();
                    outtake.spinServo1.setPower(0);
                    outtake.spinServo2.setPower(0);
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

                // Start spin servos after 400ms delay
                if (elapsed > 400 && !servosStarted) {
                    outtake.spinServo1.setPower(-1.0);
                    outtake.spinServo2.setPower(-1.0);
                    servosStarted = true;
                }
            }

            @Override
            public boolean isDone() {
                return false;  // Keep running
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
                outtake.spinServo1.setPower(0);
                outtake.spinServo2.setPower(0);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        };
    }

    public Command stopReverseSequence() {
        return new Command() {
            @Override
            public void start() {
                drive.setMulti(1.0);
                intake.off.schedule();
                // Stop spin servos
                outtake.spinServo1.setPower(0);
                outtake.spinServo2.setPower(0);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        };
    }
}