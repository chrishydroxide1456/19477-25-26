package org.firstinspires.ftc.teamcode.opmodes.redauto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.opmodes.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import java.util.*;

@Autonomous(name = "🔴 12-Ball Auto Red (Gate Path)", group = "Red", preselectTeleOp = "TeleOpRed")
public class TwelveBallAutoRedGate extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Follower follower;

    private AutoState currentState = AutoState.IDLE;
    private long stateStartTime = 0;

    // Timing queue for sub-state actions
    private List<ScheduledAction> scheduledActions = new ArrayList<>();

    // Timing constants (tunable)
    private static final long SHOOT_SEQUENCE_TIME = 2900;
    private static final long INTAKE_START_DELAY = 3000;
    private static final long GATE_WAIT_TIME = 1250;  // Wait at gate before continuing

    // Shot velocities (tunable for each shot)
    private static final double SHOT_1_VELOCITY = 1315.0 + 25.0;  // Preload shot
    private static final double SHOT_2_VELOCITY = 1365.0 + 25.0;  // After spike mark 1
    private static final double SHOT_3_VELOCITY = 1350.0 + 25.0;  // After spike mark 2
    private static final double SHOT_4_VELOCITY = 1350.0 + 25.0;  // After spike mark 3

    private enum AutoState {
        IDLE,
        // First volley (preload)
        DRIVE_TO_SCORE_1, SHOOT_1,
        // Second volley (spike mark 1) - NOW USES GATE PATH
        DRIVE_TO_SPIKE_1, INTAKE_SPIKE_1, DRIVE_TO_GATE, WAIT_AT_GATE, DRIVE_GATE_TO_SCORE_2, SHOOT_2,
        // Third volley (spike mark 2)
        DRIVE_TO_SPIKE_2, INTAKE_SPIKE_2, DRIVE_BACK_TO_SCORE_3, SHOOT_3,
        // Fourth volley (spike mark 3)
        DRIVE_TO_SPIKE_3, INTAKE_SPIKE_3, DRIVE_BACK_TO_SCORE_4, SHOOT_4,
        // Park
        DRIVE_TO_PARK, COMPLETE
    }

    private static class ScheduledAction {
        long executeTime;
        Runnable action;

        ScheduledAction(long executeTime, Runnable action) {
            this.executeTime = executeTime;
            this.action = action;
        }
    }

    public TwelveBallAutoRedGate() {
        intake = Intake.INSTANCE;
        outtake = Outtake.INSTANCE;
        ll = LL.INSTANCE;
        drive = Drive.INSTANCE;

        addComponents(
                new SubsystemComponent(ll, drive, intake, outtake),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        outtake.initialize(hardwareMap);
        ll.initialize(hardwareMap);

        follower = PedroComponent.follower();

        // Build RED alliance trajectories (sets LL.ID = 20)
        TrajectoryFactory.buildTrajectories(follower, true);
        follower.setStartingPose(TrajectoryFactory.goalStartPos.mirror());

        telemetry.addLine("🔴 12-Ball State Machine Ready (Gate Path)");
        telemetry.addData("Target Tag ID", LL.ID);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Ensure overriding is off for auto
        org.firstinspires.ftc.teamcode.commandbase.Routines.overriding = false;
        changeState(AutoState.DRIVE_TO_SCORE_1);
    }

    @Override
    public void onUpdate() {
        follower.update();

        // Execute scheduled actions
        long now = System.currentTimeMillis();
        Iterator<ScheduledAction> it = scheduledActions.iterator();
        while (it.hasNext()) {
            ScheduledAction action = it.next();
            if (now >= action.executeTime) {
                action.action.run();
                it.remove();
            }
        }

        // State transitions
        switch (currentState) {
            case DRIVE_TO_SCORE_1:
                if (!follower.isBusy()) changeState(AutoState.SHOOT_1);
                break;
            case SHOOT_1:
                if (getStateTime() > SHOOT_SEQUENCE_TIME) changeState(AutoState.DRIVE_TO_SPIKE_1);
                break;

            case DRIVE_TO_SPIKE_1:
                if (!follower.isBusy()) changeState(AutoState.INTAKE_SPIKE_1);
                break;
            case INTAKE_SPIKE_1:
                if (!follower.isBusy()) changeState(AutoState.DRIVE_TO_GATE);
                break;
            case DRIVE_TO_GATE:
                if (!follower.isBusy()) changeState(AutoState.WAIT_AT_GATE);
                break;
            case WAIT_AT_GATE:
                if (getStateTime() > GATE_WAIT_TIME) changeState(AutoState.DRIVE_GATE_TO_SCORE_2);
                break;
            case DRIVE_GATE_TO_SCORE_2:
                if (!follower.isBusy()) changeState(AutoState.SHOOT_2);
                break;
            case SHOOT_2:
                if (getStateTime() > SHOOT_SEQUENCE_TIME) changeState(AutoState.DRIVE_TO_SPIKE_2);
                break;

            case DRIVE_TO_SPIKE_2:
                if (!follower.isBusy()) changeState(AutoState.INTAKE_SPIKE_2);
                break;
            case INTAKE_SPIKE_2:
                if (!follower.isBusy()) changeState(AutoState.DRIVE_BACK_TO_SCORE_3);
                break;
            case DRIVE_BACK_TO_SCORE_3:
                if (!follower.isBusy()) changeState(AutoState.SHOOT_3);
                break;
            case SHOOT_3:
                if (getStateTime() > SHOOT_SEQUENCE_TIME) changeState(AutoState.DRIVE_TO_SPIKE_3);
                break;

            case DRIVE_TO_SPIKE_3:
                if (!follower.isBusy()) changeState(AutoState.INTAKE_SPIKE_3);
                break;
            case INTAKE_SPIKE_3:
                if (!follower.isBusy()) changeState(AutoState.DRIVE_BACK_TO_SCORE_4);
                break;
            case DRIVE_BACK_TO_SCORE_4:
                if (!follower.isBusy()) changeState(AutoState.SHOOT_4);
                break;
            case SHOOT_4:
                if (getStateTime() > SHOOT_SEQUENCE_TIME) changeState(AutoState.DRIVE_TO_PARK);
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) changeState(AutoState.COMPLETE);
                break;
        }

        updateTelemetry();
    }

    private void changeState(AutoState newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();
        scheduledActions.clear();

        switch (newState) {
            case DRIVE_TO_SCORE_1:
                intake.keeping.schedule();
                follower.followPath(TrajectoryFactory.goalToScore, true);

                // START SPINNING UP MOTORS IMMEDIATELY DURING DRIVE
                Outtake.shooting = true;
                LL.targetVel = SHOT_1_VELOCITY;
                break;

            case SHOOT_1:
            case SHOOT_2:
            case SHOOT_3:
            case SHOOT_4:
                // Motors should already be at speed, execute shoot sequence immediately
                executeShootSequence();
                break;

            case DRIVE_TO_SPIKE_1:
            case DRIVE_TO_SPIKE_2:
            case DRIVE_TO_SPIKE_3:
                // Stop motors while collecting balls
                Outtake.shooting = false;
                LL.targetVel = 0.0;
                intake.on.schedule();

                // Start path
                if (newState == AutoState.DRIVE_TO_SPIKE_1) {
                    follower.followPath(TrajectoryFactory.scoreToSpikeMark1, true);
                } else if (newState == AutoState.DRIVE_TO_SPIKE_2) {
                    follower.followPath(TrajectoryFactory.scoreToSpikeMark2, true);
                } else {
                    follower.followPath(TrajectoryFactory.scoreToSpikeMark3, true);
                }

                // Schedule intake to start after delay
                scheduleAction(INTAKE_START_DELAY, () -> {
                    outtake.spinServo1.setPower(-0.7);
                    outtake.spinServo2.setPower(-0.7);
                });
                break;

            case INTAKE_SPIKE_1:
                follower.followPath(TrajectoryFactory.spikeMark1ToEnd, true);
                break;

            case INTAKE_SPIKE_2:
                follower.followPath(TrajectoryFactory.spikeMark2ToEnd, true);
                break;

            case INTAKE_SPIKE_3:
                follower.followPath(TrajectoryFactory.spikeMark3ToEnd, true);
                break;

            case DRIVE_TO_GATE:
                // Continue intaking while driving to gate
                follower.followPath(TrajectoryFactory.spikeMark1EndToGate, true);
                break;

            case WAIT_AT_GATE:
                // Wait at gate position for 1 second (no action needed, just waiting)
                // Robot continues holding balls in keeping mode
                break;

            case DRIVE_GATE_TO_SCORE_2:
                // Switch to keeping mode and stop spin servos
                intake.keeping.schedule();
                outtake.spinServo1.setPower(-0.5);
                outtake.spinServo2.setPower(-0.5);

                // START SPINNING UP MOTORS DURING DRIVE BACK
                Outtake.shooting = true;
                LL.targetVel = SHOT_2_VELOCITY;

                // Follow gate to score path
                follower.followPath(TrajectoryFactory.gateToScore, true);
                break;

            case DRIVE_BACK_TO_SCORE_3:
            case DRIVE_BACK_TO_SCORE_4:
                // Switch to keeping mode and stop spin servos
                intake.keeping.schedule();
                outtake.spinServo1.setPower(-0.5);
                outtake.spinServo2.setPower(-0.5);

                // START SPINNING UP MOTORS DURING DRIVE BACK
                Outtake.shooting = true;

                // Set velocity based on which shot
                if (newState == AutoState.DRIVE_BACK_TO_SCORE_3) {
                    LL.targetVel = SHOT_3_VELOCITY;
                } else {
                    LL.targetVel = SHOT_4_VELOCITY;
                }

                // Start appropriate path
                if (newState == AutoState.DRIVE_BACK_TO_SCORE_3) {
                    follower.followPath(TrajectoryFactory.spikeMark2EndToScore, true);
                } else {
                    follower.followPath(TrajectoryFactory.spikeMark3EndToScore, true);
                }
                break;

            case DRIVE_TO_PARK:
                // Stop motors
                Outtake.shooting = false;
                LL.targetVel = 0.0;

                follower.followPath(TrajectoryFactory.scoreToOutOfTheWay, true);
                intake.keeping.schedule();
                break;

            case COMPLETE:
                intake.off.schedule();
                outtake.spinServo1.setPower(0);
                outtake.spinServo2.setPower(0);
                LL.targetVel = 0.0;
                Outtake.shooting = false;
                break;
        }
    }

    /**
     * Executes the precise shoot sequence timing
     * Matches the timing from Routines.java
     */
    private void executeShootSequence() {
        // T+0ms: Reverse intake and spin servos backward
        scheduleAction(0, () -> {
            intake.revmoving.schedule();
            outtake.spinServo1.setPower(-1.0);
            outtake.spinServo2.setPower(-1.0);
        });

        // T+210ms: Stop intake and spin servos (210ms of reversing)
        scheduleAction(185, () -> {
            intake.off.schedule();
            outtake.spinServo1.setPower(0);
            outtake.spinServo2.setPower(0);
        });

        // T+500ms: Open gate
        scheduleAction(500, () -> {
            outtake.open.schedule();
        });

        // T+900ms: Start intake forward
        scheduleAction(900, () -> {
            intake.autonshooting.schedule();
        });

        // T+2400ms: Close gate and cleanup
        scheduleAction(SHOOT_SEQUENCE_TIME - 100, () -> {
            outtake.close.schedule();
            intake.off.schedule();
            outtake.spinServo1.setPower(0);
            outtake.spinServo2.setPower(0);
            // Keep Outtake.shooting = true and motors running for next volley
        });
    }

    private void scheduleAction(long delayMs, Runnable action) {
        scheduledActions.add(new ScheduledAction(
                System.currentTimeMillis() + delayMs,
                action
        ));
    }

    private long getStateTime() {
        return System.currentTimeMillis() - stateStartTime;
    }

    private void updateTelemetry() {
        telemetry.addLine("=== STATE MACHINE ===");
        telemetry.addData("State", currentState);
        telemetry.addData("State Time", "%.2fs", getStateTime() / 1000.0);
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addLine();

        telemetry.addLine("=== VISION ===");
        telemetry.addData("Tag Visible", LL.tagVisible ? "YES" : "NO");
        if (LL.tagVisible) {
            telemetry.addData("Distance", "%.1f in", LL.distance);
            telemetry.addData("Heading Adj", "%.1f°", LL.headingAdjust);
        }
        telemetry.addLine();

        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("LL.targetVel", "%.0f", LL.targetVel);
        telemetry.addData("Outtake.shooting", Outtake.shooting ? "YES" : "NO");
        telemetry.addData("Top Motor Power", "%.3f", outtake.Tmotor.getPower());
        telemetry.addData("Bottom Motor Power", "%.3f", outtake.Bmotor.getPower());
        telemetry.addData("Top Motor RPM", "%.0f", outtake.Tmotor.getVelocity());
        telemetry.addData("Bottom Motor RPM", "%.0f", outtake.Bmotor.getVelocity());

        if (LL.targetVel > 0) {
            telemetry.addData("Top Error", "%.0f RPM", LL.targetVel - outtake.Tmotor.getVelocity());
            telemetry.addData("Bottom Error", "%.0f RPM", LL.targetVel - outtake.Bmotor.getVelocity());
        }
        telemetry.addLine();

        telemetry.addData("Scheduled Actions", scheduledActions.size());
        telemetry.update();
    }

    @Override
    public void onStop() {
        intake.off.schedule();
        outtake.spinServo1.setPower(0);
        outtake.spinServo2.setPower(0);
        outtake.Tmotor.setPower(0);
        outtake.Bmotor.setPower(0);
        LL.targetVel = 0.0;
        Outtake.shooting = false;
    }
}