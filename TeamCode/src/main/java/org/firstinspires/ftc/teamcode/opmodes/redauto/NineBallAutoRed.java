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

@Deprecated
@Autonomous(name = "🔴 9-Ball Auto Red", group = "Red", preselectTeleOp = "TeleOpRed")
public class NineBallAutoRed extends NextFTCOpMode {

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
    private static final long SPINUP_DELAY_BEFORE_SHOOT = 100;  // Minimal delay, motors already spinning
    private static final long SHOOT_SEQUENCE_TIME = 2500;
    private static final long INTAKE_START_DELAY = 400;

    // Shot velocities (tunable for each shot)
    private static final double SHOT_1_VELOCITY = 1450.0;  // Preload shot
    private static final double SHOT_2_VELOCITY = 1465.0;  // After spike mark 1
    private static final double SHOT_3_VELOCITY = 1475.0;  // After spike mark 2

    private enum AutoState {
        IDLE,
        // First volley (preload)
        DRIVE_TO_SCORE_1, SHOOT_1,
        // Second volley (spike mark 1)
        DRIVE_TO_SPIKE_1, INTAKE_SPIKE_1, DRIVE_BACK_TO_SCORE_2, SHOOT_2,
        // Third volley (spike mark 2)
        DRIVE_TO_SPIKE_2, INTAKE_SPIKE_2, DRIVE_BACK_TO_SCORE_3, SHOOT_3,
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

    public NineBallAutoRed() {
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

        telemetry.addLine("🔴 9-Ball State Machine Ready (EARLY SPINUP)");
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
                if (!follower.isBusy()) changeState(AutoState.DRIVE_BACK_TO_SCORE_2);
                break;
            case DRIVE_BACK_TO_SCORE_2:
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
//                outtake.Tmotor.setPower(0.85);  // Direct power backup
//                outtake.Bmotor.setPower(0.85);
                break;

            case SHOOT_1:
            case SHOOT_2:
            case SHOOT_3:
                // Motors should already be at speed, execute shoot sequence immediately
                executeShootSequence();
                break;

            case DRIVE_TO_SPIKE_1:
            case DRIVE_TO_SPIKE_2:
                // Stop motors while collecting balls
                Outtake.shooting = false;
                LL.targetVel = 0.0;
                outtake.Tmotor.setPower(0);
                outtake.Bmotor.setPower(0);

                // Start path
                if (newState == AutoState.DRIVE_TO_SPIKE_1) {
                    follower.followPath(TrajectoryFactory.scoreToSpikeMark1, true);
                } else {
                    follower.followPath(TrajectoryFactory.scoreToSpikeMark2, true);
                }

                // Schedule intake to start after delay
                scheduleAction(INTAKE_START_DELAY, () -> {
                    intake.on.schedule();
                    outtake.spinServo1.setPower(-1.0);
                    outtake.spinServo2.setPower(-1.0);
                });
                break;

            case INTAKE_SPIKE_1:
                follower.followPath(TrajectoryFactory.spikeMark1ToEnd, true);
                break;

            case INTAKE_SPIKE_2:
                follower.followPath(TrajectoryFactory.spikeMark2ToEnd, true);
                break;

            case DRIVE_BACK_TO_SCORE_2:
            case DRIVE_BACK_TO_SCORE_3:
                // Switch to keeping mode and stop spin servos
                intake.keeping.schedule();
                outtake.spinServo1.setPower(0);
                outtake.spinServo2.setPower(0);

                // START SPINNING UP MOTORS DURING DRIVE BACK
                Outtake.shooting = true;

                // Set velocity based on which shot
                if (newState == AutoState.DRIVE_BACK_TO_SCORE_2) {
                    LL.targetVel = SHOT_2_VELOCITY;
                } else {
                    LL.targetVel = SHOT_3_VELOCITY;
                }

//                outtake.Tmotor.setPower(0.85);  // Direct power backup
//                outtake.Bmotor.setPower(0.85);

                // Start appropriate path
                if (newState == AutoState.DRIVE_BACK_TO_SCORE_2) {
                    follower.followPath(TrajectoryFactory.spikeMark1EndToScore, true);
                } else {
                    follower.followPath(TrajectoryFactory.spikeMark2EndToScore, true);
                }
                break;

            case DRIVE_TO_PARK:
                // Stop motors
                Outtake.shooting = false;
                LL.targetVel = 0.0;
                outtake.Tmotor.setPower(0);
                outtake.Bmotor.setPower(0);

                follower.followPath(TrajectoryFactory.scoreToOutOfTheWay, true);
                intake.keeping.schedule();
                break;

            case COMPLETE:
                intake.off.schedule();
                outtake.spinServo1.setPower(0);
                outtake.spinServo2.setPower(0);
                outtake.Tmotor.setPower(0);
                outtake.Bmotor.setPower(0);
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

        // T+150ms: Stop intake and spin servos (150ms of reversing)
        scheduleAction(210, () -> {
            intake.off.schedule();
            outtake.spinServo1.setPower(0);
            outtake.spinServo2.setPower(0);
        });

        // T+500ms: Open gate
        scheduleAction(500, () -> {
            outtake.open.schedule();
        });

        // T+900ms: Start intake forward (250ms pause after stopping)
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