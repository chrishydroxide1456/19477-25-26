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
    private static final long FLYWHEEL_SPINUP_TIME = 1200;    // Time to reach target RPM (increased for safety)
    private static final long SHOOT_SEQUENCE_TIME = 2500;      // Total shoot duration
    private static final long INTAKE_START_DELAY = 400;        // Delay before starting intake

    private enum AutoState {
        IDLE,
        // First volley (preload)
        DRIVE_TO_SCORE_1, SPINUP_1, SHOOT_1,
        // Second volley (spike mark 1)
        DRIVE_TO_SPIKE_1, INTAKE_SPIKE_1, DRIVE_BACK_TO_SCORE_2, SPINUP_2, SHOOT_2,
        // Third volley (spike mark 2)
        DRIVE_TO_SPIKE_2, INTAKE_SPIKE_2, DRIVE_BACK_TO_SCORE_3, SPINUP_3, SHOOT_3,
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

        // DON'T start intake on init - wait for auto to start
        // intake.keeping.schedule();  // REMOVED

        telemetry.addLine("🔴 9-Ball State Machine Ready");
        telemetry.addData("Target Tag ID", LL.ID);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
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
                if (!follower.isBusy()) changeState(AutoState.SPINUP_1);
                break;
            case SPINUP_1:
                if (getStateTime() > FLYWHEEL_SPINUP_TIME) changeState(AutoState.SHOOT_1);
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
                if (!follower.isBusy()) changeState(AutoState.SPINUP_2);
                break;
            case SPINUP_2:
                if (getStateTime() > FLYWHEEL_SPINUP_TIME) changeState(AutoState.SHOOT_2);
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
                if (!follower.isBusy()) changeState(AutoState.SPINUP_3);
                break;
            case SPINUP_3:
                if (getStateTime() > FLYWHEEL_SPINUP_TIME) changeState(AutoState.SHOOT_3);
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
                break;

            case SPINUP_1:
                // CRITICAL: Set targetVel BEFORE shooting flag
                // This ensures Outtake.periodic() captures the velocity
                LL.targetVel = 1600.0;
                Outtake.shooting = true;
                // LED will turn GREEN automatically in Outtake.periodic()
                break;

            case SHOOT_1:
            case SHOOT_2:
            case SHOOT_3:
                executeShootSequence();
                break;

            case DRIVE_TO_SPIKE_1:
            case DRIVE_TO_SPIKE_2:
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
                // Intake and spin servos already running
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

                // Start appropriate path
                if (newState == AutoState.DRIVE_BACK_TO_SCORE_2) {
                    follower.followPath(TrajectoryFactory.spikeMark1EndToScore, true);
                } else {
                    follower.followPath(TrajectoryFactory.spikeMark2EndToScore, true);
                }
                break;

            case SPINUP_2:
            case SPINUP_3:
                // CRITICAL: Set targetVel BEFORE shooting flag
                if (LL.tagVisible) {
                    LL.targetVel = ll.gettargetVel(LL.distance);
                } else {
                    LL.targetVel = 1600.0; // Fallback velocity
                }
                Outtake.shooting = true;
                // Outtake.periodic() will handle the PID control
                break;

            case DRIVE_TO_PARK:
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

        // T+150ms: Stop intake and spin servos (150ms of reversing)
        scheduleAction(150, () -> {
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
            intake.onmoving.schedule();
        });

        // T+2400ms: Close gate and cleanup
        scheduleAction(SHOOT_SEQUENCE_TIME - 100, () -> {
            outtake.close.schedule();
            intake.off.schedule();
            outtake.spinServo1.setPower(0);
            outtake.spinServo2.setPower(0);
            Outtake.shooting = false;
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
        telemetry.addData("Target RPM", "%.0f", LL.targetVel);
        telemetry.addData("Top Motor", "%.0f RPM", outtake.Tmotor.getVelocity());
        telemetry.addData("Bottom Motor", "%.0f RPM", outtake.Bmotor.getVelocity());
        telemetry.addData("Top Error", "%.0f", LL.targetVel - outtake.Tmotor.getVelocity());
        telemetry.addData("Bottom Error", "%.0f", LL.targetVel - outtake.Bmotor.getVelocity());
        telemetry.addData("Shooting", Outtake.shooting ? "YES" : "NO");
        telemetry.addLine();

        telemetry.addData("Scheduled Actions", scheduledActions.size());
        telemetry.update();
    }

    @Override
    public void onStop() {
        intake.off.schedule();
        outtake.spinServo1.setPower(0);
        outtake.spinServo2.setPower(0);
        LL.targetVel = 0.0;
        Outtake.shooting = false;
    }
}