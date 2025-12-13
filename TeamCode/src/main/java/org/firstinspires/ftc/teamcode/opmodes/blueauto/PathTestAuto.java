package org.firstinspires.ftc.teamcode.opmodes.blueauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.opmodes.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "🔵 Path Test - 9 Ball Route", group = "Test", preselectTeleOp = "TeleOpBlue")
public class PathTestAuto extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;

    // Constructor - this is where components should be added (like Kotlin's init block)
    public PathTestAuto() {
        // Initialize subsystems
        intake = Intake.INSTANCE;
        outtake = Outtake.INSTANCE;
        ll = LL.INSTANCE;
        drive = Drive.INSTANCE;

        // Add components (including PedroComponent with Constants)
        addComponents(
                new SubsystemComponent(ll, drive, intake, outtake),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        // Initialize hardware-dependent subsystems
        outtake.initialize(hardwareMap);
        ll.initialize(hardwareMap);

        // Build trajectories for blue alliance (follower is now initialized)
        TrajectoryFactory.buildTrajectories(PedroComponent.follower(), false);

        // Set starting pose
        PedroComponent.follower().setStartingPose(TrajectoryFactory.goalStartPos);

        telemetry.addLine("✅ Path Test Initialized");
        telemetry.addData("Starting Pose", TrajectoryFactory.goalStartPos);
        telemetry.addLine("Will drive 9-ball route without intake/shooting");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Just the paths - no intake or shooting
        Command pathTestSequence = new SequentialGroup(
                // First volley path - go directly to score
                new FollowPath(TrajectoryFactory.goalToScore, true),
                new Delay(1.0), // Simulate shooting time

                // Second volley paths (spike mark 1)
                new FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                new FollowPath(TrajectoryFactory.spikeMark1ToEnd, true),
                new FollowPath(TrajectoryFactory.spikeMark1EndToScore, true),
                new Delay(1.0), // Simulate shooting time

                // Third volley paths (spike mark 2)
                new FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                new FollowPath(TrajectoryFactory.spikeMark2ToEnd, true),
                new FollowPath(TrajectoryFactory.spikeMark2EndToScore, true),
                new Delay(1.0), // Simulate shooting time

                // Park
                new FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true)
        );

        pathTestSequence.schedule();
    }

    @Override
    public void onUpdate() {
        // Update telemetry
        telemetry.addData("Current Pose", PedroComponent.follower().getPose());
        telemetry.addData("Is Busy", PedroComponent.follower().isBusy());
        telemetry.addData("X", "%.1f", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", "%.1f", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void onStop() {
        // Stop all motors
        intake.off.schedule();
        outtake.spinServo1.setPower(0);
        outtake.spinServo2.setPower(0);
    }
}