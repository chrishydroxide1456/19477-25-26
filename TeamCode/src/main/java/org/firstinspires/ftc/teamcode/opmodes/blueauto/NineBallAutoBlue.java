package org.firstinspires.ftc.teamcode.opmodes.blueauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.opmodes.AutoRoutines;
import org.firstinspires.ftc.teamcode.opmodes.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "9-Ball Auto Blue", group = "Blue", preselectTeleOp = "TeleOpBlue")
public class NineBallAutoBlue extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Routines routines;
    private AutoRoutines autoRoutines;

    @Override
    public void onInit() {
        // Initialize subsystems
        intake = Intake.INSTANCE;
        outtake = Outtake.INSTANCE;
        outtake.initialize(hardwareMap);
        ll = LL.INSTANCE;
        ll.initialize(hardwareMap);
        drive = Drive.INSTANCE;

        // Initialize routines
        routines = new Routines(intake, outtake, ll, drive);
        autoRoutines = new AutoRoutines(routines, intake);

        // Add components (including PedroComponent with Constants)
        addComponents(
                new SubsystemComponent(ll, drive, intake, outtake),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

        // Build trajectories for blue alliance
        TrajectoryFactory.buildTrajectories(PedroComponent.follower(), false);

        // Set starting pose
        PedroComponent.follower().setStartingPose(TrajectoryFactory.goalStartPos);

        telemetry.addLine("9-Ball Blue Auto Initialized");
        telemetry.addData("Starting Pose", TrajectoryFactory.goalStartPos);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Schedule the 9-ball autonomous routine
        autoRoutines.nineBallAuto().schedule();
    }

    @Override
    public void onUpdate() {
        // Update telemetry
        telemetry.addData("Current Pose", PedroComponent.follower().getPose());
        telemetry.addData("Is Busy", PedroComponent.follower().isBusy());
        telemetry.addData("Tag Visible", LL.tagVisible);

        if (LL.tagVisible) {
            telemetry.addData("Distance", "%.1f in", LL.distance);
            telemetry.addData("Heading Adjust", "%.1f°", LL.headingAdjust);
        }

        telemetry.addData("Shooting", Outtake.shooting);
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