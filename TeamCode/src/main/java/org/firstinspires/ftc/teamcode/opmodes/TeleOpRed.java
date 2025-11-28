package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.*;
import static dev.nextftc.bindings.Bindings.button;

import dev.nextftc.core.components.*;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpRed extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Routines routines;

    @Override
    public void onInit() {
        LL.ID = 24;
        Drive.INSTANCE.stopAutoAlign();

        intake = Intake.INSTANCE;
        outtake = Outtake.INSTANCE;
        outtake.initialize(hardwareMap);
        ll = LL.INSTANCE;
        ll.initialize(hardwareMap);
        drive = Drive.INSTANCE;
        routines = new Routines(intake, outtake, ll, drive);

        addComponents(
                new SubsystemComponent(ll, drive, intake, outtake),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        // DPAD UP = AUTO ALIGN
        button(() -> gamepad2.dpad_up).whenBecomesTrue(() -> {
            if (tagVisible) {
                routines.autoAlignOnly().schedule();
            } else {
                gamepad2.rumble(100); // Haptic feedback for no target
            }
        });

        // DPAD DOWN = SHOOT SEQUENCE
        button(() -> gamepad2.dpad_down).whenBecomesTrue(() -> {
            if (tagVisible) {
                routines.testoutSequence().schedule();
            } else {
                gamepad2.rumble(100); // Haptic feedback for no target
            }
        });

        // A = intake toggle
        button(() -> gamepad2.a).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().schedule())
                .whenBecomesFalse(() -> routines.stopinSequence().schedule());
        // A = intake toggle
        button(() -> gamepad2.x).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.reverse.schedule())
                .whenBecomesFalse(() -> routines.stopReverseSequence().schedule());
    }

    @Override
    public void onUpdate() {
        drive.driverdrive(gamepad2);

        if (tagVisible) {
            // Calculate what the RPM SHOULD be
            double calculatedRPM = ll.gettargetVel(distance);

            telemetry.addLine("=== SHOOT SYSTEM ===");
            telemetry.addData("Distance", "%.1f in", distance);
            telemetry.addData("Calculated RPM", "%.0f", calculatedRPM);
            telemetry.addData("Static targetVel", "%.0f RPM", targetVel);
            telemetry.addData("Match?", calculatedRPM == targetVel ? "YES" : "NO!");
            telemetry.addData("Power", "%.2f%%", (0.25 + (targetVel / 3000.0)) * 100);
            telemetry.addData("headingAdjust", "%.1f°", headingAdjust);
            telemetry.addData("AutoAlign", Drive.INSTANCE.isAutoAlignActive());
            telemetry.addData("Shooting", Outtake.shooting);
        } else {
            telemetry.addLine("=== NO TARGET ===");
            telemetry.addData("Limelight", "No AprilTag visible");
        }
        telemetry.update();
    }
}