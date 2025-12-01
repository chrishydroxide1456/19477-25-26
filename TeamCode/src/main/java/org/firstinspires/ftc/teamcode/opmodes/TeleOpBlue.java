package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.*;
import static dev.nextftc.bindings.Bindings.button;

import dev.nextftc.core.components.*;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpBlue extends NextFTCOpMode {

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
            routines.autoAlignOnly().schedule();
        });

        // DPAD DOWN = YOUR WORKING SEQUENCE
        button(() -> gamepad2.dpad_down).whenBecomesTrue(() -> {
            routines.testoutSequence().schedule();  // CHANGED: use testoutSequence
        });

        // A = intake toggle
        button(() -> gamepad2.a).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().schedule())
                .whenBecomesFalse(() -> routines.stopinSequence().schedule());
    }

    @Override
    public void onUpdate() {
        ll.adjust();
        outtake.periodic();
        drive.driverdrive(gamepad2);

        telemetry.addLine("=== SHOOT SYSTEM ===");
        telemetry.addData("headingAdjust", "%.1f°", headingAdjust);
        telemetry.addData("autoAlign", Drive.INSTANCE.isAutoAlignActive());
        telemetry.addData("targetVel", "%.0f", targetVel);
        telemetry.addData("distance", "%.1f in", distance);
        telemetry.update();
    }
}
