package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.ID;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.distance;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.headingAdjust;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;
import static dev.nextftc.bindings.Bindings.button;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Routines routines;

    @Override
    public void onInit() {
        // Initialize subsystems
        intake = Intake.INSTANCE;
        outtake = Outtake.INSTANCE;
        outtake.initialize(hardwareMap);
        ll = LL.INSTANCE;
        ll.initialize(hardwareMap);
        drive = Drive.INSTANCE;

        routines = new Routines(intake, outtake, ll, drive);

        // Add components for NextFTC
        addComponents(
                new SubsystemComponent(ll, drive, intake, outtake),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        // === Button bindings ===

        // Test outtake sequence
        button(() -> gamepad2.dpad_up)
                .whenBecomesTrue(() -> routines.testoutSequence().schedule());

        // Intake in/out toggle
        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().schedule())
                .whenBecomesFalse(() -> routines.stopinSequence().schedule());


        button(() -> gamepad2.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.keeping.start())
                .whenBecomesFalse(() -> intake.off.start());
        // Intake reverse

        button(() -> gamepad2.x)
                .toggleOnBecomesTrue()   
                .whenBecomesTrue(() -> Intake.INSTANCE.reverse.schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.off.schedule());



// ------------------ OUTTAKE TESTS -----------------------
        // Outtake manual high-speed
//        button(() -> gamepad1.x)
//                .toggleOnBecomesTrue()
//                .whenBecomesTrue(() -> Outtake.INSTANCE.teston4000.schedule())
//                .whenBecomesFalse(() -> Outtake.INSTANCE.testoff.schedule());
//
//        // Outtake test sequence
//        button(() -> gamepad1.a)
//                .toggleOnBecomesTrue()
//                .whenBecomesTrue(() -> routines.outtaketest1().schedule())
//                .whenBecom esFalse(() -> Outtake.INSTANCE.testoff.schedule());
    }

    @Override
    public void onUpdate() {
        // Drive with gamepad1
        drive.driverdrive(gamepad1);

        // LL adjustments (if needed)
        if (gamepad2.a) {
            ll.setID();
        }

        // Telemetry
        telemetry.addData("ID", ID);
        telemetry.addData("headingadjust", headingAdjust);
        telemetry.addData("distance", distance);
        telemetry.addData("targetvelo", targetVel);
        telemetry.update();
    }
}
