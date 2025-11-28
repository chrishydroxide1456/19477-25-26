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
//    public Intake intake = Intake.INSTANCE;
//    public Outtake outtake = Outtake.INSTANCE;
//    public LL ll = LL.INSTANCE;
//    public Drive drive = Drive.INSTANCE;
//    public Routines routines = new Routines();
    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Routines routines;

    public void onInit() {
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

        button(() -> gamepad2.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequencemoving().start())
                .whenBecomesFalse(() ->intake.off.start());

        button(() -> gamepad2.dpad_up) //change to x later
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().start())
                .whenBecomesFalse(() -> intake.off.stop(true));

        button(() -> gamepad2.dpad_down)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.reverse.start())
                .whenBecomesFalse(() -> intake.off.start());

//        button(() -> gamepad2.a)
//                .toggleOnBecomesTrue()
//                .whenBecomesTrue(() -> routines.outSequence().start());

        button(() -> gamepad1.xWasPressed())
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.outtaketest1().start());

        button(() -> gamepad1.aWasPressed())
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.outtaketest2().start());
    }


    public void onUpdate() {
        drive.driverdrive(gamepad1);
        if (gamepad2.a) {
            ll.setID();
        }

        ll.adjust();
        drive.autodrive(headingAdjust);

        telemetry.addData("ID", ID);
        telemetry.addData("headingadjust", headingAdjust);
        telemetry.addData("distance", distance);
        telemetry.addData("targetvelo", targetVel);
        telemetry.update();
    }

}
