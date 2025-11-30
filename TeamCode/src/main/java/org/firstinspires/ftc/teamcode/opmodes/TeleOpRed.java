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
public class TeleOpRed extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Routines routines;

    private long ledBlinkTimer = 0;
    private boolean ledState = false;

    private boolean outtakeSequenceActive = false;
    private long sequenceStartTime = 0;

    @Override
    public void onInit() {
        LL.ID = 0; // ignoring ID for now

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

        button(() -> gamepad2.dpad_up)
                .whenBecomesTrue(() -> {
                    routines.testoutSequence().schedule();
                    outtakeSequenceActive = true;
                    sequenceStartTime = System.currentTimeMillis();
                });

        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().schedule())
                .whenBecomesFalse(() -> routines.stopinSequence().schedule());
    }

    @Override
    public void onUpdate() {
        drive.driverdrive(gamepad2);

        // Vision + flywheel every loop
        ll.adjust();
        outtake.periodic();

        // LED blink
        long currentTime = System.currentTimeMillis();
        if (currentTime - ledBlinkTimer >= 500) {
            ledBlinkTimer = currentTime;
            ledState = !ledState;

            if (outtakeSequenceActive) {
                outtake.led.setPosition(ledState ? 1.0 : 0.7);
            } else {
                outtake.led.setPosition(ledState ? 0.5 : 0.3);
            }
        }

        if (outtakeSequenceActive && currentTime - sequenceStartTime > 4000) {
            outtakeSequenceActive = false;
        }

        telemetry.addData("ID", ID);
        telemetry.addData("headingAdjust", headingAdjust);
        telemetry.addData("distance", distance);
        telemetry.addData("targetVel", targetVel);
        telemetry.addData("autoAlign", drive.isAutoAlignActive());
        telemetry.update();
    }
}
