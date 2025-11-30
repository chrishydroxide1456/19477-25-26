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
public class TeleOpBlue extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Routines routines;

    // LED blinking variables
    private long ledBlinkTimer = 0;
    private boolean ledState = false;

    // Outtake sequence tracking
    private boolean outtakeSequenceActive = false;
    private long sequenceStartTime = 0;

    @Override
    public void onInit() {
        // Initialize subsystems
        LL.ID = 20;
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

        // ------------------ BUTTON BINDINGS ------------------

        // Test outtake sequence
        button(() -> gamepad2.dpad_up)
                .whenBecomesTrue(() -> {
                    routines.testoutSequence().schedule();
                    outtakeSequenceActive = true;
                    sequenceStartTime = System.currentTimeMillis();
                });

        // Intake toggle
        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().schedule())
                .whenBecomesFalse(() -> routines.stopinSequence().schedule());

        // Intake keeping
        button(() -> gamepad2.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.keeping.start())
                .whenBecomesFalse(() -> intake.off.start());

        // Intake reverse
        button(() -> gamepad2.x)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.reverse.schedule())
                .whenBecomesFalse(() -> intake.off.schedule());
    }

    @Override
    public void onUpdate() {
        // Driving
        drive.driverdrive(gamepad2);

        // Update Limelight ID if button pressed
        if (gamepad2.a) {
            ll.setID();
        }

        // ------------------ LED CONTROL ------------------
        long currentTime = System.currentTimeMillis();
        if (currentTime - ledBlinkTimer >= 500) { // toggle every 0.5s
            ledBlinkTimer = currentTime;
            ledState = !ledState;

            if (outtakeSequenceActive) {
                // Red LED during sequence
                outtake.led.setPosition(ledState ? 1.0 : 0.7);
            } else {
                // Green LED otherwise
                outtake.led.setPosition(ledState ? 0.5 : 0.3);
            }
        }

        // Reset sequence flag after 4 seconds (or adjust to match your routine duration)
        if (outtakeSequenceActive && currentTime - sequenceStartTime > 4000) {
            outtakeSequenceActive = false;
        }

        // ------------------ TELEMETRY ------------------
        telemetry.addData("ID", ID);
        telemetry.addData("headingAdjust", headingAdjust);
        telemetry.addData("distance", distance);
        telemetry.addData("targetVel", targetVel);
        telemetry.update();
    }
}
