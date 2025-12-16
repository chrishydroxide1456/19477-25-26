package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.Routines.overriding;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.distance;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.headingAdjust;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.tagVisible;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.targetVel;
import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpSolo extends NextFTCOpMode {

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
        button(() -> gamepad2.left_bumper).whenBecomesTrue(() -> {
            if (tagVisible) {
                routines.autoAlignOnly().schedule();
            } else {
                gamepad2.rumble(100); // Haptic feedback for no target
            }
        });

        // DPAD DOWN = SHOOT SEQUENCE
        button(() -> gamepad2.right_bumper).whenBecomesTrue(() -> {
            if (tagVisible) {
                overriding = false;
                Outtake.shooting = false;
                routines.testoutSequence().schedule();
            }
//            else {
//                gamepad2.rumble(100); // Haptic feedback for no target
//            }
        });

        // A = prep spin up
        button(() -> gamepad2.y).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> overriding = true)
                .whenBecomesFalse(() -> overriding = false);

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
        if (gamepad2.right_trigger > 0.1) {
            drive.setMulti(0.58); // Slow mode
        } else {
            drive.setMulti(1.0);  // Normal speed
        }
        // Manual trigger-based turning for fine adjustments
        double triggerTurn = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.55;

        // Create modified gamepad with trigger turning added
        if (Math.abs(triggerTurn) > 0.05) {
            // Apply trigger turning by directly controlling motors
            double y = -gamepad2.left_stick_y;
            double x = -gamepad2.left_stick_x;
            double rx = -gamepad2.right_stick_x + triggerTurn; // Add trigger turning

            double FLpower = (-y + x + rx) * Drive.multi;
            double BLpower = (y + x - rx) * Drive.multi;
            double FRpower = (-y - x - rx) * Drive.multi;
            double BRpower = (y - x + rx) * Drive.multi;

            Drive.INSTANCE.FLmotor.setPower(FLpower);
            Drive.INSTANCE.FRmotor.setPower(FRpower);
            Drive.INSTANCE.BLmotor.setPower(BLpower);
            Drive.INSTANCE.BRmotor.setPower(BRpower);
        } else {
            // Normal driving
            drive.driverdrive(gamepad2);
        }

        // Get current motor RPMs
        double TmotorRPM = outtake.Tmotor.getVelocity();
        double BmotorRPM = outtake.Bmotor.getVelocity();

        if (tagVisible) {
            // Calculate what the RPM SHOULD be
            double calculatedRPM = ll.gettargetVel(distance);

//            if (!Outtake.spinup && !Outtake.shooting) {
//                gamepad2.rumble(0.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
//            }

            telemetry.addLine("=== SHOOT SYSTEM ===");
            telemetry.addData("Distance", "%.1f in", distance);
            telemetry.addData("Target RPM", "%.0f", targetVel);
            telemetry.addData("Top Motor RPM", "%.0f", TmotorRPM);
            telemetry.addData("Bottom Motor RPM", "%.0f", BmotorRPM);
            telemetry.addData("Top Error", "%.0f RPM", targetVel - TmotorRPM);
            telemetry.addData("Bottom Error", "%.0f RPM", targetVel - BmotorRPM);
            telemetry.addData("headingAdjust", "%.1f°", headingAdjust);
            telemetry.addData("AutoAlign", Drive.INSTANCE.isAutoAlignActive());
            telemetry.addData("Shooting", Outtake.shooting);
        } else {
            telemetry.addLine("=== NO TARGET ===");
            telemetry.addData("Limelight", "No AprilTag visible");
            telemetry.addLine("");
            telemetry.addLine("=== MOTOR STATUS ===");
            telemetry.addData("Top Motor RPM", "%.0f", TmotorRPM);
            telemetry.addData("Bottom Motor RPM", "%.0f", BmotorRPM);
        }
        telemetry.update();

    }

}