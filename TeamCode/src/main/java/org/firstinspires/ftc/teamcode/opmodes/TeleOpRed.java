package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.*;
import static dev.nextftc.bindings.Bindings.button;

import static org.firstinspires.ftc.teamcode.commandbase.Routines.overriding;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.core.commands.utility.InstantCommand;
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

        // LEFT BUMPER = AUTO ALIGN
        button(() -> gamepad1.left_bumper).whenBecomesTrue(() -> {
            if (tagVisible) {
                routines.autoAlignOnly().schedule();
            }
        });

        // RIGHT BUMPER = SHOOT SEQUENCE
        button(() -> gamepad1.right_bumper).whenBecomesTrue(() -> {
            if (tagVisible) {
                overriding = false;
                Outtake.shooting = false;
                routines.testoutSequence().schedule();
            }
        });

        // Y = OVERRIDE MODE (manual 1200 RPM prespin)
        button(() -> gamepad2.y).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> overriding = true)
                .whenBecomesFalse(() -> overriding = false);

        // A = INTAKE TOGGLE
        button(() -> gamepad2.a).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().schedule())
                .whenBecomesFalse(() -> routines.stopinSequence().schedule());

        // X = REVERSE INTAKE
        button(() -> gamepad2.x).toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.reverse.schedule())
                .whenBecomesFalse(() -> routines.stopReverseSequence().schedule());
    }

    @Override
    public void onUpdate() {
        // Speed control
        if (gamepad1.right_trigger > 0.1) {
            drive.setMulti(0.58); // Slow mode
        } else {
            drive.setMulti(1.0);  // Normal speed
        }

        // Manual trigger-based turning for fine adjustments
        double triggerTurn = (gamepad2.left_trigger - gamepad2.right_trigger) * 0.55;

        // Apply driving with trigger turning
        if (Math.abs(triggerTurn) > 0.05) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x + triggerTurn;

            double FLpower = (-y + x + rx) * Drive.multi;
            double BLpower = (y + x - rx) * Drive.multi;
            double FRpower = (-y - x - rx) * Drive.multi;
            double BRpower = (y - x + rx) * Drive.multi;

            Drive.INSTANCE.FLmotor.setPower(FLpower);
            Drive.INSTANCE.FRmotor.setPower(FRpower);
            Drive.INSTANCE.BLmotor.setPower(BLpower);
            Drive.INSTANCE.BRmotor.setPower(BRpower);
        } else {
            drive.driverdrive(gamepad1);
        }

        // Telemetry
        double TmotorRPM = outtake.Tmotor.getVelocity();
        double BmotorRPM = outtake.Bmotor.getVelocity();

        // Beam break status
        telemetry.addLine("=== BEAM BREAK SENSOR ===");
        telemetry.addData("Beam Status", outtake.getBeamBreakState() ? "🔴 BROKEN" : "✅ INTACT");
        if (outtake.getBeamBreakState()) {
            telemetry.addData("Duration", "%.1fs", outtake.getBeamBrokenDuration() / 1000.0);
            if (outtake.isBeamBrokenLongEnough()) {
                telemetry.addData("LED", "🟣 PURPLE (>500ms)");
            }
        }
        telemetry.addLine();

        if (tagVisible) {
            telemetry.addLine("=== SHOOT SYSTEM ===");
            telemetry.addData("Distance", "%.1f in", distance);
            telemetry.addData("Calculated RPM", "%.0f", ll.gettargetVel(distance));
            telemetry.addData("LL Target RPM", "%.0f", targetVel);
            telemetry.addLine();
            telemetry.addData("Top Motor RPM", "%.0f", TmotorRPM);
            telemetry.addData("Top Motor Power", "%.3f", outtake.Tmotor.getPower());
            telemetry.addData("Top Error", "%.0f RPM", targetVel - TmotorRPM);
            telemetry.addLine();
            telemetry.addData("Bottom Motor RPM", "%.0f", BmotorRPM);
            telemetry.addData("Bottom Motor Power", "%.3f", outtake.Bmotor.getPower());
            telemetry.addData("Bottom Error", "%.0f RPM", targetVel - BmotorRPM);
            telemetry.addLine();
            telemetry.addData("Override Mode", overriding);
            telemetry.addData("Shooting", Outtake.shooting);
            telemetry.addData("headingAdjust", "%.1f°", headingAdjust);
            telemetry.addData("AutoAlign", Drive.INSTANCE.isAutoAlignActive());
        } else {
            telemetry.addLine("=== NO TARGET ===");
            telemetry.addData("Limelight", "No AprilTag visible");
            telemetry.addLine();
            telemetry.addLine("=== MOTOR STATUS ===");
            telemetry.addData("Top Motor RPM", "%.0f", TmotorRPM);
            telemetry.addData("Bottom Motor RPM", "%.0f", BmotorRPM);
        }
        telemetry.update();
    }

}