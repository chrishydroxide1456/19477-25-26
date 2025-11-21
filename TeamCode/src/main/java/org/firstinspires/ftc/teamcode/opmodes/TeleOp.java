package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends NextFTCOpMode {

    private final Drive drive = Drive.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Outtake outtake = Outtake.INSTANCE;

    @Override
    public void onInit() {
        addComponents(
                SubsystemComponent(drive, intake, outtake),
                BulkReadComponent,
                BindingsComponent
        );
    }

    @Override
    public void onBindings() {
        // Drive controls - continuous driving with gamepad1 sticks
        run(() -> drive.driveWithGamepad(gamepad1));
        
        //region Intake Controls (Gamepad1)
        
        // Right trigger: Run intake forward
        gamepad1.rightTrigger.asButton(value -> value > 0.5)
                .whenBecomesTrue(() -> intake.start.schedule())
                .whenBecomesFalse(() -> intake.stop.schedule());
        
        // Left trigger: Reverse intake
        gamepad1.leftTrigger.asButton(value -> value > 0.5)
                .whenBecomesTrue(() -> intake.reverse.schedule())
                .whenBecomesFalse(() -> intake.stop.schedule());
        
        // Alternative button controls for intake
        gamepad1.a.whenBecomesTrue(() -> intake.start.schedule());
        gamepad1.b.whenBecomesTrue(() -> intake.stop.schedule());
        gamepad1.x.whenBecomesTrue(() -> intake.reverse.schedule());
        
        //endregion
        
        //region Outtake Controls (Gamepad1)
        
        // Right bumper: Run outtake motors
        gamepad1.rightBumper.whenBecomesTrue(() -> outtake.on.schedule())
                            .whenBecomesFalse(() -> outtake.off.schedule());
        
        // Left bumper: Reverse outtake motors
        gamepad1.leftBumper.whenBecomesTrue(() -> outtake.reverse.schedule())
                           .whenBecomesFalse(() -> outtake.off.schedule());
        
        // Y button: Turn off outtake
        gamepad1.y.whenBecomesTrue(() -> outtake.off.schedule());
        
        // D-pad controls for servo blocker
        gamepad1.dpadUp.whenBecomesTrue(() -> outtake.open.schedule());    // Open blocker
        gamepad1.dpadDown.whenBecomesTrue(() -> outtake.close.schedule()); // Close blocker
        
        //endregion
    }

}
