package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Odo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Competition TeleOp")
public class TeleOp extends NextFTCOpMode {

    private final Drive drive = Drive.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Outtake outtake = Outtake.INSTANCE;
    private final LL limelight = LL.INSTANCE;
    private final Odo odometry = Odo.INSTANCE;

    @Override
    public void onInit() {
        addComponents(
                SubsystemComponent(drive, intake, outtake, limelight, odometry),
                BulkReadComponent,
                BindingsComponent
        );
    }

    @Override
    public void onBindings() {
        // Drive controls - continuous driving with gamepad1
        run(() -> drive.driveWithGamepad(gamepad1));
        
        //region Gamepad1 - Driver Controls
        
        // Intake controls using triggers
        gamepad1.rightTrigger.asButton(value -> value > 0.5).whenBecomesTrue(() -> intake.start.schedule())
                .whenBecomesFalse(() -> intake.slowOut.schedule());
        
        gamepad1.leftTrigger.asButton(value -> value > 0.5).whenBecomesTrue(() -> intake.reverse.schedule())
                .whenBecomesFalse(() -> intake.slowOut.schedule());
        
        // Quick intake actions
        gamepad1.a.whenBecomesTrue(() -> intake.start.schedule());
        gamepad1.b.whenBecomesTrue(() -> intake.stop.schedule());
        gamepad1.x.whenBecomesTrue(() -> intake.reverse.schedule());
        
        // Limelight alignment
        gamepad1.leftBumper.whenBecomesTrue(() -> Routines.limelightAdjust().schedule());
        
        // Emergency stop
        gamepad1.back.whenBecomesTrue(() -> Routines.stopAll().schedule());
        
        //endregion
        
        //region Gamepad2 - Operator Controls
        
        // Outtake motor controls
        gamepad2.a.whenBecomesTrue(() -> outtake.start.schedule());
        gamepad2.b.whenBecomesTrue(() -> outtake.stop.schedule());
        
        // Outtake gate controls
        gamepad2.x.whenBecomesTrue(() -> outtake.open.schedule());
        gamepad2.y.whenBecomesTrue(() -> outtake.close.schedule());
        
        // Score sequence
        gamepad2.rightBumper.whenBecomesTrue(() -> Routines.scoreSequence().schedule());
        
        // Intake sequence
        gamepad2.leftBumper.whenBecomesTrue(() -> Routines.intakeSequence().schedule());
        
        // Eject sequence
        gamepad2.back.whenBecomesTrue(() -> Routines.ejectSequence().schedule());
        
        //endregion
    }

}
