package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends NextFTCOpMode {
    public static Intake intake = Intake.INSTANCE;
    public static Outtake outtake = Outtake.INSTANCE;
    public static LL ll = LL.INSTANCE;
    public static Drive drive = Drive.INSTANCE;
    public static Routines routines = new Routines();

    public void onInit() {
        addComponents(
                new SubsystemComponent(ll, drive, outtake, intake),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public void onUpdate() {
        drive.driverdrive(gamepad1);


        button(() -> gamepad2.dpad_up)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.inSequence().start())
                .whenBecomesFalse(() -> intake.off.start());

        button(() -> gamepad2.dpad_down)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.reverse.start())
                .whenBecomesFalse(() -> intake.off.start());

        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> routines.outSequence().start())
                .whenBecomesFalse(() -> new ParallelGroup(
                        outtake.close,
                        outtake.off
                )
                );

    }

}
