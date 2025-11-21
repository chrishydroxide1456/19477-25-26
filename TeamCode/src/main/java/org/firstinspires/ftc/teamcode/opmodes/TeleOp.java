package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Odo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends NextFTCOpMode {

    private Drive drive = Drive.INSTANCE;
    private Intake intake = Intake.INSTANCE;
    private Outtake outtake = Outtake.INSTANCE;
    private LL limelight = LL.INSTANCE;
    private Odo odometry = Odo.INSTANCE;

    public void onInit() {
        addComponents(
                SubsystemComponent(drive),
                SubsystemComponent(intake),
                SubsystemComponent(outtake),
                SubsystemComponent(limelight),
                SubsystemComponent(odometry),
                BulkReadComponent,
                BindingsComponent
        );
    }

    @Override
    public void onBindings() {
        // Drive controls - gamepad1 left stick for translation, right stick for rotation
        run(() -> drive.driverdrive(gamepad1));
        
        // Intake controls - gamepad1
        gamepad1.a.onPress(intake::on);
        gamepad1.b.onPress(intake::off);
        gamepad1.x.onPress(intake::reverse);
        
        // Outtake controls - gamepad2
        gamepad2.a.onPress(outtake.on);
        gamepad2.b.onPress(outtake.off);
        gamepad2.x.onPress(outtake::open);
        gamepad2.y.onPress(outtake::close);
        
        // Limelight adjustment - can be triggered as needed
        gamepad2.dpad_up.onPress(limelight::adjust);
    }

}
