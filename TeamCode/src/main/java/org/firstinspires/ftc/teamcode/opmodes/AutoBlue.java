
package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.LL.*;
import static dev.nextftc.bindings.Bindings.button;

import dev.nextftc.core.components.*;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.Routines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoBlue extends NextFTCOpMode {

    private Intake intake;
    private Outtake outtake;
    private LL ll;
    private Drive drive;
    private Routines routines;

    @Override
    public void onInit() {
        LL.ID = 20;
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
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

}