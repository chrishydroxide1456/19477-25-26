package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends NextFTCOpMode {

    public void onInit() {
        addComponents(
                SubsystemComponent(LL.INSTANCE),
                BulkReadComponent,
                BindingsComponent
        );
    }

}
