package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.AutoRoutines;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Odo;

@Autonomous(name = "Basic Auto")
public class Auto extends NextFTCOpMode {

    private final Drive drive = Drive.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Outtake outtake = Outtake.INSTANCE;
    private final LL limelight = LL.INSTANCE;
    private final Odo odometry = Odo.INSTANCE;

    @Override
    public void onInit() {
        addComponents(
                SubsystemComponent(drive, intake, outtake, limelight, odometry),
                BulkReadComponent
        );
    }

    @Override
    public void onStartButtonPressed() {
        // Schedule the basic autonomous routine
        AutoRoutines.basicAutoRoutine().schedule();
    }

}

