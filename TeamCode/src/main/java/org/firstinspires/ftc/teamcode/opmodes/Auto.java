package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.LL;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Odo;

@Autonomous
public class Auto extends NextFTCOpMode {

    private Drive drive = Drive.INSTANCE;
    private Intake intake = Intake.INSTANCE;
    private Outtake outtake = Outtake.INSTANCE;
    private LL limelight = LL.INSTANCE;
    private Odo odometry = Odo.INSTANCE;

    @Override
    public void onInit() {
        addComponents(
                SubsystemComponent(drive),
                SubsystemComponent(intake),
                SubsystemComponent(outtake),
                SubsystemComponent(limelight),
                SubsystemComponent(odometry),
                BulkReadComponent
        );
    }

    @Override
    public void onStart() {
        // TODO: Add autonomous sequence here
        // Example autonomous sequence:
        // 1. Drive forward
        // 2. Score preload
        // 3. Navigate to samples
        // 4. Collect and score samples
        // 5. Park
    }

}
