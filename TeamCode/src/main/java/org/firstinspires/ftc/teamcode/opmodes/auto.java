package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;

import java.util.concurrent.TimeUnit;

@Autonomous
public class auto extends LinearOpMode {
    private AutoDrive autodrive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        autodrive = new AutoDrive(hardwareMap);

        waitForStart();

//        runtime.reset();
//        runtime.startTime();

//        while (opModeIsActive()) {
        autodrive.drivewithpower(0.1);
        sleep(100);
        autodrive.stopauto();
        requestOpModeStop();
        }

    }
