package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;

@TeleOp
public class teleop extends OpMode {

    private Drive drive;
    private Outtake outtake;
    private Intake intake;
    private GamepadEx driver;
    private GamepadEx operator;

    public void init() {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        driver = new GamepadEx(gamepad1);

        boolean yeslaunch = false;
        double yesgate = -1.0;

    }

    @Override
    public void loop() {
        driver.readButtons();

        drive.driveTeleop(driver);

        outtake.maxRPM();
        intake.intakethem(driver);

        outtake.togglegate(driver);
        outtake.launch(driver);

        drive.updateOdo();
    }

}
