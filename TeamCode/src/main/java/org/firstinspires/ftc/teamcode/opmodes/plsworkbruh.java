package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.Odo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;

@TeleOp
public class plsworkbruh extends OpMode {

    private Drive drive;
    private Outtake outtake;
    private Intake intake;

    //private Odo odo;
    private Gamepad driver;
    private Gamepad operator;

    private boolean Tmotorstate = false;
    private boolean lastTmotorstate = false;

//    private boolean inmotorstate = false;
//    private boolean lastinmotorstate = false;
//    private boolean inmotorreverse = false;
//    private boolean lastinmotorreverse = false;
    private boolean servostate = false;
    private boolean lastservostate = false;

    private Servo gateServo;

    public void init() {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        driver = gamepad1;
        operator = gamepad2;

        gateServo = hardwareMap.get(Servo.class, "gateServo");

    }

    @Override
    public void loop() {
//        driver.readButtons();
//        operator.readButtons();
        drive.driveto(driver);
        intake.dointake(operator);

        boolean currentTmotorstate = operator.x;
//        boolean currentinmotorstate = operator.getButton(GamepadKeys.Button.X);
//        boolean currentinmotorreverse = operator.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean currentservostate = driver.b;

        if (currentTmotorstate && !lastTmotorstate) {
            Tmotorstate = !Tmotorstate;
        }
        lastTmotorstate = currentTmotorstate;

//        if (currentinmotorstate && !lastinmotorstate) {
//            inmotorstate = !inmotorstate;
//        }
//        lastinmotorstate = currentinmotorstate;
//
//        if (currentinmotorreverse && !lastinmotorreverse) {
//            inmotorreverse = !inmotorreverse;
//        }
//        lastinmotorreverse = currentinmotorreverse;

        if (currentservostate && !lastservostate) {
            servostate = !servostate;
        }
        lastservostate = currentservostate;

        //execute
        if(Tmotorstate) {
            outtake.outtakeOn();
        } else {
            outtake.outtakeOff();
        }

//        if(inmotorstate) {
//            intake.intakeOn();
//        } else {
//            intake.intakeOff();
//        }
//
//        if(inmotorreverse) {
//            intake.intakeReverse();
//        } else {
//            intake.intakeOff();
//        }

        if(servostate) {
            //outtake.openGate();
            gateServo.setPosition(1.0);
        } else {
            //outtake.closeGate();
            gateServo.setPosition(0.0);
        }


    }
}