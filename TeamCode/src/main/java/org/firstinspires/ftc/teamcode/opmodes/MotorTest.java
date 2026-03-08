package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@TeleOp(name = "Dashboard Motor Test", group = "Test")
public class MotorTest extends OpMode {
    private Servo hoodServo;
    public static double servopos;

    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "gateServo1");
        hoodServo.setPosition(0.0);

        servopos = 0.0;
    }

    @Override
    public void loop() {

        hoodServo.setPosition(servopos);

        if (hoodServo != null) {
            hoodServo.setPosition(servopos);
        }

        telemetry.addData("Servo Pos", "%.3f", servopos);
        telemetry.addData("Gamepad1 LB", gamepad1.left_bumper);
        telemetry.update();
    }
}

//0.23 min
//0.36 max