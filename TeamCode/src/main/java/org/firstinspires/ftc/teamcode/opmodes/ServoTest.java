package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Dashboard Servo Test", group = "Test")
public class ServoTest extends OpMode {
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