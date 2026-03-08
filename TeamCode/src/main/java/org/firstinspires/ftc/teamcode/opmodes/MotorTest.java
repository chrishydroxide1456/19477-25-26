package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "Dashboard Motor Test", group = "Test")
public class MotorTest extends OpMode {
    //    private final MotorEx testmotor = new MotorEx("BRmotor");
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "FLmotor");
    }

    @Override
    public void loop() {
        motor.setPower(0.3);
    }
}

//0.23 min
//0.36 max