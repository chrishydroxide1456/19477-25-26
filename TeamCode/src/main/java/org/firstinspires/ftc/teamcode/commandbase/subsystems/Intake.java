package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

public class Intake {
    private final DcMotorEx inmotor;

    public Intake(HardwareMap hwMap) {
        inmotor = hwMap.get(DcMotorEx.class, "inmotor");
    }

    public void intakethem(GamepadEx gamepad) {
        if (gamepad.getButton(GamepadKeys.Button.X)) {
            inmotor.setVelocity(1400.0*28.0/60.0);
        }
        else {
            inmotor.setVelocity(0);
        }
    }

}
