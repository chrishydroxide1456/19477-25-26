package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class intake extends SubsystemBase {

    private MotorEx inWheel;

    public enum
    @Override //1 related problem
    public void init(HardwareMap hwMap) {
        MotorEx inWheel = new MotorEx(hwMap, "inWheel");
    }

}
