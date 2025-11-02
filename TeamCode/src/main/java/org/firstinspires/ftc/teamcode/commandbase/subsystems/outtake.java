package org.firstinspires.ftc.teamcode.commandbase.subsystems;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.LED;

import static org.firstinspires.ftc.teamcode.constants.*;

public class outtake extends SubsystemBase {

    private Servo Lgate, Rgate;
    private MotorEx Lflywheel, Rflywheel;
    private MotorGroup flywheel;
    private GamepadEx gamepad1;
    private final PIDFController flywheelcontroller = new PIDFController(FLYWHEEL_PIDF_CONSTANTS);
    private LED maxRPMled;

    @Override
    public void init(HardwareMap hwMap) {
        MotorEx Lflywheel = new MotorEx(hwMap, "Lflywheel", Motor.GoBILDA.RPM_312); //i think this is the right one
        MotorEx Rflywheel = new MotorEx(hwMap, "Rflywheel", Motor.GoBILDA.RPM_312);
        //grab internal dcmotor object??
        Lflywheel.setRunMode(Motor.RunMode.VelocityControl);
        Rflywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel = new MotorGroup(Lflywheel, Rflywheel);
        flywheel.setVeloCoefficients(kP, kI, kD);

        Servo Lgate = hwMap.get(Servo.class, "Lgate");
        Servo Rgate = hwMap.get(Servo.class "Rgate");
        Lgate.setPosition(gateCloseDist);
        Rgate.setPosition(gateCloseDist);

        maxRPMled = hwMap.get(LED.class, "maxRPMled");
        maxRPMled.off()
    }


    public void openGate() {
        Lgate.setPosition(gateOpenDist);
        Rgate.setPosition(gateOpenDist);
    }
    public void closeGate() {
        Lgate.setPosition(gateCloseDist);
        Rgate.setPosition(gateCloseDist);
    }
    public void launch() { //if gamepad button is pressed in opmode
        Rflywheel.setVelocity(targetVelocityTPS);
    }
    //ok so in the opmode we can use feedbackloop to tell the LED to turn on
    public void maxRPMreached(double flywheelRPM) {
        if (flywheelRPM > 5800) { //adding a little tolerance, may adjust later
            maxRPMled.on();
        }
        else {
            maxRPMled.off()
        }
    }



    public void updateflywheel() {
        flywheel.
    }

    public void periodic() {
        updateflywheel();
    }
}

//transfer in shootOrient code later
