package org.firstinspires.ftc.teamcode.commandbase.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.RevIMU;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.kinematics.HolonomicOdometry;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

//what is drive range?
public class mecanumdrive extends SubsystemBase {

    private MotorEx FLmotor, FRmotor, BLmotor, BRmotor;
    private GamepadEx driverOp;

    public mecanumdrive(HardwareMap hwMap, String motorname) {
        super(); //initializing subsystem
        FLmotor = new MotorEx(hwMap, "FLmotor");
        FLmotor = new MotorEx(hwMap, "FLmotor");
        FLmotor = new MotorEx(hwMap, "FLmotor");
        FLmotor = new MotorEx(hwMap, "FLmotor");

        RevIMU imu = new RevIMU(hwMap);
        imu.init();

        GamepadEx driverOp = new GamepadEx(Gamepad gamepad); //add in gamepad later

        //insert later with constants: public static final double TRACKWIDTH = ;
        //insert later with constants: public static final double CENTER_WHEEL_OFFSET = ;
        //insert later with constants: public static final double WHEEL_DIAMETER = ;
        //insert later with constants: public static final double TICKS_PER_REV = ; -- are we also using encoders?
        //insert later with constants: public static final double DISTANCE_PER_PULSE = ;

        //pick up on odometry later

        mecanumdrive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRotation2d().getDegrees()
        )

    }

}
