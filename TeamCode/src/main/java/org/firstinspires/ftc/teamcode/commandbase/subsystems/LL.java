package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LL implements Subsystem {

    public static final LL INSTANCE = new LL();
    private LL() {}
    public static double targetVel; //calculate periodically using Atag info

    double distance;


    Limelight3A limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

    limelight.pipelineSwitch(0);
    limelight.start();


    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
    LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
        Pose3D botPose = llResult.getBotpose_MT2();
//        telemetry.addData("Tx", llResult.getTx());
//        telemetry.addData("Ty", llResult.getTy());
//        telemetry.addData("Ta", llResult.getTa());
//        telemetry.addData("Yaw", botPose.getOrientation().getYaw());
        distance = TagDistance(llResult.getTa());
//        telemetry.addData("Distance", distance);
        double Kp = 0.01;
        double adjustment = -1 * Kp * llResult.getTx();

    }
}

    public double TagDistance(double ta) {
        double scale = 30665.95;
        double distance = (scale/ta);
        return distance;
    }


}
