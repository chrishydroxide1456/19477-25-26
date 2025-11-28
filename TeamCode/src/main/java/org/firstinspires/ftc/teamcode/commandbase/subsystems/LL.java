package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.subsystems.Subsystem;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LL implements Subsystem {

    public static final LL INSTANCE = new LL();

    private LL() {
    }

    public static double targetVel; //just initializing these values
    public static double headingAdjust;
    public static double distance;
    private GoBildaPinpointDriver pinpoint; //use later for auto
    public static int ID;

    Limelight3A limelight;

    public void initialize(HardwareMap hardwareMap) {
        ID = 0;
        headingAdjust = 0.0;
        distance = 0.0;
        targetVel = 0.0;

        //limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void setID() {
        LLResult llresult = limelight.getLatestResult(); //concerns with motif apriltags
        if (!llresult.getFiducialResults().isEmpty()) {
            ID = llresult.getFiducialResults().get(0).getFiducialId();
        }
    }

    public void adjust() {
        LLResult llresult = limelight.getLatestResult();
        if (!llresult.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fidresult = llresult.getFiducialResults().get(0); //fiducialresult index is determined by Ta, should be fine
            if (fidresult.getFiducialId() == ID) {
                headingAdjust = (llresult.getTx()) * Math.PI / 180;
                distance = -11.5 + (29.502 - 11.5) / (Math.tan((Math.toRadians(100 + llresult.getTy())))) + 2.5; //29.502 = atag, 11.5 = ll, 2.5 = dist btwn ll and exit point
                targetVel = gettargetVel(distance); //i hope this works
            }
        }

    }

    public double gettargetVel(double Distance) {
        double Hgoal = 47; //units are inches
        double Hshooter = 14.5;
        double launchangle = 50 * Math.PI / 180;
        double flywheelR = 2;

        double Vball = Math.sqrt((9.8 * Distance * Distance) / (2 * Math.cos(launchangle) * (Distance * Math.tan(launchangle) - (Hgoal - Hshooter))));
        double Vwheel = Vball / 0.8; //some constant, tune later
        return (Vwheel / (2 * Math.PI * flywheelR));
    }


}