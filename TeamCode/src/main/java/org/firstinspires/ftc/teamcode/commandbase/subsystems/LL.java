package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
public class LL implements Subsystem {

    public static final LL INSTANCE = new LL();
    private LL() {}
    public static double targetVel; //calculate when needed using Atag info
    public static double headingAdjust;
    private GoBildaPinpointDriver pinpoint;
    double distance;

    Limelight3A limelight;

    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void adjust() {
        LLResult llresult = limelight.getLatestResult();;
        LLResultTypes.FiducialResult fiducialresult = llresult.getFiducialResults(); //trying to get Atag ID
        if (llresult.isValid() && llresult.getFiducialResults().equals(24)) { //need to add the enum stuff for red and blue
            headingAdjust = llresult.getTx();
            targetVel = (0.1) * 30665.95/llresult.getTa(); //need to tune later. watch coach pratt vid
        }
        else {
            headingAdjust = 0.0;

        }
    }

}



