package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


public class Odo implements Subsystem {

    public static final Odo INSTANCE = new Odo();
    private Odo() {}
    
    private GoBildaPinpointDriver pinpoint;
    
    public void initialize() {
        // TODO: Initialize pinpoint driver from hardware map when hardware is configured
        // pinpoint = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "pinpoint");
    }
    
    public void periodic() {
        // TODO: Update odometry readings
        // if (pinpoint != null) {
        //     pinpoint.update();
        // }
    }

}
