package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Manual - Blue Close No Reverse", group = "Match")
public class Manual_BuleNearNoReverse extends LinearOpMode {
    public String workingMode = "bluenear";
    public String robotType = RobotDataBase.defaultType;
    public boolean parkingCenter = true;
    public boolean autoMode = false;
    public boolean useCamera = false;
    public boolean debugMode = false;
    public boolean hangFarSide = true;

    @Override
    public void runOpMode() throws InterruptedException {
        GetOurJobDone job = new GetOurJobDone(this.hardwareMap);
        job.op = this;
        job.workingMode = workingMode;
        job.robotType = robotType;
        job.parkingCenter = parkingCenter;
        job.autoMode = autoMode;
        job.useCamera = useCamera;
        job.debugMode = debugMode;
        job.hangFarSide = hangFarSide;

        job.init();
        job.robotData.reverseForwardBack = false;

        waitForStart();

        if (isStopRequested()) return;

        job.initAfterStart();

        while (opModeIsActive()) {
            job.runAfterStart();
            sleep(RobotDataBase.sleepMainThreadMilliSeconds);
        }

        job.stop();
    }
}


