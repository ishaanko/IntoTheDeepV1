package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manual - Blue Close (No Hang Auto Down)", group = "Match")
public class Manual_RedNear extends LinearOpMode {
    public String workingMode = "rednear";
    public String robotType = RobotDataBase.defaultType;
    public boolean parkingCenter = true;
    public boolean autoMode = false;
    public boolean useCamera = false;
    public boolean debugMode = false;

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

        job.init();
        job.robotData.runNearTeleOpEnd = true;

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

