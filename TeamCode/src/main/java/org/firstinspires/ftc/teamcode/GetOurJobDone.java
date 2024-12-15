package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import java.util.Arrays;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class GetOurJobDone {
    public String workingMode = "rednear";
    public String robotType = "arm";
    public boolean parkingCenter = true;
    public RobotDataBase robotData = null;
    public boolean autoMode = false;
    public boolean useCamera = false;
    public boolean debugMode = false;
    public boolean hangFarSide = false;
    public LinearOpMode op;

    public double inchesOneSquare = 24;
    public DcMotorEx _fl, _fr, _rl, _rr;

    public boolean logMode = false;
    public boolean firstTime = true;
    public boolean pwmEnable = true;

    public ElapsedTime recentActionTime = new ElapsedTime();
    public boolean enablePad1Control = false;
    public double ratioPad2WheelSpeed = 0.0; //pad2 can control wheel at the ratio speed of pad1, 0 means pad2 stick can't wheels, 1 means same as pad1

    public double wheelTurnSpeed = 2.0;

    public boolean prepareForManual = true;
    public boolean autoE2E = false;
    public int targetFrontLeft = 0, targetFrontRight = 0, targetRearLeft = 0, targetRearRight = 0;
    public String[][][] markDriveData;
    public int actionCount = 0;

    public ElapsedTime moreTimeToStart = new ElapsedTime();
    public ElapsedTime timeSinceStart = new ElapsedTime();
    public boolean stopPresetAction = false;
    public boolean cameraStreaming = true;
    public boolean hangReady = false;

    public String gotoLabelString = "";

    ThreadWheel threadWheel = null;
    ThreadArm threadArm = null;
    Thread tWheel = null;
    Thread tArm = null;

    private ArrayList<String> recordActionsTimeName = new ArrayList<>();
    private ArrayList<Double> recordActionsTimeStart = new ArrayList<>();
    private ArrayList<Double> recordActionsTimeEnd = new ArrayList<>();

    public boolean distanceWheelOperationNow = false;

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public boolean runAfterManual10seconds = false;
    public boolean runAfterManual90seconds = false;
    public boolean runAfterManual100seconds = false;

    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public MecanumDrive drive;

    public GetOurJobDone(HardwareMap hardwareMap) {
    }

    public void init() throws InterruptedException {
        if (useCamera) {
        }

        if (robotType.equals("arm")) {
            robotData = new RobotDataArm(autoMode, workingMode, debugMode);
        }
        else {
            robotData = new RobotDataSwiper(autoMode, workingMode, debugMode);
        }

        if (debugMode) {
            robotData.debugReplayActionListWithSleep = true;
        }

        if (workingMode.contains("rednear")) {
            markDriveData = robotData.markRedNear;
        } else if (workingMode.contains("redfar")) {
            markDriveData = robotData.markRedFar;
        } else if (workingMode.contains("bluenear")) {
            markDriveData = robotData.markBlueNear;
        } else if (workingMode.contains("bluefar")) {
            markDriveData = robotData.markBlueFar;
        }

        if (robotData.useDeadWheel) {
            drive = new MecanumDrive(op.hardwareMap, new Pose2d(0, 0, 0));
            drive.updatePoseEstimate();
            robotData.InitRRActions(drive);
        }

        _fl = op.hardwareMap.get(DcMotorEx.class, "frontleft");
        _fr = op.hardwareMap.get(DcMotorEx.class, "frontright");
        _rl = op.hardwareMap.get(DcMotorEx.class, "rearleft");
        _rr = op.hardwareMap.get(DcMotorEx.class, "rearright");

        motors = Arrays.asList(_fl, _rl, _rr, _fr);

        for (DcMotorEx motor : motors) {
            if (robotData.useDeadWheel) {
                // RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        if (robotData.useDeadWheel) {
            // had been set in MecanumDrive class
        }
        else {
            // Reverse the right side motors
            _fl.setDirection(DcMotor.Direction.REVERSE);
            _rl.setDirection(DcMotor.Direction.REVERSE);
            _fr.setDirection(DcMotor.Direction.FORWARD);
            _rr.setDirection(DcMotor.Direction.FORWARD);
        }

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        Integer index = 0;

        try {
            for (index = 0; index < robotData.servolist.length; index++) {
                robotData.servolist[index].servo = op.hardwareMap.get(Servo.class, robotData.servolist[index].name);
            }
        }
        catch (Exception e) {
            for (; index < robotData.motorlist.length; index++) {
                // servo init failed due to lost connection of expansion hub or wrong name, all remaining servos will not be matched later
                robotData.servolist[index].name = "";
                robotData.servolist[index].log = false;
            }
            op.telemetry.addData("servolist: ", "init failed, %s", e.getMessage());
        }

        try {
            for (index = 0; index < robotData.crservolist.length; index++) {
                robotData.crservolist[index].crservo = op.hardwareMap.get(CRServo.class, robotData.crservolist[index].name);
                robotData.crservolist[index].crservo.setDirection(robotData.crservolist[index].forward ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
            }
        }
        catch (Exception e) {
            for (; index < robotData.crservolist.length; index++) {
                robotData.crservolist[index].name = "";
                robotData.crservolist[index].log = false;
            }
            op.telemetry.addData("crservolist: ", "init failed, %s", e.getMessage());
        }

        try {
            for (index = 0; index < robotData.motorlist.length; index++) {
                robotData.motorlist[index].motor = op.hardwareMap.get(DcMotorEx.class, robotData.motorlist[index].name);
                robotData.motorlist[index].motor.setDirection(robotData.motorlist[index].forward ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
                robotData.motorlist[index].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robotData.motorlist[index].motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, robotData.motorlist[index].pidf);
                robotData.motorlist[index].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robotData.motorlist[index].initialposition = robotData.motorlist[index].motor.getCurrentPosition();
            }
        }
        catch (Exception e) {
            for (; index < robotData.motorlist.length; index++) {
                robotData.motorlist[index].name = "";
                robotData.motorlist[index].log = false;
            }
            op.telemetry.addData("motorlist: ", "init failed, %s", e.getMessage());
        }

        try {
            for (index = 0; index < robotData.colorsensorlist.length; index++) {
                robotData.colorsensorlist[index].sensor = op.hardwareMap.get(NormalizedColorSensor.class, robotData.colorsensorlist[index].name);
            }
        }
        catch (Exception e) {
            for (; index < robotData.colorsensorlist.length; index++) {
                // servo init failed due to lost connection of expansion hub or wrong name, all remaining servos will not be matched later
                robotData.colorsensorlist[index].name = "";
                robotData.colorsensorlist[index].log = false;
            }
            op.telemetry.addData("colorsensorlist: ", "init failed, %s", e.getMessage());
        }

        try {
            for (index = 0; index < robotData.distancesensorlist.length; index++) {
                robotData.distancesensorlist[index].sensor = op.hardwareMap.get(DistanceSensor.class, robotData.distancesensorlist[index].name);
            }
        }
        catch (Exception e) {
            for (; index < robotData.distancesensorlist.length; index++) {
                // servo init failed due to lost connection of expansion hub or wrong name, all remaining servos will not be matched later
                robotData.distancesensorlist[index].name = "";
                robotData.distancesensorlist[index].log = false;
            }
            op.telemetry.addData("distancesensorlist: ", "init failed, %s", e.getMessage());
        }

        if (autoMode) {
            replayActionList(robotData.presetActionsAutoModeInitBeforeStart);
        }
        else {
            replayActionList(robotData.presetActionsManualModeInitBeforeStart);
        }

        op.telemetry.addData("status: ", "init completed, ready for start.");
        op.telemetry.addData("setting", " odometry wheel used = %s", robotData.useDeadWheel ? "yes" : "no");
        op.telemetry.update();
    }

    public void initAfterStart() throws InterruptedException {
        timeSinceStart.reset();

        if (autoMode == false) {
            threadWheel = new ThreadWheel();
            threadArm = new ThreadArm();
            tWheel = new Thread(threadWheel);
            tArm = new Thread(threadArm);
            replayActionList(robotData.presetActionsInitAfterStartNonAutoMode);
        }

        if (robotData.runAfterAuto) {
            replayActionList(robotData.presetActionsManualModeInitAfterAuto);
        }
    }

    private ElapsedTime lastMainActionTime = new ElapsedTime();
    private String lastMainActionName = "";

    public void runAfterStart() throws InterruptedException {
        if (firstTime) {
            timeSinceStart.reset();
            firstTime = false;
            if (robotData.useDeadWheel) {
            }
            else {
                targetFrontLeft = _fl.getCurrentPosition();
                targetFrontRight = _fr.getCurrentPosition();
                targetRearLeft = _rl.getCurrentPosition();
                targetRearRight = _rr.getCurrentPosition();
            }
            if (autoMode) {
                if (workingMode.equals("rednear")) {
                    replayActionList(robotData.presetActionsRedNear);
                } else if (workingMode.equals("redfar")) {
                    replayActionList(robotData.presetActionsRedFar);
                } else if (workingMode.equals("bluenear")) {
                    replayActionList(robotData.presetActionsBlueNear);
                } else if (workingMode.equals("bluefar")) {
                    replayActionList(robotData.presetActionsBlueFar);
                }
            } else {
                tWheel.start();
                tArm.start();
            }
        }

        if (autoMode == true)
            return;

        if (robotData.runAfterAuto && !runAfterManual10seconds && timeSinceStart.milliseconds() > 10000) {
            runAfterManual10seconds = true;
            replayActionList(robotData.presetActionsManualAfter10Seconds);
        }
        if (robotData.runNearTeleOpEnd && !runAfterManual90seconds && timeSinceStart.milliseconds() > 90000) {
            runAfterManual90seconds = true;
            replayActionList(robotData.presetActionsManualAfter90Seconds);
        }
        if (robotData.runNearTeleOpEnd && !runAfterManual100seconds && timeSinceStart.milliseconds() > 100000) {
            runAfterManual100seconds = true;
            replayActionList(robotData.presetActionsManualAfter100Seconds);
        }

        boolean ignoreSame = false;
        if (lastMainActionTime.milliseconds() < robotData.minTimeOfTwoOperations) {
            ignoreSame = true;
        }
        if (op.gamepad1.back) {
            if (ignoreSame && lastMainActionName.equals("back"))
                return;
            replayActionList(robotData.presetActionsPad1Back);
            lastMainActionName = "back";
            lastMainActionTime.reset();
            return;
        }
        if (distanceWheelOperationNow) {
            return;
        }

        // enable for both controller
        double y=0, x=0, rx=0, denominator;
        double frontLeftPower, rearLeftPower, frontRightPower, rearRightPower, speedmultiplier;

        if (robotData.useNewManualDriveMethod) {
            // get y value from left_stick and x from right_stick
            y = op.gamepad1.left_stick_y;
            if (y == 0 || Math.abs(y) < Math.abs(op.gamepad1.left_stick_x)) {
                y = -op.gamepad1.left_stick_x;
            }
            if (op.gamepad1.left_bumper || op.gamepad1.right_bumper) {
                rx = -op.gamepad1.right_stick_x;
            }
            else {
                x = -op.gamepad1.right_stick_x;
                if (x == 0 || Math.abs(op.gamepad1.right_stick_y) > Math.abs(x)) {
                    x = -op.gamepad1.right_stick_y;
                }
            }
            x *= 0.5;
            y *= 0.5;
            rx *= 0.5;
        }
        else {
            y = op.gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
            x = -op.gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
            rx = -op.gamepad1.right_stick_x * 0.5;
        }

        if (!RobotDataBase.defaultType.equals("arm") && robotData.reverseForwardBack)
        {
            x *= -1;
            y *= -1;
            rx *= 1;
        }

        //if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0) {
        //y = gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
        //x = -gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
        //rx = -gamepad1.right_stick_x * 0.5;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        rearLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        rearRightPower = (y + x - rx) / denominator;

        speedmultiplier = -1;
        if (op.gamepad1.left_trigger > 0 && op.gamepad1.right_trigger > 0) {
            speedmultiplier = 0.2;
        }
        else if (op.gamepad1.left_trigger > 0) {
            speedmultiplier = -0.4;
        } else if (op.gamepad1.right_trigger > 0) {
            speedmultiplier = 1;
        }

        speedmultiplier *= robotData.fastWheelSpeedManual;

        if (op.gamepad1.left_bumper || op.gamepad1.left_stick_button || op.gamepad1.right_stick_button) {
            speedmultiplier *= 2;
        }

        if (distanceWheelOperationNow) {
            return;
        }
        _fl.setPower(frontLeftPower * speedmultiplier);
        _rl.setPower(rearLeftPower * speedmultiplier);
        _fr.setPower(frontRightPower * speedmultiplier);
        _rr.setPower(rearRightPower * speedmultiplier);

        if (frontLeftPower != 0 || rearLeftPower != 0 || frontRightPower != 0 || rearRightPower != 0) {
            //op.telemetry.addData("frontleft, frontright, rearleft, rearright", "%.2f, %.2f, %.2f, %.2f", frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
            //op.telemetry.update();
        }

        return;
    }

    public void stop() throws InterruptedException{
        if (autoMode == false) {
            threadWheel.stop();
            threadArm.stop();
        }
        if (useCamera && cameraStreaming) {
        }
    }

    private void logAction(String s) {

        String sAll = String.format("logAction count %d, action %s\r\n", actionCount, s);
        String sTemp = "";

        for (int i = 0; i < robotData.servolist.length; i++) {
            if (robotData.servolist[i].log) {
                sTemp = String.format("servo %s : %.3f\r\n", robotData.servolist[i].name, robotData.servolist[i].servo.getPosition());
                sAll += sTemp;
            }
        }

        for (int i = 0; i < robotData.crservolist.length; i++) {
            if (robotData.crservolist[i].log) {
                //op.telemetry.addData("crservo ", "%s : %.3f", robotData.crservolist[i].name, robotData.crservolist[i].crservo.getPower());
                sTemp = String.format("crservo %s : %.3f\r\n", robotData.crservolist[i].name, robotData.crservolist[i].crservo.getPower());
                sAll += sTemp;
            }
        }

        for (int i = 0; i < robotData.motorlist.length; i++) {
            if (robotData.motorlist[i].log) {
                //op.telemetry.addData("motor ", "%s : %d", robotData.motorlist[i].name, robotData.motorlist[i].motor.getCurrentPosition());
                sTemp = String.format("motor %s : %d, initposition = %d, power = %.5f\r\n", robotData.motorlist[i].name,
                        robotData.motorlist[i].motor.getCurrentPosition() - robotData.motorlist[i].initialposition, robotData.motorlist[i].initialposition, robotData.motorlist[i].motor.getPower());
                sAll += sTemp;
            }
        }

        for (int i = 0; i < robotData.colorsensorlist.length; i++) {
            if (robotData.colorsensorlist[i].log) {
                float[] hsvValues = new float[3];
                NormalizedRGBA colors = robotData.colorsensorlist[i].sensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                sTemp = String.format("%s : Hue %3f, Saturation %.3f Value %.3f; RGB(%.3f, %.3f, %.3f) \r\n", robotData.colorsensorlist[i].name,
                        hsvValues[0], hsvValues[1], hsvValues[2], colors.red, colors.green, colors.blue);
                sAll += sTemp;
            }
        }

        for (int i = 0; i < robotData.distancesensorlist.length; i++) {
            if (robotData.distancesensorlist[i].log) {
                sTemp = String.format("%s : %3f inchs \r\n", robotData.distancesensorlist[i].name,
                        robotData.distancesensorlist[i].sensor.getDistance(DistanceUnit.INCH));
                sAll += sTemp;
            }
        }

        if (robotData.useDeadWheel) {
            // might not be correct, use LocalizerTest to get more accurate positions;
            //sTemp = String.format("Odometry left %d, right %d, rear %d\r\n", _fl.getCurrentPosition(), _rr.getCurrentPosition(), _fr.getCurrentPosition());
            drive.updatePoseEstimate();
            sTemp = String.format("x = %.2f, y = %.2f, heading (ori/deg) = %.3f/%.3f\r\n", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble(), Math.toDegrees(drive.pose.heading.toDouble()));
            sAll += sTemp;
        }

        sTemp = String.format("manualwheel = %s, debugwithsleep = %s\r\n", distanceWheelOperationNow ? "on" : "off", robotData.debugReplayActionListWithSleep ? "on" : "off");
        sAll += sTemp;

        // show in telemetry
        for (int i = 0; i < recordActionsTimeName.size(); i++) {
            sTemp = String.format("%s : %.3f (%.3f - %.3f)\r\n", recordActionsTimeName.get(i),
                    (recordActionsTimeEnd.get(i) - recordActionsTimeStart.get(i)) / 1000.0, recordActionsTimeStart.get(i) / 1000.0, recordActionsTimeEnd.get(i) / 1000.0);
            sAll += sTemp;
        }

        op.telemetry.addData("info", sAll);
        op.telemetry.update();
    }

    private void resetToPresetPosition(int presetMode) {
        if (presetMode == 0) {
            logAction("Initial");
            //replayActions(presetActionsShoulderUp);
        }
    }

    public void resetServoPosition() {
    }


    private double frontLeftLastPower = 0;
    private double frontRightLastPower = 0;
    private double rearLeftLastPower = 0;
    private double rearRightLastPower = 0;
    private ElapsedTime lastWheelActionTime = new ElapsedTime();
    private String lastWheelActionName = "";

    public void controlWheels() throws InterruptedException {
        boolean ignoreSame = false;
        if (lastWheelActionTime.milliseconds() < robotData.minTimeOfTwoOperations) {
            ignoreSame = true;
        }
        if (op.gamepad1.a) {
            if (ignoreSame && lastWheelActionName.equals("a"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1AWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1AWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1A);
            lastWheelActionName = "a";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.b) {
            if (ignoreSame && lastWheelActionName.equals("b"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1BWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1BWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1B);
            lastWheelActionName = "b";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.x) {
            if (ignoreSame && lastWheelActionName.equals("x"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1XWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1XWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1X);
            lastWheelActionName = "x";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.y) {
            if (ignoreSame && lastWheelActionName.equals("y"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1YWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1YWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Y);
            lastWheelActionName = "y";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_left) {
            if (ignoreSame && lastWheelActionName.equals("left"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1LeftWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1LeftWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Left);
            lastWheelActionName = "left";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_right) {
            if (ignoreSame && lastWheelActionName.equals("right"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1RightWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1RightWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Right);
            lastWheelActionName = "right";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_up) {
            if (ignoreSame && lastWheelActionName.equals("up"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1UpWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1UpWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Up);
            lastWheelActionName = "up";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_down) {
            if (ignoreSame && lastWheelActionName.equals("down"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1DownWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1DownWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Down);
            lastWheelActionName = "right";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.back) {
            if (ignoreSame && lastWheelActionName.equals("back"))
                return;
            replayActionList(robotData.presetActionsPad1Back);
            lastWheelActionName = "back";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.start) {
            if (ignoreSame && lastWheelActionName.equals("start"))
                return;
            replayActionList(robotData.presetActionsPad1Start);
            lastWheelActionName = "start";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.left_trigger > 0) {
            //if (ignoreSame && lastWheelActionName.equals("left_trigger"))
            //    return;
            replayActionList(robotData.presetActionsPad1LeftTrigger);
            lastWheelActionName = "left_trigger";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.right_trigger > 0) {
            //if (ignoreSame && lastWheelActionName.equals("right_trigger"))
            //    return;
            replayActionList(robotData.presetActionsPad1RightTrigger);
            lastWheelActionName = "right_trigger";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.left_bumper) {
            //if (ignoreSame && lastWheelActionName.equals("left_bumper"))
            //    return;
            replayActionList(robotData.presetActionsPad1LeftBumper);
            lastWheelActionName = "left_bumper";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.right_bumper) {
            //if (ignoreSame && lastWheelActionName.equals("right_bumper"))
            //    return;
            replayActionList(robotData.presetActionsPad1RightBumper);
            lastWheelActionName = "right_bumper";
            lastWheelActionTime.reset();
            return;
        }
        if (debugMode && op.gamepad1.options) {
            //setLogMode(!logMode);
            return;
        }
        if (debugMode && op.gamepad1.left_stick_button && op.gamepad1.right_stick_button) {
            //resetServoPosition();
            return;
        }
    }

    private ElapsedTime lastArmActionTime = new ElapsedTime();
    private String lastArmActionName = "";

    private void controlArm() throws InterruptedException {
        boolean ignoreSame = false;
        if (lastArmActionTime.milliseconds() < robotData.minTimeOfTwoOperations) {
            ignoreSame = true;
        }

        if (op.gamepad2.x) {
            if (ignoreSame && lastArmActionName.equals("x"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2XWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2XWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2XWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2X);
            }
            lastArmActionName = "x";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.a) {
            if (ignoreSame && lastArmActionName.equals("a"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2AWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2AWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2AWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2A);
            }
            lastArmActionName = "a";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.b) {
            if (ignoreSame && lastArmActionName.equals("b"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2BWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2BWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2BWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2B);
            }
            lastArmActionName = "b";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.y) {
            if (ignoreSame && lastArmActionName.equals("y"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2YWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2YWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2YWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2Y);
            }
            lastArmActionName = "y";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_up) {
            if (ignoreSame && lastArmActionName.equals("up"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2UpWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2UpWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Up);
            }
            lastArmActionName = "up";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_down) {
            if (ignoreSame && lastArmActionName.equals("down"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2DownWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2DownWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Down);
            }
            lastArmActionName = "down";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_left) {
            if (ignoreSame && lastArmActionName.equals("left"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2LeftWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2LeftWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Left);
            }
            lastArmActionName = "left";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_right) {
            if (ignoreSame && lastArmActionName.equals("right"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2RightWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2RightWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Right);
            }
            lastArmActionName = "right";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.left_trigger > 0) {
            if (ignoreSame && lastArmActionName.equals("left_trigger"))
                return;
            replayActionList(robotData.presetActionsPad2LeftTrigger);
            lastArmActionName = "left_trigger";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.right_trigger > 0) {
            if (ignoreSame && lastArmActionName.equals("right_trigger"))
                return;
            replayActionList(robotData.presetActionsPad2RightTrigger);
            lastArmActionName = "right_trigger";
            lastArmActionTime.reset();
            return;
        }
        /*
        if (opMode.gamepad2.left_bumper) {
            if (ignoreSame && lastArmActionName.equals("left_bumper"))
                return;
            replayActionList(robotData.presetActionsPad2LeftBumper);;
            lastArmActionName = "left_bumper";
            lastArmActionTime.reset();
            return;
        }
        if (opMode.gamepad2.right_bumper) {
            if (ignoreSame && lastArmActionName.equals("right_bumper"))
                return;
            replayActionList(robotData.presetActionsPad2RightBumper);;
            lastArmActionName = "right_bumper";
            lastArmActionTime.reset();
            return;
        }
        */
        if (op.gamepad2.left_stick_button) {
            if (ignoreSame && lastArmActionName.equals("left_stick"))
                return;
            replayActionList(robotData.presetActionsPad2LeftStick);
            lastArmActionName = "left_stick";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.right_stick_button) {
            if (ignoreSame && lastArmActionName.equals("right_stick"))
                return;
            replayActionList(robotData.presetActionsPad2RightStick);
            lastArmActionName = "right_stick";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.back) {
            if (ignoreSame && lastArmActionName.equals("back"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2BackWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2BackWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Back);
            }
            lastArmActionName = "back";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.start) {
            if (ignoreSame && lastArmActionName.equals("start"))
                return;
            replayActionList(robotData.presetActionsPad2Start);
            lastArmActionName = "start";
            lastArmActionTime.reset();
            return;
        }
    }

    private void playOneStepAction(String actionName, boolean ignoreRecent) {
        if (ignoreRecent) {
            if (recentActionTime.milliseconds() < robotData.minTimeOfTwoOperations) {
                // too close to last action, ignore it
                return;
            } else {
                recentActionTime.reset();
            }
        }
        logAction(actionName);
    }


    public boolean replayAction(String s) throws InterruptedException {
        actionCount = actionCount + 1;
        //
        // explain of "\\s+"
        // \\s: This is a shorthand character class in regular expressions that matches any whitespace character. This includes:
        // Spaces ( ), Tabs (\t), Newline characters (\n), Carriage returns (\r), Form feeds (\f)
        // +: This is a quantifier that matches one or more of the preceding element. In this case, it means “one or more whitespace characters.”
        //
        String[] splitStrings = s.split("\\s+");
        for (int k = 0; k < splitStrings.length; k++) {
            // if there is a variable like %%arg1=10 and not being replaced, will remove the "%%arg1=" to get the default the value
            if (splitStrings[k].startsWith("%%arg")) {
                splitStrings[k] = splitStrings[k].substring(7); // size of "%%arg1="
            }
            // remove the '@' if there is one at the beginning
            if (splitStrings[k].length() > 0 && splitStrings[k].charAt(0) == '@') {
                splitStrings[k] = splitStrings[k].substring(1);
            }
        }
        if (splitStrings.length == 0) {
            return true;
        }

        if (splitStrings[0].contains("sleep")) {
            int repeatTimes = 1;
            repeatTimes = getIntegerByGeneralStringParams(splitStrings[1]);
            waitElapsedTime(repeatTimes);
        }
        else if (splitStrings[0].equals("log")) {
            logAction(splitStrings.length > 1 ? splitStrings[1] : "status ");
        }
        else if (splitStrings[0].equals("nextstep") || splitStrings[0].equals("call")) {
            nextStep(splitStrings);
        }
        else if (splitStrings[0].equals("motor")) {
            motorgeneral(splitStrings);
        }
        else if (splitStrings[0].equals("servo")) {
            servogeneral(splitStrings);
        }
        else if (splitStrings[0].equals("crservo")) {
            crservogeneral(splitStrings);
        }
        else if (splitStrings[0].equals("manualwheel")) {
            manualWheel(splitStrings);
        }
        else if (splitStrings[0].equals("thread")) {
            replayInNewThread(splitStrings);
        }
        else if (splitStrings[0].equals("rr")) {
            rr(splitStrings);
        }
        else if (splitStrings[0].equals("exit")) {
            if (splitStrings.length >= 2 && splitStrings[1].equals("true"))
                return true;
            else
                return false;
        }
        else if (splitStrings[0].equals("set")) {
            // not return here if want to continue the next operations like drop the pixels automatically
            setParameters(splitStrings);
        }
        else if (splitStrings[0].equals("if")) {
            return checkParameters(splitStrings);
        }
        else {
        }
        return true;
    }

    private HashMap<String, String> customParamsMap = new HashMap<String, String>();
    private HashMap<String, OneInstance> oneInstanceMap = new HashMap<>();

    public void setParameters(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length < 3) {
            return;
        }
        if (splitStrings[1].equals("drivespeed")) {
            double speed = 1.0;
            if (splitStrings[2].equals("default")) {
                speed = 1.0;
            }
            else {
                speed = Double.parseDouble(splitStrings[2]);
            }
            robotData.fastWheelSpeedManual = speed;
        }
        else if (splitStrings[1].equals("scriptstopable")) {
            boolean stopable = false;
            if (splitStrings[2].equals("1") || splitStrings[2].equals("yes")) {
                stopable = true;
            }
            else if (splitStrings[2].equals("0") || splitStrings[2].equals("no")) {
                stopable = false;
            }
            robotData.scriptStopable = stopable;
        }
        else if (splitStrings[1].equals("armpower")) {
            if (splitStrings[2].equals("off")) {
                //ServoController controller = _arm.getController();
                //if (controller.getPwmStatus() != ServoController.PwmStatus.DISABLED) {
                //    controller.pwmDisable();
                //}
            }
            else if (splitStrings[2].equals("on")) {
                //ServoController controller = _arm.getController();
                //controller.pwmEnable();
            }
        }
        else if (splitStrings[1].equals("lifterarmpower")) {
            if (splitStrings[2].equals("off")) {
                //ServoController controller = _lifterarm.getController();
                //if (controller.getPwmStatus() != ServoController.PwmStatus.DISABLED) {
                //    controller.pwmDisable();
                //}
            }
            else if (splitStrings[2].equals("on")) {
                //ServoController controller = _lifterarm.getController();
                //controller.pwmEnable();
            }
        }
        else if (splitStrings[1].equals("hangready")) {
            if (splitStrings[2].equals("on")) {
                hangReady = true;
            }
            else if (splitStrings[2].equals("off")) {
                hangReady = false;
            }
        }
        else if (splitStrings[1].equals("kp") || splitStrings[1].equals("ki") || splitStrings[1].equals("kd") || splitStrings[1].equals("kf") ) {
            // seems not working, tbd
            // setKPIDFGeneral(splitStrings);
        }
        else if (splitStrings[1].equals("debug")) {
            if (splitStrings[2].equals("on")) {
                robotData.debugReplayActionListWithSleep = true;
            }
            else if (splitStrings[2].equals("off")) {
                robotData.debugReplayActionListWithSleep = false;
            }
        }
        else if (splitStrings[1].equals("recordactionstime")) {
            if (splitStrings[2].equals("off")) {
                robotData.recordActionsTime = false;
            }
            else if (splitStrings[2].equals("on")) {
                // record action time will only work in one script currently
                robotData.recordActionsTime = true;
                // reset the list;
                recordActionsTimeName.clear();
                recordActionsTimeStart.clear();
                recordActionsTimeEnd.clear();
            }
        }
        else if (splitStrings[1].equals("custom")) {
            // create custom set parameters
            customParamsMap.put(splitStrings[2], splitStrings[3]);
        }
        else if (splitStrings[1].equals("oneinstance")) {
            // set oneinstance abc 2000 // when checking the name of abc and within range of 2000ms, it will return true
            double tLength = getDoubleByGeneralStringParams(splitStrings[3]);
            double tBegin = timeSinceStart.milliseconds();
            oneInstanceMap.put(splitStrings[2], new OneInstance(tLength, tBegin));
        }
    }


    //
    // Not working currently
    //
    public void setKPIDFGeneral(String[] splitStrings) throws InterruptedException {

        int i = getMotorGeneralIndex(splitStrings[2]);
        if (i < 0)
            return;

        boolean delta = false;
        if (splitStrings.length >= 5 && splitStrings[4].equals("delta"))
            delta = true;

        double value = Double.parseDouble(splitStrings[3]);

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients  pidfOrig = robotData.motorlist[i].motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients  pidfModified = new PIDFCoefficients(pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);

        if (splitStrings[1].equals("kp")) {
            if (delta)
                pidfModified.p += value;
            else
                pidfModified.p = value;
        } else if (splitStrings[1].equals("ki")) {
            if (delta)
                pidfModified.i += value;
            else
                pidfModified.i = value;
        } else if (splitStrings[1].equals("kd")) {
            if (delta)
                pidfModified.d += value;
            else
                pidfModified.d = value;
        } else if (splitStrings[1].equals("kf")) {
            if (delta)
                pidfModified.f += value;
            else
                pidfModified.f = value;
        } else
            return;

        // change coefficients using methods included with DcMotorEx class.
        robotData.motorlist[i].motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfModified);

        // re-read coefficients and verify change.
        PIDFCoefficients pidfNew = robotData.motorlist[i].motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("Motor", "%s", splitStrings[2]);
        op.telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f", pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
        op.telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f", pidfNew.p, pidfNew.i, pidfNew.d, pidfNew.f);
        op.telemetry.update();
    }


    public int getServoGeneralIndex(String name) {
        for (int i = 0; i < robotData.servolist.length; i++) {
            if (name.equals(robotData.servolist[i].name)) {
                return i;
            }
        }
        return -1;
    }

    public int getMotorGeneralIndex(String name) {
        for (int i = 0; i < robotData.motorlist.length; i++) {
            if (name.equals(robotData.motorlist[i].name)) {
                return i;
            }
        }
        return -1;
    }

    public int getColorSensorGeneralIndex(String name) {
        for (int i = 0; i < robotData.colorsensorlist.length; i++) {
            if (name.equals(robotData.colorsensorlist[i].name)) {
                return i;
            }
        }
        return -1;
    }

    public int getDistanceSensorGeneralIndex(String name) {
        for (int i = 0; i < robotData.distancesensorlist.length; i++) {
            if (name.equals(robotData.distancesensorlist[i].name)) {
                return i;
            }
        }
        return -1;
    }

    public int getIntegerByGeneralStringParams(String value) throws InterruptedException {
        int result = 0;
        if (robotData.generalStringParamsMap.containsKey(value)) {
            value = robotData.generalStringParamsMap.get(value);
        }
        result = Integer.parseInt(value);
        return result;
    }

    public double getDoubleByGeneralStringParams(String value) throws InterruptedException {
        double result = 0.0;
        if (robotData.generalStringParamsMap.containsKey(value)) {
            value = robotData.generalStringParamsMap.get(value);
        }
        result = Double.parseDouble(value);
        return result;
    }

    public int getValueByMotorGeneralIndex(String value, int i) throws InterruptedException {
        int result = 0;
        if (value.equals("max")) {
            result = robotData.motorlist[i].posmax;
        }
        else if (value.equals("min")) {
            result = robotData.motorlist[i].posmin;
        }
        else if (robotData.motorlist[i].map.containsKey(value)) {
            result = robotData.motorlist[i].map.get(value);
        }
        else {
            result = getIntegerByGeneralStringParams(value);
        }
        return result;
    }

    public double getValueByServoGeneralIndex(String value, int i) throws InterruptedException {
        double result = 0;
        if (value.equals("min")) {
            result = robotData.servolist[i].posmin;
        }
        else if (value.equals("max")) {
            result = robotData.servolist[i].posmax;
        }
        else {
            if (robotData.servolist[i].map.containsKey(value)) {
                result = robotData.servolist[i].map.get(value);
            }
            else {
                result = getDoubleByGeneralStringParams(value);
            }
        }
        return result;
    }

    public Servo getServoByName(String name) {
        int i = getServoGeneralIndex(name);
        if (i >= 0)
            return robotData.servolist[i].servo;

        // search the specific servo if not defined in the general servo list
        return null;
    }

    public DcMotorEx getMotorByName(String name) {
        int i = getMotorGeneralIndex(name);
        if (i >= 0)
            return robotData.motorlist[i].motor;

        // search the specific motor if not defined in the general motor list
        return null;
    }

    public static <T extends Number & Comparable<T>> boolean generalCompare(String operator, T currentValue, T value) {
        boolean result = false;
        if (operator.equals(">") || operator.equals("gt")) {
            result = currentValue.compareTo(value) > 0;
        }
        else if (operator.equals(">=") || operator.equals("ge")) {
            result = currentValue.compareTo(value) >= 0;
        }
        else if (operator.equals("==") || operator.equals("=") || operator.equals("eq")) {
            result = currentValue.compareTo(value) == 0;
        }
        else if (operator.equals("!=")) {
            result = currentValue.compareTo(value) != 0;
        }
        else if (operator.equals("<=") || operator.equals("le")) {
            result = currentValue.compareTo(value) <= 0;
        }
        else if (operator.equals("<") || operator.equals("lt")) {
            result = currentValue.compareTo(value) < 0;
        }
        return result;
    }

    public boolean checkParameters(String[] splitStrings) throws InterruptedException {
        if (robotData.debugReplayActionListWithSleep) {
            for (int i=0; i<splitStrings.length; i++) {
                op.telemetry.addData("arg", "%d %s", i, splitStrings[i]);
            }
            op.telemetry.update();
            sleep(2000);
        }

        if (splitStrings.length < 3) {
            return false;
        }

        boolean result = true;
        // "check motor motorname1 <= 100"
        // "check servo servoname1 > 0.1"
        if (splitStrings[1].equals("motor")) {
            int i = getMotorGeneralIndex(splitStrings[2]);
            if (i >= 0) {
                int value = getValueByMotorGeneralIndex(splitStrings[4], i);
                int currentValue = robotData.motorlist[i].motor.getCurrentPosition() - robotData.motorlist[i].initialposition;
                result = generalCompare(splitStrings[3], currentValue, value);
            }
        }
        else if (splitStrings[1].equals("servo")) {
            // check servo position value
            // check servo position value
            int i = getServoGeneralIndex(splitStrings[2]);
            if (i >= 0) {
                double value = getValueByServoGeneralIndex(splitStrings[4], i);
                double currentValue = robotData.servolist[i].servo.getPosition();
                result = generalCompare(splitStrings[3], currentValue, value);
            }
        }
        else if (splitStrings[1].equals("colorsensor")) {
            int i = getColorSensorGeneralIndex(splitStrings[2]);
            if (i >= 0) {
                String scolor = "empty";
                float[] hsvValues = new float[3];
                NormalizedRGBA colors = robotData.colorsensorlist[i].sensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                if (hsvValues[0] >= 181) {
                    scolor = "blue";
                }
                else if (hsvValues[0] >=60 && hsvValues[0] < 181) {
                    scolor = "yellow";
                    if (hsvValues[0] == 60) {
                        // red and yellow might be same as 60 for Hue value
                        if (hsvValues[1] >= 0.5 && colors.green > colors.red) {
                            scolor = "yellow";
                        }
                        else {
                            scolor = "red";
                        }
                    }
                }
                else if (hsvValues[0] >= 0 && hsvValues[0] < 60) {
                    if (hsvValues[0] == 0) {
                        if (hsvValues[1] >= 0.5 && colors.red >= 0.006 && colors.red > colors.blue) {
                            scolor = "red";
                        }
                        else {
                            // not detected, leave it as blank
                        }
                    }
                    else
                        scolor = "red";
                }
                result = scolor.equals(splitStrings[4]);
                if (splitStrings[3].equals("!=")) {
                    result = !result;
                }
            }
        }
        else if (splitStrings[1].equals("custom")) {
            if (customParamsMap.containsKey(splitStrings[2])) {
                // custom only support string, operator only support "==" by far
                if (customParamsMap.get(splitStrings[2]).equals(splitStrings[4])) {
                    result = true;
                }
            }
            else {
                result = false;
            }
        }
        else if (splitStrings[1].equals("oneinstance")) {
            // if oneinstance abc == off // if haven't set a timer with name abc, it will return true;
            // custom only support string, operator only support "==" by far
            if (oneInstanceMap.containsKey(splitStrings[2])) {
                OneInstance t = oneInstanceMap.get(splitStrings[2]);
                if (timeSinceStart.milliseconds() - t.tBegin < t.tLength) {
                    // still in the timer range
                    result = true;
                }
                else {
                    result = false;
                }
            }
            else {
                result = false;
            }
            if (splitStrings[4].equals("off")) {
                result = !result;
            }
        }
        else if (splitStrings[1].equals("key")) {
            // if key pad1leftbumper == on else goto pad1bumperoff
            // key only support "on"/"off", operator only support "==" by far
            if (splitStrings[2].equals("pad1leftbumper")) {
                result = op.gamepad1.left_bumper;
            }
            else if (splitStrings[2].equals("pad1rightbumper")) {
                result = op.gamepad1.right_bumper;
            }
            else if (splitStrings[2].equals("pad1lefttrigger")) {
                result = op.gamepad1.left_trigger > 0.95;
            }
            else if (splitStrings[2].equals("pad1righttrigger")) {
                result = op.gamepad1.right_trigger > 0.95;
            }
            else if (splitStrings[2].equals("pad2leftbumper")) {
                result = op.gamepad2.left_bumper;
            }
            else if (splitStrings[2].equals("pad2rightbumper")) {
                result = op.gamepad2.right_bumper;
            }
            else if (splitStrings[2].equals("pad2lefttrigger")) {
                result = op.gamepad2.left_trigger > 0.95;
            }
            else if (splitStrings[2].equals("pad2righttrigger")) {
                result = op.gamepad2.right_trigger > 0.95;
            }
            else if (splitStrings[2].equals("pad1up")) {
                result = op.gamepad1.dpad_up;
            }
            else if (splitStrings[2].equals("pad1down")) {
                result = op.gamepad1.dpad_down;
            }
            else if (splitStrings[2].equals("pad1left")) {
                result = op.gamepad1.dpad_left;
            }
            else if (splitStrings[2].equals("pad1right")) {
                result = op.gamepad1.dpad_right;
            }
            else if (splitStrings[2].equals("pad1x")) {
                result = op.gamepad1.x;
            }
            else if (splitStrings[2].equals("pad1y")) {
                result = op.gamepad1.y;
            }
            else if (splitStrings[2].equals("pad1a")) {
                result = op.gamepad1.a;
            }
            else if (splitStrings[2].equals("pad1b")) {
                result = op.gamepad1.b;
            }
            else if (splitStrings[2].equals("pad2up")) {
                result = op.gamepad2.dpad_up;
            }
            else if (splitStrings[2].equals("pad2down")) {
                result = op.gamepad2.dpad_down;
            }
            else if (splitStrings[2].equals("pad2left")) {
                result = op.gamepad2.dpad_left;
            }
            else if (splitStrings[2].equals("pad2right")) {
                result = op.gamepad2.dpad_right;
            }
            else if (splitStrings[2].equals("pad2x")) {
                result = op.gamepad2.x;
            }
            else if (splitStrings[2].equals("pad2y")) {
                result = op.gamepad2.y;
            }
            else if (splitStrings[2].equals("pad2a")) {
                result = op.gamepad2.a;
            }
            else if (splitStrings[2].equals("pad2b")) {
                result = op.gamepad2.b;
            }
            else {
                return false;
            }
            if (splitStrings[4].equals("off")) {
                result = !result;
            }
            if (splitStrings[3].equals("!=")) {
                result = !result;
            }
        }

        if (result == false && splitStrings.length >= 8 && splitStrings[5].equals("else")) {
            if (robotData.debugReplayActionListWithSleep) {
                op.telemetry.addData("result", "%d", result ? 1 : 0);
                op.telemetry.update();
                sleep(2000);
            }
            if (splitStrings[6].equals("nextstep")) {
                if (robotData.nextstepmap.containsKey(splitStrings[7])) {
                    replayActionList(robotData.nextstepmap.get(splitStrings[6]));
                    result = false;
                }
            }
            else if (splitStrings[6].equals("goto")) {
                gotoLabelString = splitStrings[7];
            }
        }
        if (robotData.debugReplayActionListWithSleep) {
            op.telemetry.addData("gotoLabelString", "%s", gotoLabelString);
            op.telemetry.update();
            sleep(2000);
        }
        return result;
    }

    public boolean replayInNewThread(String[] splitStrings) throws InterruptedException {
        boolean result = true;
        if (splitStrings.length < 2) {
            return result;
        }
        String action = "";
        // reform the action string without the thread keyword
        for (int i=1; i<splitStrings.length; i++) {
            //String temp = i == 1 ? splitStrings[i] : "@"+splitStrings[i];
            String temp = splitStrings[i];
            action = action + temp;
            action = action + " ";
        }

        if (robotData.replayInNewThreadWhenUsingThreadKeyword) {
            ThreadReplay threadReplay = null;
            Thread tReplay = null;
            threadReplay = new ThreadReplay();
            threadReplay.action = action;
            tReplay = new Thread(threadReplay);
            tReplay.start();
        }
        else {
            replayAction(action);
        }
        return result;
    }

    public void manualWheel(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length < 1)
            return;
        if (splitStrings[1].equals("enable")) {
            distanceWheelOperationNow = false;
        }
        else if (splitStrings[1].equals("disable")) {
            distanceWheelOperationNow = true;
        }
    }

    public void crservogeneral(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length < 3)
            return;
        double power = 0, powermin = -1.0, powermax = 1.0;
        power = getDoubleByGeneralStringParams(splitStrings[2]);
        CRServo crs = null;

        boolean foundinlist = false;
        for (int i = 0; i < robotData.crservolist.length; i++) {
            if (splitStrings[1].equals(robotData.crservolist[i].name)) {
                foundinlist = true;
                crs = robotData.crservolist[i].crservo;
                powermin = robotData.crservolist[i].powermin;
                powermax = robotData.crservolist[i].powermax;
                break;
            }
        }

        if (!foundinlist) {
            //if (splitStrings[1].equals("rollerintake")) {
            //    crs = _rollerintake;
            //} else {
            //    return;
            //}
        }

        if (power > powermax)
            power = powermax;
        else if (power < powermin)
            power = powermin;

        crs.setPower(power);

        if (splitStrings.length >= 4) {

            int t = Integer.parseInt(splitStrings[3]);
            waitElapsedTime(t);

            // stop power
            crs.setPower(0);
        }

        op.telemetry.addData("crservo ", "%s : %f", splitStrings[1], power);
        op.telemetry.update();
        return;
    }

    public void motorgeneral(String[] splitStrings) throws InterruptedException {
        /*
        if (robotData.debugReplayActionListWithSleep) {
            for (int i=0; i<splitStrings.length; i++) {
                op.telemetry.addData("arg", "%d %s", i, splitStrings[i]);
            }
            op.telemetry.update();
            sleep(2000);
        }
         */
        if (splitStrings.length < 3)
            return;
        double power = 0, powermin = -1.0, powermax = 1.0;
        DcMotor motor = null;

        int i = getMotorGeneralIndex(splitStrings[1]);
        if (i >= 0) {
            motor = robotData.motorlist[i].motor;
            powermin = robotData.motorlist[i].powermin;
            powermax = robotData.motorlist[i].powermax;
        }
        else {
            if (splitStrings[1].equals("frontleft")) {
                motor = _fl;
            } else if (splitStrings[1].equals("frontright")) {
                motor = _fr;
            } else if (splitStrings[1].equals("rearleft")) {
                motor = _rl;
            } else if (splitStrings[1].equals("rearright")) {
                motor = _rr;
            } else {
                return;
            }
        }

        if (splitStrings[2].equals("initposition") && i >= 0) {
            robotData.motorlist[i].initialposition = robotData.motorlist[i].motor.getCurrentPosition();
            return;
        }

        power = getDoubleByGeneralStringParams(splitStrings[2]);
        /*
        if (power > powermax)
            power = powermax;
        else if (power < powermin)
            power = powermin;
         */
        if (splitStrings.length >= 4) {

            if (splitStrings[3].equals("runtoposition") && i >= 0) {
                int pos = getValueByMotorGeneralIndex(splitStrings[4], i);
                pos += robotData.motorlist[i].initialposition;
                int maxTimer = 5000;
                if (splitStrings.length >= 6) {
                    maxTimer = getIntegerByGeneralStringParams(splitStrings[5]);
                }
                boolean bhold = false;
                if (splitStrings.length >= 7) {
                    if (splitStrings[6].equals("hold"))
                        bhold = true;
                }
                //op.telemetry.addData("motor ", "%s , %f, runtopos %d", splitStrings[1], power, pos);
                //op.telemetry.update();
                //motorRunToPositionWithSoftStop(motor, pos, power, 5000);
                motorRunToPosition(motor, pos, power, maxTimer, bhold);
            }
            else {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setPower(power);
                // run for some time;
                int t = getIntegerByGeneralStringParams(splitStrings[3]);
                waitElapsedTime(t);
                // stop power
                motor.setPower(0);
            }
        }
        else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(power);
            //op.telemetry.addData("motor ", "%s , %f", splitStrings[1], power);
            //op.telemetry.update();
        }

        return;
    }

    public void servoset(Servo sv, double pos, boolean delta, ServoInfo info) {
        if (delta) {
            pos = sv.getPosition() + pos;
        }
        if (pos >= info.posmax)
            pos = info.posmax;
        else if (pos <= info.posmin)
            pos = info.posmin;
        sv.setPosition(pos);
    }

    public void servogeneral(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length < 3)
            return;

        Servo sv;
        ServoInfo info = new ServoInfo();
        boolean delta = false;

        if (splitStrings.length >= 4 && splitStrings[3].equals("delta"))
            delta = true;

        int i = getServoGeneralIndex(splitStrings[1]);
        if (i >= 0) {
            double pos = getValueByServoGeneralIndex(splitStrings[2], i);
            servoset(robotData.servolist[i].servo, pos, delta, robotData.servolist[i]);
            op.telemetry.addData("servo ", "%s : %f", splitStrings[1], pos);
            op.telemetry.update();
        }
        else {
            // add special handle if the servo is not defined in the servo list
            return;
        }
        return;
    }

    public void motorRunToPosition(DcMotor motor, int destPosition, double powerIn, int maxWaitMilliSeconds, boolean bhold) throws InterruptedException {
        double power = powerIn;
        ElapsedTime     motorRunTime = new ElapsedTime();

        double holdPower = 0.0005;
        //if (robotData.generalStringParamsMap.containsKey("sliderpowerhold")) {
        //    String value = robotData.generalStringParamsMap.get("sliderpowerhold");
        //    holdPower = Double.parseDouble(value);
        //}

        motor.setTargetPosition(destPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power); // Set power to start the motor

        while ((motorRunTime.milliseconds() < maxWaitMilliSeconds) && motor.isBusy()) {
            /*
            if ((motorRunTime.milliseconds() > (maxWaitMilliSeconds / 4)) && motor.getPower() <= 0.001) {
                op.telemetry.addData("[power < 0.001] Target Position", destPosition);
                op.telemetry.addData("Current Position", motor.getCurrentPosition());
                op.telemetry.update();
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return;
            }
            */
            op.telemetry.addData("Target Position", destPosition);
            op.telemetry.addData("Current Position", motor.getCurrentPosition());
            op.telemetry.update();
            sleep(1); // Wait sometime between updates
        }
        // Stop all motion;
        motor.setPower(bhold ? holdPower : 0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean replayActionList(String[] list) throws InterruptedException {
        String gotoLabelStringLocal = "";

        // only record time in current script
        boolean originalRecordActionsTime = robotData.recordActionsTime;
        boolean recordActionsTimeInThisScript = false;

        for (int i = 0; i < list.length; i++) {
            if (robotData.recordActionsTime && robotData.recordActionsTime != originalRecordActionsTime) {
                recordActionsTimeInThisScript = true;
            }
            // stop the script when one of the keys are pressed
            if (robotData.scriptStopable && (op.gamepad1.left_bumper  || op.gamepad1.right_bumper || op.gamepad2.left_bumper || op.gamepad2.right_bumper)) {
                robotData.scriptStopable = false;
                return false;
            }
            if (robotData.debugReplayActionListWithSleep) {
                op.telemetry.addData("replay ", "no.%d %s", i, list[i]);
                op.telemetry.update();
                sleep(2000);
            }

            if (!gotoLabelStringLocal.isEmpty()) {
                // until find exact lable
                if (list[i].startsWith(gotoLabelStringLocal)) {
                    // found
                    gotoLabelStringLocal = "";
                }
                if (robotData.debugReplayActionListWithSleep) {
                    op.telemetry.addData("gotoLabelStringLocal ", "%s %s", gotoLabelStringLocal, list[i]);
                    op.telemetry.update();
                    sleep(2000);
                }
                continue;
            }

            if (recordActionsTimeInThisScript) {
                recordActionsTimeName.add(list[i]);
                recordActionsTimeStart.add(timeSinceStart.milliseconds());
            }
            boolean result = replayAction(list[i]);
            if (recordActionsTimeInThisScript) {
                recordActionsTimeEnd.add(timeSinceStart.milliseconds());
            }
            if (!gotoLabelString.isEmpty() && (list[i].startsWith("if"))) {
                gotoLabelStringLocal = gotoLabelString;
                gotoLabelString = "";
                if (robotData.debugReplayActionListWithSleep) {
                    op.telemetry.addData("gotoLabelStringLocal ", "%s %s", gotoLabelStringLocal, list[i]);
                    op.telemetry.update();
                    sleep(2000);
                }
                continue;
            }
            if (result == false)
                return false;
        }
        return true;
    }

    boolean nextStep(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length > 2){
            // might have args, need replace the orignal one list if there is any args. currently suppport 1 or 2 args beginning with %%arg1=, or %%arg2=.
            if (robotData.nextstepmap.containsKey(splitStrings[1])) {
                String[] list = robotData.nextstepmap.get(splitStrings[1]).clone();
                for (int i=0; i<list.length; i++) {
                    if (list[i].contains("%%")) {
                        String[] sTempSplit = list[i].split("\\s+");
                        for (int k = 0; k < sTempSplit.length; k++) {
                            if (splitStrings.length > 2 && sTempSplit[k].startsWith("%%arg1=")) {
                                sTempSplit[k] = splitStrings[2];
                            }
                            if (splitStrings.length > 3 && sTempSplit[k].startsWith("%%arg2=")) { // containing two args
                                sTempSplit[k] = splitStrings[3];
                            }
                            if (splitStrings.length > 4 && sTempSplit[k].startsWith("%%arg3=")) {// contains three args
                                sTempSplit[k] = splitStrings[4];
                            }
                        }
                        // reconstruct the string
                        String action = "";
                        // reform the action string without the thread keyword
                        for (int n=0; n<sTempSplit.length; n++) {
                            action = action + sTempSplit[n];
                            action = action + " ";
                        }
                        list[i] = action;
                    }
                }
                // original strings copied and changed
                return replayActionList(list);
            }
        }
        else if (robotData.nextstepmap.containsKey(splitStrings[1])) {
            return replayActionList(robotData.nextstepmap.get(splitStrings[1]));
        }
        return false;
    }

    class ThreadWheel implements Runnable {
        private volatile boolean exitThread = false;
        public void run() {
            try {
                while (!exitThread) {
                    controlWheels();
                    sleep(1);
                }
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
                return;
            }
        }

        public void stop() {
            exitThread = true;
        }
    }

    class ThreadArm implements Runnable {
        private volatile boolean exitThread = false;
        public void run() {
            try {
                while (!exitThread) {
                    controlArm();
                    sleep(1);
                }
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
                return;
            }
        }

        public void stop() {
            exitThread = true;
        }
    }

    class ThreadReplay implements Runnable {
        public String action;
        public void run() {
            try {
                replayAction(action);
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
                return;
            }
        }
    }

    void waitElapsedTime(int waitTime) throws InterruptedException {
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < waitTime) {
            //telemetry.addData("Waiting millisecond: ", moreTimeToStart.milliseconds()  );
            //telemetry.update();
            sleep(1);
            continue;
        }
    }

    boolean rr(String[] splitStrings) throws InterruptedException {
        if (!robotData.useDeadWheel)
            return false;

        if (splitStrings[1].equals("dropbasketdynamic")) { // just test, not really working as expected
            drive.updatePoseEstimate();
            Pose2d beginPose = drive.pose;
            //drive.setDrivePowers(PoseVelocity2d );
            double xhang = 26, xgetbricks = 39, xdropbasket = 7.8; //12;
            double yhang = -32, ygetbrick1 = 4, ydropbasket = 20.2; //22;

            TrajectoryActionBuilder tab;
            Action action;
            if (splitStrings[2].equals("splineto")) {
                tab = drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(xdropbasket, ydropbasket), -Math.PI / 4);
            }
            else if (splitStrings[2].equals("splinetolinear")) {
                tab = drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4), 0);
            }
            else if (splitStrings[2].equals("strafeto")) {
                tab = drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(xdropbasket, ydropbasket))
                        .turn(-Math.PI / 4);
            }
            else if (splitStrings[2].equals("strafetolinear")) {
                tab = drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(xdropbasket, ydropbasket), -Math.PI / 4);
            } else {
                tab = drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(xdropbasket, ydropbasket), -Math.PI / 4);
            }
            action = tab.build();
            //action.preview(Canvas);
            Actions.runBlocking(action);
        }
        else if (splitStrings[1].equals("delta")) { // working well
            drive.updatePoseEstimate();
            Pose2d beginPose = drive.pose;
            double xdelta = getDoubleByGeneralStringParams(splitStrings[2]);
            double ydelta = getDoubleByGeneralStringParams(splitStrings[3]);
            double newx = drive.pose.position.x + xdelta, newy = drive.pose.position.y + ydelta;

            TrajectoryActionBuilder tab;
            Action action;
            tab = drive.actionBuilder(beginPose)
                 .strafeTo(new Vector2d(newx, newy));
            action = tab.build();
            Actions.runBlocking(action);
        }
        else if (splitStrings[1].equals("forward")) { // work too, though hard to simulate the time pressed
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-0.5, 0.0),
                    0
            ));
            waitElapsedTime(500);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0.0, 0.0),
                    0
            ));
        }
        else {
            for (int i = 0; i < robotData.rrActionsName.size(); i++) {
                if (robotData.rrActionsName.get(i).equals(splitStrings[1])) {
                    Actions.runBlocking(robotData.rrActions.get(i));
                }
            }
        }
        return true;
    }

}
