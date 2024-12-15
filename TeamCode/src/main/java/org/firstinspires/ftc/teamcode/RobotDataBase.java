package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.HashMap;

class OneInstance {
    double tLength;
    double tBegin;
    public OneInstance(double tLength, double tBegin) {
        this.tLength = tLength;
        this.tBegin = tBegin;
    }
};

class ColorSensorInfo {
    public String name;
    public NormalizedColorSensor sensor;
    public boolean log;
    public ColorSensorInfo(String name, boolean log) {
        this.name = name;
        this.log = log;
    }
};

class DistanceSensorInfo {
    public String name;
    public DistanceSensor sensor;
    public boolean log;
    public DistanceSensorInfo(String name, boolean log) {
        this.name = name;
        this.log = log;
    }
};

class ServoInfo {
    public String name;
    public double posmin;
    public double posmax;
    public double posdefault; // default is a keyword, so use posdefault;
    public double open;
    public double close;
    public boolean log;
    HashMap<String, Double> map;
    public Servo servo;

    public ServoInfo(String name, double posmin, double posmax, double posdefault, double open, double close, boolean log) {
        this.name = name;
        this.posmin = posmin;
        this.posmax = posmax;
        this.posdefault = posdefault;
        this.open = open;
        this.close = close;
        this.log = log;
        map = new HashMap<String, Double>();
    }

    public ServoInfo(String name, boolean log, double posmin, double posmax, String str1, String str2, String str3, String str4, String str5, String str6, String str7, String str8) {
        this.name = name;
        this.posmin = posmin;
        this.posmax = posmax;
        this.log = log;
        map = new HashMap<String, Double>();

        String[] strParams = new String[] {str1, str2, str3, str4, str5, str6, str7, str8};
        for (int i = 0; i < strParams.length; i++) {
            String param = strParams[i];
            if (!param.isEmpty()) {
                int pos = param.indexOf('=');
                if (pos < 0)
                    continue;
                String key = param.substring(0, pos);
                String strvalue = param.substring(pos+1);
                Double value = Double.parseDouble(strvalue);
                map.put(key, value);
            }
        }

    }
    public ServoInfo() {
        this.name = "";
        this.posmin = 0.0;
        this.posmax = 1.0;
        this.posdefault = 0.5;
        this.open = 0.75;
        this.close = 0.25;
        this.log = false;
        map = new HashMap<String, Double>();
    }
};

class CRServoInfo {
    public String name;
    public double powermin;
    public double powermax;
    public boolean forward;
    public boolean log;
    public CRServo crservo;
    public CRServoInfo(String name, double powermin, double powermax, boolean forward, boolean log) {
        this.name = name;
        this.powermin = powermin;
        this.powermax = powermax;
        this.forward = forward;
        this.log = log;
    }
    public CRServoInfo() {
    }
};

class MotorInfo {
    public String name;
    public double powermin;
    public double powermax;
    public boolean forward;
    public boolean log; // whether show information in log
    public DcMotorEx motor;
    public int initialposition; // might change every time
    public int posmin;
    public int posmax;
    public int stepsize;
    public int possmall;
    public int posbig;
    public PIDFCoefficients pidf;
    HashMap<String, Integer> map;

    public MotorInfo(String name, int possmall, int posbig, int posmax, int stepsize, double powermin, double powermax, boolean forward, boolean log) {
        this.name = name;
        this.possmall = possmall;
        this.posbig = posbig;
        this.posmax = posmax;
        this.stepsize = stepsize;
        this.powermin = powermin;
        this.powermax = powermax;
        this.forward = forward;
        this.log = log;
        posmin = 0;
        map = new HashMap<String, Integer>();
    }

    public MotorInfo(String name, boolean forward, boolean log, int stepsize, int posmax, PIDFCoefficients pidf, String str1, String str2, String str3, String str4, String str5) {
        this.name = name;
        this.stepsize = stepsize;
        this.forward = forward;
        this.posmax = posmax;
        this.log = log;
        posmin = 0;
        this.powermin = -1.0;
        this.powermax = 1.0;
        this.pidf = pidf;
        map = new HashMap<String, Integer>();

        String[] strParams = new String[] {str1, str2, str3, str4, str5};
        for (int i = 0; i < strParams.length; i++) {
            String param = strParams[i];
            if (!param.isEmpty()) {
                int pos = param.indexOf('=');
                if (pos < 0)
                    continue;
                String key = param.substring(0, pos);
                String strvalue = param.substring(pos+1);
                Integer value = Integer.parseInt(strvalue);
                map.put(key, value);
            }
        }
    }

    public MotorInfo() {
        map = new HashMap<String, Integer>();
    }
};

class RoadRunnerTuningParams {
    // params used in MecanumDrive class
    public double inPerTick = 0.001998;
    public double lateralInPerTick = 0.0015834100566173572;
    public double trackWidthTicks = 143.42728127388037;

    public double kS = 0.6565797226088916;
    public double kV = 0.0003885572059537054;
    public double kA = 0.00005;

    public double axialGain = 0.0;
    public double lateralGain = 0;
    public double headingGain = 0.0; // shared with turn

    // params used in ThreeeDeadWheelLocalizer class
    public double par0YTicks = -70.46564435811187; // y position of the first parallel encoder (in tick units)
    public double par1YTicks = 78.7611443034976; // y position of the second parallel encoder (in tick units)
    public double perpXTicks = 54.343436628883666; // x position of the perpendicular encoder (in tick units)

    /*
    Three deadwheels
    1. ForwardPushTest --> inPerTick
    2. ForwardRampLogger -> ks, kv
    3. LateralRampLogger -> lateralInPerTick
    4. AngularRampLogger -> trackWidthTicks, par0, par1, perp
    5. ManualFeedfowardTuner -> Fine-tune kS, kV and add kA (graph vref against v0)
    6. ManualFeedbackTuner
     */
    public RoadRunnerTuningParams() {
        if (RobotDataBase.defaultType.equals("arm")) {
            inPerTick = 0.0020028164606478;
            lateralInPerTick = 0.0015451554111637759;
            trackWidthTicks = 7403.295084879505;

            kS = 0.5527812902303602;
            kV = 0.0003956868027071911;
            kA = 0.00006; //0.00005;

            axialGain = 7.5; //10.0;
            lateralGain = 1; //0;
            headingGain = 3.75; //2.0; // shared with turn

            // params used in ThreeeDeadWheelLocalizer class
            par0YTicks = -2928.0020642344175; //-2911.6703586608014; // y position of the first parallel encoder (in tick units)
            par1YTicks = 2803.4294896949446; //2811.607875790149; // y position of the second parallel encoder (in tick units)
            perpXTicks =  -3254.922499100754; //-3240.0487486738066; // x position of the perpendicular encoder (in tick units)
        }
        else if (RobotDataBase.defaultType.equals("swiper")) {
            // TODO: set all of those value to 0 to start a new tuning
            inPerTick = 0.002;
            lateralInPerTick = 0.001333683346055284;//0.0014802715031756182; //0.0013750150497965998; //0.0029349102488267285; //0.0015033902231669078; //0.0014233932241515606; //0.0015834100566173572;
            trackWidthTicks = 4984.174812870643; //4908.721667053424; // = 10.5inch / inPerTick; 4815.10771132754; //4846.309497011316;

            kS = 1.1394559439283736; //1.2384505728729382; //1.624548272149985;
            kV = 0.00026809094061452265;// 0.000263637989424697; //0.0002717084551590099; //0.00026263242439021975; //0.00026034444147174683;
            kA = 0.00007; //0.00007;

            axialGain = 20.0; //20.0;
            lateralGain = 20.0; //10.0;
            headingGain = 10.0; //6.0; // shared with turn

            // params used in ThreeeDeadWheelLocalizer class
            par0YTicks = -2912.413106503288; //-2727.5985887929314;// 2942.7727004517146; //; // y position of the first parallel encoder (in tick units)
            par1YTicks = 2707.669133875703; //2876.485177054986; //; // y position of the second parallel encoder (in tick units)
            perpXTicks =  2318.56304170385; //2336.3572902295273; //2454.49824976273; //; // x position of the perpendicular encoder (in tick units)
        }
        else if (RobotDataBase.defaultType.equals("swiperold")){
            inPerTick = 0.001996028734497; //;0.0019991253826451; //0.0019909302068355;
            lateralInPerTick = 0.001450961360343333; //0.0014233932241515606; //0.0015834100566173572;
            trackWidthTicks = 4450.081854983195; // 4435.056223760847; //4461.962059725758; //4644.894374341509; //4696.256212125061; //4678.001130307175; //4595.101609867788

            kS = 1.1400311132056107; //1.121509569386693; //1.2337817832868976;
            kV = 0.0002974816092109568; //0.00029923102708363737; //0.0003062970634198508;
            kA = 0.00007;

            axialGain = 8.0;
            lateralGain = 10.0;
            headingGain = 1.0; // shared with turn

            // params used in ThreeeDeadWheelLocalizer class
            par0YTicks = 2973.13840556989; //2998.150327831735; //-2911.6703586608014; // y position of the first parallel encoder (in tick units)
            par1YTicks = -2651.604240713771; //-2652.3405771731036; //2811.607875790149; // y position of the second parallel encoder (in tick units)
            perpXTicks =  2444.7458377027456; //2392.0075406896117; //-3240.0487486738066; // x position of the perpendicular encoder (in tick units)

            /*
            inPerTick = 0.0019955930653141; // 96 / 48106; 0.0019991253826451; //0.0019909302068355;
            lateralInPerTick = 0.001519924630882535; //0.0015033902231669078; //0.0014233932241515606; //0.0015834100566173572;
            trackWidthTicks = 5261.59375; // = 10.5inch / inPerTick; 4815.10771132754; //4846.309497011316;

            kS = 1.6474512644024806;
            kV = 0.00026203703165653244;
            kA = 0.00005;

            axialGain = 8.0;
            lateralGain = 6.0;
            headingGain = 6.0; // shared with turn

            // params used in ThreeeDeadWheelLocalizer class
            par0YTicks = 2942.3822614152946;// 2942.7727004517146; //; // y position of the first parallel encoder (in tick units)
            par1YTicks = -2670.0890495099416; //; // y position of the second parallel encoder (in tick units)
            perpXTicks =  2454.547769676639; //2454.49824976273; //; // x position of the perpendicular encoder (in tick units)


             */
        }
        else {
            // TODO: set all of those value to 0 to start a new tuning
            inPerTick = 0.00;
            lateralInPerTick = 0.00; //0.0015033902231669078; //0.0014233932241515606; //0.0015834100566173572;
            trackWidthTicks = 5000; // = 10.5inch / inPerTick; 4815.10771132754; //4846.309497011316;

            kS = 0.0;
            kV = 0.0;
            kA = 0.0;

            axialGain = 0.0;
            lateralGain = 0.0;
            headingGain = 0.0; // shared with turn

            // params used in ThreeeDeadWheelLocalizer class
            par0YTicks = 0.0;// 2942.7727004517146; //; // y position of the first parallel encoder (in tick units)
            par1YTicks = 0.0; //; // y position of the second parallel encoder (in tick units)
            perpXTicks =  0.0; //2454.49824976273; //; // x position of the perpendicular encoder (in tick units)
        }
    };
};

abstract public class RobotDataBase {
    public static String defaultType = "arm";
    public static boolean useGobildaPinpoint = false;
    public String type = defaultType;
    public String workingMode = "rednear";
    public boolean autoMode = false;
    public boolean debugMode = false;
    public boolean parkingCenter = true;
    public boolean useDeadWheel = true;
    public boolean parkingAtBackDrop = true;
    public double fastWheelSpeedManual = 1.00;
    public boolean replayInNewThreadWhenUsingThreadKeyword = true;
    public boolean sliderUseSoftStop = true;
    public boolean useNewManualDriveMethod = false;
    public boolean hangFarSide = false;
    public boolean useIMU = false;
    public boolean reverseForwardBack = true;
    public boolean recordActionsTime = false;
    public boolean runAfterAuto = false;
    public boolean runNearTeleOpEnd = false;

    public static int sleepMainThreadMilliSeconds = 1;
    public static double yawErrorRatioBase = 90;
    public static double yawErrorBigValue = 30;
    public static double yawExpectedDelta = 0.25; // 0 ~ 180
    public int minTimeOfTwoOperations = 150; //milliseconds, 0.15 second

    // debug settings
    public boolean debugReplayActionListWithSleep = false;
    public boolean scriptStopable = false;

    public boolean useListInfo = true;
    public ServoInfo[] servolist;
    public CRServoInfo[] crservolist;
    public MotorInfo[] motorlist;
    public ColorSensorInfo[] colorsensorlist;
    public DistanceSensorInfo[] distancesensorlist;
    public RoadRunnerTuningParams rrparams = new RoadRunnerTuningParams();

    public String[][] generalStringParams;
    public HashMap<String, String> generalStringParamsMap = new HashMap<String, String>();

    public String[][][] markRedNear;
    public String[][][] markRedFar;
    public String[][][] markBlueNear;
    public String[][][] markBlueFar;

    public ArrayList<String> rrActionsName = new ArrayList<>();
    public ArrayList<Action> rrActions = new ArrayList<>();

    // rednear = 0
    // readfar = 1
    // bluenear = 2
    // bluefar = 3

    public boolean useDistanceSensor = false;
    public boolean useLifter = false;
    public double distanceTurn90Degree = 24.35;

    public String[] presetActionsAutoModeInitBeforeStart = {};
    public String[] presetActionsManualModeInitBeforeStart = {};
    public String[] presetActionsInitAfterStartNonAutoMode = {};

    public String[] presetActionsManualModeInitAfterAuto = {};

    public String[] presetActionsLeft = {
    };

    public String[] presetActionsRedNear = {};

    public String[] presetActionsRedFar = presetActionsRedNear;

    public String[] presetActionsBlueNear = presetActionsRedNear;

    public String[] presetActionsBlueFar = presetActionsRedNear;

    public String[] presetActionsPad1Up = {
    };
    public String[] presetActionsPad1Down = {
    };
    public String[] presetActionsPad1Left = {
    };
    public String[] presetActionsPad1Right = {
    };
    public String[] presetActionsPad1X = {
    };
    public String[] presetActionsPad1Y = {
    };
    public String[] presetActionsPad1A = {
    };
    public String[] presetActionsPad1B = {
    };

    public String[] presetActionsPad1UpWithRightBumper = {
    };
    public String[] presetActionsPad1DownWithRightBumper = {
    };
    public String[] presetActionsPad1LeftWithRightBumper = {
    };
    public String[] presetActionsPad1RightWithRightBumper = {
    };
    public String[] presetActionsPad1UpWithLeftBumper = {
    };
    public String[] presetActionsPad1DownWithLeftBumper = {
    };
    public String[] presetActionsPad1LeftWithLeftBumper = {
    };
    public String[] presetActionsPad1RightWithLeftBumper = {
    };
    public String[] presetActionsPad1XWithLeftBumper = {
    };
    public String[] presetActionsPad1YWithLeftBumper = {
    };
    public String[] presetActionsPad1AWithLeftBumper = {
    };
    public String[] presetActionsPad1BWithLeftBumper = {
    };
    public String[] presetActionsPad1XWithRightBumper = {
    };
    public String[] presetActionsPad1YWithRightBumper = {
    };
    public String[] presetActionsPad1AWithRightBumper = {
    };
    public String[] presetActionsPad1BWithRightBumper = {
    };

    public String[] presetActionsPad1Back = {
    };
    public String[] presetActionsPad1Start = {
    };

    public String[] presetActionsPad1LeftTrigger = {
    };
    public String[] presetActionsPad1RightTrigger = {
    };
    public String[] presetActionsPad1LeftBumper = {
    };
    public String[] presetActionsPad1RightBumper = {
    };

    public String[] presetActionsPad2Up = {
    };
    public String[] presetActionsPad2Down = {
    };
    public String[] presetActionsPad2Left = {
    };
    public String[] presetActionsPad2Right = {
    };

    public String[] presetActionsPad2UpWithLeftBumper = {
    };
    public String[] presetActionsPad2DownWithLeftBumper = {
    };
    public String[] presetActionsPad2LeftWithLeftBumper = {
    };
    public String[] presetActionsPad2RightWithLeftBumper = {
    };

    public String[] presetActionsPad2UpWithRightBumper = {
    };
    public String[] presetActionsPad2DownWithRightBumper = {
    };
    public String[] presetActionsPad2LeftWithRightBumper = {
    };
    public String[] presetActionsPad2RightWithRightBumper = {
    };

    public String[] presetActionsPad2X = {
    };
    public String[] presetActionsPad2Y = {
    };
    public String[] presetActionsPad2A = {
    };
    public String[] presetActionsPad2B = {
    };

    public String[] presetActionsPad2XWithRightBumper = {
    };
    public String[] presetActionsPad2YWithRightBumper = {
    };
    public String[] presetActionsPad2AWithRightBumper = {
    };
    public String[] presetActionsPad2BWithRightBumper = {
    };
    public String[] presetActionsPad2XWithLeftBumper = {
    };
    public String[] presetActionsPad2YWithLeftBumper = {
    };
    public String[] presetActionsPad2AWithLeftBumper = {
    };
    public String[] presetActionsPad2BWithLeftBumper = {
    };
    public String[] presetActionsPad2XWithLeftStickX = {
    };
    public String[] presetActionsPad2YWithLeftStickX = {
    };
    public String[] presetActionsPad2AWithLeftStickX = {
    };
    public String[] presetActionsPad2BWithLeftStickX = {
    };
    public String[] presetActionsPad2LeftStick = {
    };
    public String[] presetActionsPad2RightStick = {
    };
    public String[] presetActionsPad2LeftTrigger = {
    };
    public String[] presetActionsPad2RightTrigger = {
    };

    public String[] presetActionsPad2Back = {
    };
    public String[] presetActionsPad2BackWithLeftBumper = {
    };
    public String[] presetActionsPad2BackWithRightBumper = {
    };
    public String[] presetActionsPad2Start = {
    };
    public String[] presetActionsDefault = {
    };
    public String[] presetActionsPad2LeftstickRightStick = {
    };
    public String[] presetActionsManualCommand1 = {
    };
    public String[] presetActionsManualAfter10Seconds = {
    };
    public String[] presetActionsManualAfter90Seconds = {
    };
    public String[] presetActionsManualAfter100Seconds = {
    };

    public HashMap<String, String[]> nextstepmap = new HashMap<String, String[]>();

    public void InitBase()
    {
        nextstepmap.put("presetActionsRedNear", presetActionsRedNear);
        nextstepmap.put("presetActionsRedFar", presetActionsRedFar);
        nextstepmap.put("presetActionsBlueNear", presetActionsBlueNear);
        nextstepmap.put("presetActionsBlueFar", presetActionsBlueFar);
        nextstepmap.put("presetActionsPad1Up", presetActionsPad1Up);
        nextstepmap.put("presetActionsPad1Down", presetActionsPad1Down);
        nextstepmap.put("presetActionsPad1Left", presetActionsPad1Left);
        nextstepmap.put("presetActionsPad1Right", presetActionsPad1Right);
        nextstepmap.put("presetActionsPad1X", presetActionsPad1X);
        nextstepmap.put("presetActionsPad1Y", presetActionsPad1Y);
        nextstepmap.put("presetActionsPad1A", presetActionsPad1A);
        nextstepmap.put("presetActionsPad1B", presetActionsPad1B);
        nextstepmap.put("presetActionsPad1UpWithRightBumper", presetActionsPad1UpWithRightBumper);
        nextstepmap.put("presetActionsPad1DownWithRightBumper", presetActionsPad1DownWithRightBumper);
        nextstepmap.put("presetActionsPad1LeftWithRightBumper", presetActionsPad1LeftWithRightBumper);
        nextstepmap.put("presetActionsPad1RightWithRightBumper", presetActionsPad1RightWithRightBumper);
        nextstepmap.put("presetActionsPad1UpWithLeftBumper", presetActionsPad1UpWithLeftBumper);
        nextstepmap.put("presetActionsPad1DownWithLeftBumper", presetActionsPad1DownWithLeftBumper);
        nextstepmap.put("presetActionsPad1LeftWithLeftBumper", presetActionsPad1LeftWithLeftBumper);
        nextstepmap.put("presetActionsPad1RightWithLeftBumper", presetActionsPad1RightWithLeftBumper);
        nextstepmap.put("presetActionsPad1XWithLeftBumper", presetActionsPad1XWithLeftBumper);
        nextstepmap.put("presetActionsPad1YWithLeftBumper", presetActionsPad1YWithLeftBumper);
        nextstepmap.put("presetActionsPad1AWithLeftBumper", presetActionsPad1AWithLeftBumper);
        nextstepmap.put("presetActionsPad1BWithLeftBumper", presetActionsPad1BWithLeftBumper);
        nextstepmap.put("presetActionsPad1XWithRightBumper", presetActionsPad1XWithRightBumper);
        nextstepmap.put("presetActionsPad1YWithRightBumper", presetActionsPad1YWithRightBumper);
        nextstepmap.put("presetActionsPad1AWithRightBumper", presetActionsPad1AWithRightBumper);
        nextstepmap.put("presetActionsPad1BWithRightBumper", presetActionsPad1BWithRightBumper);
        nextstepmap.put("presetActionsPad1Back", presetActionsPad1Back);
        nextstepmap.put("presetActionsPad1Start", presetActionsPad1Start);
        nextstepmap.put("presetActionsPad1LeftTrigger", presetActionsPad1LeftTrigger);
        nextstepmap.put("presetActionsPad1RightTrigger", presetActionsPad1RightTrigger);
        nextstepmap.put("presetActionsPad1LeftBumper", presetActionsPad1LeftBumper);
        nextstepmap.put("presetActionsPad1RightBumper", presetActionsPad1RightBumper);
        nextstepmap.put("presetActionsPad2Up", presetActionsPad2Up);
        nextstepmap.put("presetActionsPad2Down", presetActionsPad2Down);
        nextstepmap.put("presetActionsPad2Left", presetActionsPad2Left);
        nextstepmap.put("presetActionsPad2Right", presetActionsPad2Right);
        nextstepmap.put("presetActionsPad2UpWithLeftBumper", presetActionsPad2UpWithLeftBumper);
        nextstepmap.put("presetActionsPad2DownWithLeftBumper", presetActionsPad2DownWithLeftBumper);
        nextstepmap.put("presetActionsPad2LeftWithLeftBumper", presetActionsPad2LeftWithLeftBumper);
        nextstepmap.put("presetActionsPad2RightWithLeftBumper", presetActionsPad2RightWithLeftBumper);
        nextstepmap.put("presetActionsPad2UpWithRightBumper", presetActionsPad2UpWithRightBumper);
        nextstepmap.put("presetActionsPad2DownWithRightBumper", presetActionsPad2DownWithRightBumper);
        nextstepmap.put("presetActionsPad2LeftWithRightBumper", presetActionsPad2LeftWithRightBumper);
        nextstepmap.put("presetActionsPad2RightWithRightBumper", presetActionsPad2RightWithRightBumper);
        nextstepmap.put("presetActionsPad2X", presetActionsPad2X);
        nextstepmap.put("presetActionsPad2Y", presetActionsPad2Y);
        nextstepmap.put("presetActionsPad2A", presetActionsPad2A);
        nextstepmap.put("presetActionsPad2B", presetActionsPad2B);
        nextstepmap.put("presetActionsPad2XWithRightBumper", presetActionsPad2XWithRightBumper);
        nextstepmap.put("presetActionsPad2YWithRightBumper", presetActionsPad2YWithRightBumper);
        nextstepmap.put("presetActionsPad2AWithRightBumper", presetActionsPad2AWithRightBumper);
        nextstepmap.put("presetActionsPad2BWithRightBumper", presetActionsPad2BWithRightBumper);
        nextstepmap.put("presetActionsPad2XWithLeftBumper", presetActionsPad2XWithLeftBumper);
        nextstepmap.put("presetActionsPad2YWithLeftBumper", presetActionsPad2YWithLeftBumper);
        nextstepmap.put("presetActionsPad2AWithLeftBumper", presetActionsPad2AWithLeftBumper);
        nextstepmap.put("presetActionsPad2BWithLeftBumper", presetActionsPad2BWithLeftBumper);
        nextstepmap.put("presetActionsPad2XWithLeftStickX", presetActionsPad2XWithLeftStickX);
        nextstepmap.put("presetActionsPad2YWithLeftStickX", presetActionsPad2YWithLeftStickX);
        nextstepmap.put("presetActionsPad2AWithLeftStickX", presetActionsPad2AWithLeftStickX);
        nextstepmap.put("presetActionsPad2BWithLeftStickX", presetActionsPad2BWithLeftStickX);
        nextstepmap.put("presetActionsPad2LeftStick", presetActionsPad2LeftStick);
        nextstepmap.put("presetActionsPad2RightStick", presetActionsPad2RightStick);
        nextstepmap.put("presetActionsPad2LeftTrigger", presetActionsPad2LeftTrigger);
        nextstepmap.put("presetActionsPad2RightTrigger", presetActionsPad2RightTrigger);
        nextstepmap.put("presetActionsPad2Back", presetActionsPad2Back);
        nextstepmap.put("presetActionsPad2Start", presetActionsPad2Start);
        nextstepmap.put("presetActionsDefault", presetActionsDefault);
        nextstepmap.put("presetActionsPad2LeftstickRightStick", presetActionsPad2LeftstickRightStick);
        nextstepmap.put("presetActionsManualCommand1", presetActionsManualCommand1);
        nextstepmap.put("presetActionsManualAfter10Seconds", presetActionsManualAfter10Seconds);
        nextstepmap.put("presetActionsManualAfter10Seconds", presetActionsManualAfter10Seconds);
        nextstepmap.put("presetActionsManualAfter90Seconds", presetActionsManualAfter90Seconds);
        nextstepmap.put("presetActionsManualAfter100Seconds", presetActionsManualAfter100Seconds);

        for (int i=0; i<generalStringParams.length; i++) {
            generalStringParamsMap.put(generalStringParams[i][0], generalStringParams[i][1]);
        }
    }

    abstract void InitRRActions(MecanumDrive drive);
}
