package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotDataSwiper extends RobotDataBase {

	public RobotDataSwiper(boolean autoMode, String workingMode, boolean debugMode) {
		type = "swiper";
		this.autoMode = autoMode;
		this.workingMode = workingMode;
		this.debugMode = debugMode;
		useDeadWheel = true;

		generalStringParams = new String[][] {
				{"sliderpowerhold",     "0.0005"},
				{"sliderhightime",     "920"}, //"900", //"875", //"750", // change to 800 after slider add some tension stuff
				{"sliderlowtime",     "400"},
				{"powerin", 			"0.5"},
				{"powerout", 			"-0.5"},
				{"dropwaitservo",       "300"}, // 900 for torque servo, 200 for super speed, 400 for speed servo
				{"dropwaitservofromvertical",       "1"}, // if doesn't want the "throw" operation, change it to something like 300 (0.3s)
				{"hangwaitservo",       "100"}, // 600 for torque servo, 100 for speed servo
				{"servoleftdrophangdelta",       "0.01"},
				{"servorightdrophangdelta",       "-0.01"},
				{"sliderhanglowafterautotime",     "1800"},
				{"sliderhanglowupfrombuttomtime",     "4800"}, // from 4800, 4600
				{"sliderhanglowautoupfrombuttomtime",     "3750"},
				{"sliderhanglowlifttime",     "2830"},
		};

		//
		// Put servos/motors/crservos to the end of the list which are connecting to the expansion hub, thus still can operate the control hub
		//    components when expansion hub disconnected unexpected (which can only recover from a cold reboot)
		//
		this.servolist = new ServoInfo[] {
				new ServoInfo("leftarm", true, 0.0, 1.0,
						//"pick=0.198", "exchange=0.705", "move=0.649", "abovebar=0.458", "picknear=0.198", "checkcolor=0.298", "", ""),
						"pick=0.218", "exchange=0.705", "move=0.669", "abovebar=0.478", "picknear=0.218", "checkcolor=0.318", "", ""),
				//new ServoInfo("rightarm", true, 0.0, 1.0,
				//        "pick=0.39", "exchange=0.618", "move=0.559", "pickready=0.208", "picknear=0.37", "", "", ""),
				//new ServoInfo("leftdrop", true, 0.0, 1.0, // torque servo
				//		"vertical=0.61", "exchange=0.09", "ascent1=0.87", "dropbasket=0.72", "hang=0.79", "nearascent1=0.80", "", ""),
				new ServoInfo("toprotate", true, 0.0, 1.0,
						"straight=0.59", "hang=0.269", "righthang=0.269", "ascent1=0.59", "", "", "", ""),
				//new ServoInfo("lowrotate",   true, 0.0, 1.0,
				//		"straight=0.55", "leftpick=0.84", "rightpick=0.25", "drop=0.50", "getbrick3=0.689"),
				new ServoInfo("lowrotate",   true, 0.0, 1.0,
						"straight=0.55", "leftpick=0.91", "rightpick=0.22", "drop=0.50", "getbrick3=0.71", "backpick=0.0", "", ""),
				// top
				new ServoInfo("rightdrop", true, 0.0, 1.0, // speed servo
						//"vertical=0.459", "exchange=0.909", "ascent1=0.149", "dropbasket=0.239", "hang=0.299"),
						//"vertical=0.459", "exchange=0.909", "ascent1=0.149", "dropbasket=0.239", "hang=0.359"), // for vertical
						//"vertical=0.459", "exchange=0.909", "ascent1=0.149", "dropbasket=0.239", "hang=0.299", "", "", ""),
						"vertical=0.459", "exchange=0.889", "ascent1=0.149", "dropbasket=0.239", "hang=0.299", "nearascent1=0.409", "", ""),
		};

		this.crservolist = new CRServoInfo[] {
				new CRServoInfo("intake",       -1.00,           1.000,         false,      false),
				new CRServoInfo("outtake",       -1.00,           1.000,         false,      false),
		};

		this.motorlist = new MotorInfo[] {
				//new MotorInfo("sliderdropleft",  true, true,100, 2800, new PIDFCoefficients(1.0, 0.1, 0,1),
				//		"hang=800", "drophigh=1300", "droplow=900", "", ""),
				new MotorInfo("sliderdropright",  false, true,100, 2800, new PIDFCoefficients(1.0, 0.1, 0,1),
						"hang=800", "drophigh=1300", "droplow=900", "", ""),
				// power 1.0 800ms runtoposition=1500, 900ms=1732, 1000ms=1992
				//new MotorInfo("sliderpickleft",true, true,100, 1980, new PIDFCoefficients(1.0, 0.1, 0,1),
				//		"pickshort=900", "picklong=1900", "hang=850", "brick1=1200", "brick2=1500"),
				new MotorInfo("sliderpickright",false, true,100, 1980, new PIDFCoefficients(1.0, 0.1, 0,1),
						"pickshort=900", "picklong=1800", "hang=850", "brick1=1200", "brick2=1500"),
				//new MotorInfo("hanglow",false, true,100, 2000, null,"top=-14400", "autotop=1800", "autodown=-1800", "", ""),
				new MotorInfo("hanglow",true, true,100, 2000, null,"top=3586", "topauto=2475", "hang=1716", "", ""),
				//new MotorInfo("hanghigh",true, true,100, 1980, new PIDFCoefficients(1.0, 0.1, 0,1),
				//		"vertical=200", "picklong=1800", "hang=850", "brick1=1200", "brick2=1500"),
		};

		this.colorsensorlist = new ColorSensorInfo[] {
				new ColorSensorInfo("pickcolor", true),
		};

		this.distancesensorlist = new DistanceSensorInfo[] {};

		presetActionsAutoModeInitBeforeStart = new String[]{
				"servo lowrotate straight",
				"servo toprotate straight",
				"servo leftarm exchange",
				"servo rightarm exchange",
				"servo leftdrop exchange",
				"servo rightdrop exchange",
		};
		presetActionsManualModeInitBeforeStart = new String[]{
		};

		presetActionsInitAfterStartNonAutoMode = new String[]{
				"servo lowrotate straight",
				"servo toprotate straight",
				"servo leftarm exchange",
				"servo rightarm exchange",
				"servo leftdrop exchange",
				"servo rightdrop exchange",
				"rr manualbegin",
		};

		presetActionsManualModeInitAfterAuto = new String[]{
				//"thread motor hanglow -1.0 sliderhanglowafterautotime",
		};

		presetActionsRedNear = new String[]{
				"set recordactionstime on",
				"thread rr dropbasketbegin", //1.537s
				"sleep 350", //"sleep 650", // --------------------------------> save about 0.85s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", //2.253s
				"sleep 1500", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr getbrick1frombasket", //splineTo version 2.744s; strafe 3.264s
				"sleep 2000",
				"servo leftarm pick", //drop the arm first
				"servo rightarm pick",
				"crservo intake powerin",
				"sleep 600",
				"nextstep presetActionsGetBrickStraight 500", //1.073s
				"thread nextstep presetActionsExchange 320 300 500", //1.511s
				//"sleep 100", // --------------------------------> save about 1.5s exchange done mostly when robot moving
				"thread rr dropbasket1", //2.393s
				"sleep 1500", // --------------------------------> save about 0.89s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", // 2.239s
				"sleep 1500", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr getbrick2frombasket", // 2.243s
				"sleep 1400",
				"servo leftarm checkcolor", //drop the arm first
				"servo rightarm checkcolor",
				"sleep 300",
				"servo leftarm pick", //drop the arm first
				"servo rightarm pick",
				"crservo intake powerin",
				"sleep 500",
				"nextstep presetActionsGetBrickStraight 500", //1.069s
				"thread nextstep presetActionsExchange 420 300 500", // 1.515s
				//"sleep 100", // --------------------------------> save about 1.5s, exchange done mostly when robot moving
				"thread rr dropbasket2", // 2.266s
				"sleep 1400", // --------------------------------> save about 0.86s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", // 2.247s
				"sleep 1500", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr getbrick3frombasket", // 2.206s
				"sleep 1600",
				"servo leftarm checkcolor", //drop the arm first
				"servo rightarm checkcolor",
				"sleep 200",
				"servo leftarm pick", //drop the arm first
				"servo rightarm pick",
				"crservo intake powerin",
				"sleep 400",
				"nextstep presetActionsGetBrickStraight 450", // 1.075s - 0.25 (change from 500 -> 250)
				"thread nextstep presetActionsExchange 420 300 600", // 1.525s
				//"sleep 100", // --------------------------------> save about 1.5s, exchange done mostly when robot moving
				"thread rr dropbasket3", // 2.239s
				"sleep 1400", // --------------------------------> save about 0.83s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", //2.248s
				"sleep 1500", // --------------------------------> save about 0.74s, slider need about 900ms to get down
				"thread rr getbrick4frombasketfast", // 1.731 (all 100), 2.217s (100, -50, 100); 2.827s when vel 90,-30,50; 2.116 when vel set to 75 (profile 100); 1.734s when all set to 100
				"sleep 1000", // change from 1600 -> 1400
				"servo lowrotate straight",
				"servo leftarm abovebar",
				"servo rightarm abovebar",
				"sleep 400",
				"nextstep presetActionsGetBrick4 %%arg1=blue 400 400",
				"thread nextstep presetActionsExchange 850", // 1.515s
				"thread rr dropbasket4fast", // 1.737s (all 100 might not accurate to drop basket), 2.230s (100, -50, 100); 2.835s when vel 90, -30, 50; 2.192s when vel set to 75 (profile 100), 1.730 when all set to 100
				"sleep 1600", // might be able to reduce to 1100,
				"thread nextstep presetActionsDropBasket", //2.253s
				"thread motor hanglow 0.5 runtoposition topauto",
				"sleep 1400", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr parkcenterfast", // 2.503s (100,-100,100 with another strafeTo), 2.227s (100, -50, 100); 2.929s when 2.192s when vel set to 75 (profile 100), 1.727  when all set to 100 but not accurate
				//"sleep 1000", // --------------------------------> save about 0.7s, slider need about 900ms to get down (change from 2000 -> 1500)
				//"nextstep presetActionsAscent1",
				//"sleep 1250",
				//"servo leftdrop ascent1",
				//"servo rightdrop ascent1",
				"set recordactionstime off",
		};

		presetActionsRedFar = new String[]{
				"rr hangclipbegin",// 2.287s
				"nextstep presetActionsHangClip", //2.287s
				"thread rr farbrick2", // 8.532
				"sleep 6000",
				"servo leftarm pick",
				"servo rightarm pick",
				"sleep 1500",
				"nextstep presetActionsGetClip 1200", //0.88s
				"thread nextstep presetActionsExchangeClip 700 300 500", //1.476s
				//"sleep 10", // ---------------------------------> save about 1.46s
				"rr hangclip1", //2.533s
				"nextstep presetActionsHangClip", //2.281s
				"rr getclip2", // 2.492s
				"sleep 1000",
				"servo leftarm pick",
				"servo rightarm pick",
				"sleep 1500",
				"nextstep presetActionsGetClip 1200", //0.87s
				"thread nextstep presetActionsExchangeClip 700 300 500", //1.471s
				//"sleep 10", // ---------------------------------> save about 1.46s
				"rr hangclip2", // 2.62s
				"nextstep presetActionsHangClip", //2.282s
				"rr parkfarfromhang", //2.624s
		};

		presetActionsBlueNear = new String[]{
				"nextstep presetActionsRedNear red",
		};

		presetActionsBlueFar = presetActionsRedFar;

		presetActionsPad1Up = new String[]{
				"servo lowrotate straight",
		};

		presetActionsPad1Down = new String[]{
				"servo lowrotate 0.0", // point to back
		};

		presetActionsPad1Left = new String[]{
				"servo lowrotate leftpick",
		};

		presetActionsPad1Right = new String[]{
				"servo lowrotate rightpick",
		};

		presetActionsPad1X = new String[]{
				"crservo outtake powerout",
				"crservo intake powerout",
				"sleep %%arg1=400",
				"crservo intake 0.0",
				"crservo outtake 0.0",
		};

		presetActionsPad1Y = new String[]{
				"if oneinstance exchange == off", // when there is another instance running, will not run even hit the button
					"set oneinstance exchange 1500", // until 1500ms later, another instance can be run
					"thread nextstep presetActionsExchange 1100 300 200",
		};

		presetActionsPad1A = new String[]{
				"motor sliderpickleft 1.0",
				"motor sliderpickright 1.0",
				"sleep 200",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
				"crservo intake powerin", //pick
				"servo leftarm pick",
				"servo rightarm pick",
		};

		presetActionsPad1B = new String[]{
				"if oneinstance dropbrick == off", // when there is another instance running, will not run even hit the button
					"set oneinstance dropbrick 1000", // until 1000ms later, another instance can be run.
					"thread nextstep presetActionsDropBrick",
		};

		presetActionsPad1Back = new String[]{
				//"if oneinstance log == off", // when there is another instance running, will not run even hit the button
				//	"set oneinstance log 5000", // until 5000ms later, another instance can be run
				"motor hanglow -0.5 runtoposition hang",
		};

		presetActionsPad1LeftTrigger = new String[]{
				"motor sliderpickleft powerin",
				"motor sliderpickright powerin",
				"sleep 1",
				"if key pad1lefttrigger == off", // when still hold the key, will return as check fail here
					"motor sliderpickleft 0.0",  // only stop motor when key is off
					"motor sliderpickright 0.0",
		};

		presetActionsPad1RightTrigger = new String[]{
				"motor sliderpickleft powerout",
				"motor sliderpickright powerout",
				"sleep 1",
				"if key pad1righttrigger == off", // when still hold the key, will return as check fail here
					"motor sliderpickleft 0.0",  // only stop motor when key is off
					"motor sliderpickright 0.0",
		};

		presetActionsPad1LeftBumper = new String[]{
				"motor sliderdropleft 1.0",
				"motor sliderdropright 1.0",
				"sleep sliderlowtime",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
		};

		presetActionsPad1RightBumper = new String[]{
				//"thread motor sliderdropleft 0.5 runtoposition drophigh 2000 hold",
				//"thread motor sliderdropright 0.5 runtoposition drophigh 2000 hold",
				//"exit",
				"motor sliderdropleft 1.0",
				"motor sliderdropright 1.0",
				"servo leftdrop vertical", // servo change to closer position to drop
				"servo rightdrop vertical", // servo change to closer position to drop
				"servo toprotate straight",
				"sleep sliderhightime",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
		};

		presetActionsPad2Up = new String[]{
				"motor sliderdropleft 1.0",
				"motor sliderdropright 1.0",
				"sleep 120",
				"servo leftdrop hang",
				"servo rightdrop hang",
				"servo toprotate hang",
				"sleep 280",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
				"exit",
		};

		presetActionsPad2Down = new String[]{
				"crservo outtake powerout",
				"sleep 300",
				"servo toprotate straight",
				"servo leftdrop exchange",
				"servo rightdrop exchange",
				"sleep 300",
				"crservo outtake 0.0",
				//"motor sliderdropleft -1.0",
				//"motor sliderdropright -1.0",
				//"sleep 250",
				//"motor sliderdropleft 0.0",
				//"motor sliderdropright 0.0",
				"thread motor sliderdropleft -1.0 runtoposition 0 1500",
				"thread motor sliderdropright -1.0 runtoposition 0 1500",
				"exit",
				"motor sliderdropleft -0.5",
				"motor sliderdropright -0.5",
				"sleep 200",
				"if key pad2down == off", // when still hold the key, will return as check fail here
					"motor sliderdropleft 0.0",  // only stop motor when key is off
					"motor sliderdropright 0.0",
		};

		presetActionsPad2Left = new String[]{
				"motor sliderdropleft 0.5",
				"motor sliderdropright 0.5",
				"sleep 100",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
				//"sleep 1",
				//"if key pad2left == off", // when still hold the key, will return as check fail here
				//	"motor sliderdropleft sliderpowerhold",
				//	"motor sliderdropright sliderpowerhold",
		};

		presetActionsPad2Right = new String[]{
				"motor sliderdropleft -0.5",
				"motor sliderdropright -0.5",
				"sleep 100",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
				//"sleep 1",
				//"if key pad2right == off", // when still hold the key, will return as check fail here
				//	"motor sliderdropleft sliderpowerhold",
				//	"motor sliderdropright sliderpowerhold",
		};

		presetActionsPad2UpWithLeftBumper = new String[]{
				"servo leftarm 0.01 delta",
		};
		presetActionsPad2DownWithLeftBumper = new String[]{
				"servo leftarm -0.01 delta",
		};
		presetActionsPad2LeftWithLeftBumper = new String[]{
				"manualwheel enable",
		};
		presetActionsPad2RightWithLeftBumper = new String[]{
				//"manualwheel disable",
				//"nextstep presetActionsRedNear",
		};

		presetActionsPad2UpWithRightBumper = new String[]{
				"motor hanglow 1.0 200",
				//"sleep 500",
				//"motor hanglow 0.6"
				//"motor hanghigh -1.0",
				//"sleep 2000",
				//"motor hanghigh -0.6"
		};

		presetActionsPad2DownWithRightBumper = new String[]{
				"motor hanglow -1.0 200",
				//"motor hanglow initposition",
				//"motor hanglow 0.0",
				//"motor hanghigh 0.0",
		};

		presetActionsPad2LeftWithRightBumper = new String[]{
				"motor hanglow 0.5 runtoposition top",
				//"motor hanghigh 1.0 200",
		};

		presetActionsPad2RightWithRightBumper = new String[]{
				"motor hanglow 0.5 runtoposition topauto",
				//"motor hanghigh -1.0 200",
		};

		presetActionsPad2X = new String[]{
				"manualwheel disable",
				"rr delta -3.2 0.0", // x = x - 3.2, y = y + 0.0
				"nextstep presetActionsHangClip",
				"manualwheel enable",
		};

		presetActionsPad2Y = new String[]{
				"servo leftdrop exchange",
				"servo rightdrop exchange",
				"servo toprotate straight",
				"crservo outtake powerin",
				"servo leftarm exchange",
				"servo rightarm exchange",
				"sleep 300",
				"crservo intake powerout",
				"sleep 300",
				"crservo intake 0.0",
				"sleep 500",
				"crservo outtake 0.0",
		};

		presetActionsPad2A = new String[]{
				"servo leftarm pick",
				"servo rightarm pick",
				"sleep 200",
				"crservo intake powerin", //pick
				"motor sliderpickleft 0.4",
				"motor sliderpickright 0.4",
				"sleep 500",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
				"servo leftarm exchange",
				"servo rightarm exchange",
				"motor sliderpickleft -0.4",
				"motor sliderpickright -0.4",
				"sleep 300",
				"crservo intake 0.0",
				"sleep 300",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
		};

		presetActionsPad2B = new String[]{
				"crservo outtake powerin",
				"motor sliderdropleft -0.4",
				"motor sliderdropright -0.4",
				"sleep 200",
				"motor sliderdropleft 0.001",
				"motor sliderdropright 0.001",
				"crservo outtake 0.0",
		};

		presetActionsPad2XWithRightBumper = new String[]{
				//"servo lowrotate 0.01 delta",
				//"manualwheel disable",
				//"rr delta -3.2 0.0", // x = x - 3.2, y = y + 0.0
				"nextstep presetActionsHangClip",
				//"manualwheel enable",
		};
		presetActionsPad2YWithRightBumper = new String[]{
				"servo toprotate 0.01 delta",
		};
		presetActionsPad2AWithRightBumper = new String[]{
				"servo toprotate -0.01 delta",
		};
		presetActionsPad2BWithRightBumper = new String[]{
				"servo lowrotate -0.01 delta",
		};
		presetActionsPad2XWithLeftBumper = new String[]{
				"servo toprotate 0.01 delta",
		};
		presetActionsPad2YWithLeftBumper = new String[]{
				"servo leftdrop 0.01 delta",
				"servo rightdrop 0.01 delta",
		};
		presetActionsPad2AWithLeftBumper = new String[]{
				"servo leftdrop -0.01 delta",
				"servo rightdrop -0.01 delta",
		};
		presetActionsPad2BWithLeftBumper = new String[]{
				"servo toprotate -0.01 delta",
		};
		presetActionsPad2XWithLeftStickX = new String[]{
		};
		presetActionsPad2YWithLeftStickX = new String[]{
		};
		presetActionsPad2AWithLeftStickX = new String[]{
		};
		presetActionsPad2BWithLeftStickX = new String[]{
		};
		presetActionsPad2LeftStick = new String[]{
				"motor hanglow initposition",
				"exit",

				"set recordactionstime on",
				"manualwheel disable",
				"rr dropbasketbegin", //1.537s
				"sleep 1000", //"nextstep presetActionsDropBasket", //2.253s
				"rr getbrick1frombasket", //3.264s
				"sleep 1000", //"nextstep presetActionsGetBrickStraight 1000", //1.073s
				"sleep 1000", //"nextstep presetActionsExchange 620", //1.511s
				"rr dropbasket1", //2.393s
				"sleep 1000", //"nextstep presetActionsDropBasket", // 2.239s
				"rr getbrick2frombasket", // 2.243s
				"sleep 1000", //"nextstep presetActionsGetBrickStraight 1000", //1.069s
				"sleep 1000", //"nextstep presetActionsExchange 620", // 1.515s
				"rr dropbasket2", // 2.266s
				"sleep 1000", //"nextstep presetActionsDropBasket", // 2.247s
				"rr getbrick3frombasket", // 2.206s
				"sleep 1000", //"nextstep presetActionsGetBrickStraight 1000", // 1.075s
				"sleep 1000", //"nextstep presetActionsExchange 620", // 1.525s
				"rr dropbasket3", // 2.239s
				"sleep 1000", //"nextstep presetActionsDropBasket", //2.248s
				//"rr parkcenter", //4.148s
				//"nextstep presetActionsAscent1",
				"rr getbrick4frombasketfast", // 1.731 (all 100), 2.217s (100, -50, 100); 2.827s when vel 90,-30,50; 2.116 when vel set to 75 (profile 100); 1.734s when all set to 100
				"sleep 1000",
				"rr dropbasket4fast", // 1.737s (all 100 might not accurate to drop basket), 2.230s (100, -50, 100); 2.835s when vel 90, -30, 50; 2.192s when vel set to 75 (profile 100), 1.730 when all set to 100
				"sleep 1000",
				"rr parkcenterfast", // 2.503s (100,-100,100 with another strafeTo), 2.227s (100, -50, 100); 2.929s when 2.192s when vel set to 75 (profile 100), 1.727  when all set to 100 but not accurate
				"set recordactionstime off",
		};

		presetActionsPad2RightStick = new String[]{
				"nextstep presetActionsManualCommand1",
				"exit",

				"set recordactionstime on",
				"thread rr dropbasketbegin", //1.537s
				"sleep 700", // --------------------------------> save about 0.8s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", //2.253s
				"sleep 1500", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr getbrick1frombasket", //splineTo version 2.744s; strafe 3.264s
				"sleep 2000",
				"servo leftarm pick", //drop the arm first
				"servo rightarm pick",
				"crservo intake powerin",
				"sleep 600",
				"nextstep presetActionsGetBrickStraight 500", //1.073s
				"thread nextstep presetActionsExchange 320", //1.511s
				//"sleep 100", // --------------------------------> save about 1.5s exchange done mostly when robot moving
				"thread rr dropbasket1", //2.393s
				"sleep 1600", // --------------------------------> save about 0.79s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", // 2.239s
				"sleep 1500", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr getbrick2frombasket", // 2.243s
				"sleep 1400",
				"servo leftarm pick", //drop the arm first
				"servo rightarm pick",
				"crservo intake powerin",
				"sleep 800",
				"nextstep presetActionsGetBrickStraight 500", //1.069s
				"thread nextstep presetActionsExchange 320", // 1.515s
				//"sleep 100", // --------------------------------> save about 1.5s, exchange done mostly when robot moving
				"thread rr dropbasket2", // 2.266s
				"sleep 1500", // --------------------------------> save about 0.76s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", // 2.247s
				"sleep 1500", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr getbrick3frombasket", // 2.206s
				"sleep 1600",
				"servo leftarm pick", //drop the arm first
				"servo rightarm pick",
				"crservo intake powerin",
				"sleep 600",
				"nextstep presetActionsGetBrickStraight 500", // 1.075s
				"thread nextstep presetActionsExchange 320", // 1.525s
				//"sleep 100", // --------------------------------> save about 1.5s, exchange done mostly when robot moving
				"thread rr dropbasket3", // 2.239s
				"sleep 1500", // --------------------------------> save about 0.73s, slider need about 900ms to get up
				"thread nextstep presetActionsDropBasket", //2.248s
				"sleep 1500", // --------------------------------> save about 0.74s, slider need about 900ms to get down
				"rr getbrick4frombasketfast", // 1.731 (all 100), 2.217s (100, -50, 100); 2.827s when vel 90,-30,50; 2.116 when vel set to 75 (profile 100); 1.734s when all set to 100
				"nextstep presetActionsGetBrick4 %%arg1=blue 400 400",
				"thread nextstep presetActionsExchange 850", // 1.515s
				"thread rr dropbasket4fast", // 1.737s (all 100 might not accurate to drop basket), 2.230s (100, -50, 100); 2.835s when vel 90, -30, 50; 2.192s when vel set to 75 (profile 100), 1.730 when all set to 100
				"sleep 1600", // might be able to reduce to 1100,
				"thread nextstep presetActionsDropBasket", //2.253s
				"sleep 1400", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"thread rr parkcenterfast", // 2.503s (100,-100,100 with another strafeTo), 2.227s (100, -50, 100); 2.929s when 2.192s when vel set to 75 (profile 100), 1.727  when all set to 100 but not accurate
				"sleep 2000", // --------------------------------> save about 0.7s, slider need about 900ms to get down
				"nextstep presetActionsAscent1",
				//"thread rr parkcenter", // old, 4.148s
				//"sleep 2000",
				//"nextstep presetActionsAscent1",
				//"manualwheel enable",
				"set recordactionstime off",
		};
		presetActionsPad2LeftTrigger = new String[]{
				"crservo outtake powerout 400",
		};
		presetActionsPad2RightTrigger = new String[]{
				"crservo intake powerout 400",
		};
		presetActionsPad2Back = new String[]{
				"if colorsensor pickcolor == blue else goto notblue",
					"log blue",
					"sleep 3000",
					"exit",
				"notblue:",
					"if colorsensor pickcolor == red else goto notred",
						"log red",
						"sleep 3000",
						"exit",
					"notred:",
						"if colorsensor pickcolor == yellow else goto notdetected",
							"log yellow",
							"sleep 3000",
							"exit",
						"notdetected:",
							"log notdetected",
							"sleep 3000",
							"exit",
				//"manualwheel disable",
				//"rr dropbasket4",
				//"manualwheel enable",
		};

		presetActionsPad2LeftstickRightStick = new String[]{
		};

		presetActionsManualCommand1 = new String[] {
				"motor hanglow -1.0 sliderhanglowafterautotime",
				"motor hanglow initposition",
		};

		presetActionsManualAfter10Seconds = new String[] {
				"thread nextstep presetActionsManualCommand1",
		};
		presetActionsManualAfter90Seconds = new String[] {
		};
		presetActionsManualAfter100Seconds = new String[]{
				//"thread motor hanglow 0.5 runtoposition top",
		};

		String[] presetActionsGetBrickStraight = new String[] {
				"servo leftarm pick",
				"servo rightarm pick",
				"crservo intake powerin",
				"servo lowrotate straight",
				"sleep %%arg2=100",
				"motor sliderpickleft %%arg3=0.5",
				"motor sliderpickright %%arg3=0.5",
				"sleep %%arg1=600",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
		};

		String[] presetActionsGetBrick4 = new String[] {
				"motor sliderpickleft 0.5",
				"motor sliderpickright 0.5",
				"sleep 200",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
				"crservo intake powerin",
				"servo leftarm pick",
				"servo rightarm pick",
				"sleep 200",
				"motor sliderpickleft 0.5",
				"motor sliderpickright 0.5",
				"sleep %%arg2=600",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
				//"nextstep presetActionsCheckColorAndTryAgain %%arg1=blue",				// check the color of the sample
				"servo leftarm checkcolor",
				"servo rightarm checkcolor",
				"sleep 100",
				"if colorsensor pickcolor != empty else goto tryagain",
					"if colorsensor pickcolor == %%arg1=blue", // other colors will return from here
						//drop it
						"servo lowrotate leftpick",
						"sleep 50",
						"crservo intake -1.0",
						"sleep 200",
				"tryagain:",
					//try again
					"servo lowrotate straight",
					"servo leftarm pick",
					"servo rightarm pick",
					"crservo intake powerin",
					"sleep 200",
					"motor sliderpickleft 0.5",
					"motor sliderpickright 0.5",
					"sleep %%arg3=400",
					"motor sliderpickleft 0.0",
					"motor sliderpickright 0.0",
					// check the color of the sample
					"servo leftarm checkcolor",
					"servo rightarm checkcolor",
					"sleep 100",
					"if colorsensor pickcolor != empty else goto trythird",
						"if colorsensor pickcolor == %%arg1=blue", // other colors will return from here
							//drop it
							"servo lowrotate leftpick",
							"sleep 50",
							"crservo intake -1.0",
							"sleep 200",
					"trythird:",
						"exit", //uncomment this if not want 3rd try

						"servo lowrotate straight",
						"servo leftarm pick",
						"servo rightarm pick",
						"crservo intake powerin",
						"sleep 200",
						"motor sliderpickleft 0.4",
						"motor sliderpickright 0.4",
						"sleep %%arg3=400",
						"motor sliderpickleft 0.0",
						"motor sliderpickright 0.0",
						// check the color of the sample
						"servo leftarm checkcolor",
						"servo rightarm checkcolor",
						"sleep 100",
						"if colorsensor pickcolor == %%arg1=blue", // last time, drop it for blue, other colors (including empty) will return from here
							//drop it
							"servo lowrotate leftpick",
							"sleep 50",
							"crservo intake -1.0",
							"sleep 200",
		};

		String[] presetActionsCheckColorAndTryAgain = new String[] {
				// check the color of the sample
				"servo leftarm checkcolor",
				"servo rightarm checkcolor",
				"sleep 100",
				"if colorsensor pickcolor != empty else goto tryagain",
					"if colorsensor pickcolor == %%arg1=blue", // other colors will return from here
						//drop it
						"servo lowrotate leftpick",
						"sleep 50",
						"crservo intake -1.0",
						"sleep 200",
				"tryagain:",
					//try again
					"servo lowrotate straight",
					"servo leftarm pick",
					"servo rightarm pick",
					"crservo intake powerin",
					"sleep 200",
					"motor sliderpickleft 0.4",
					"motor sliderpickright 0.4",
					"sleep %%arg2=400",
					"motor sliderpickleft 0.0",
					"motor sliderpickright 0.0",
					// check the color of the sample
					"servo leftarm checkcolor",
					"servo rightarm checkcolor",
					"sleep 100",
					"if colorsensor pickcolor != empty else goto trythird",
						"if colorsensor pickcolor == %%arg1=blue", // other colors will return from here
							//drop it
							"servo lowrotate leftpick",
							"sleep 50",
							"crservo intake -1.0",
							"sleep 200",
					"trythird:",
						"exit", //uncomment this if not want 3rd try

						"servo lowrotate straight",
						"servo leftarm pick",
						"servo rightarm pick",
						"crservo intake powerin",
						"sleep 200",
						"motor sliderpickleft 0.4",
						"motor sliderpickright 0.4",
						"sleep %%arg2=400",
						"motor sliderpickleft 0.0",
						"motor sliderpickright 0.0",
						// check the color of the sample
						"servo leftarm checkcolor",
						"servo rightarm checkcolor",
						"sleep 100",
						"if colorsensor pickcolor == %%arg1=blue", // last time, drop it for blue, other colors (including empty) will return from here
							//drop it
							"servo lowrotate leftpick",
							"sleep 50",
							"crservo intake -1.0",
							"sleep 200",
		};

		String[] presetActionsGetClip = new String[] {
				"motor sliderpickleft 0.4",
				"motor sliderpickright 0.4",
				"crservo intake 0.5", //pick
				"servo leftarm pick",
				"servo rightarm pick",
				"servo lowrotate straight",
				"sleep %%arg1=800",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
		};

		String[] presetActionsExchangeClip = new String[]{
				"servo lowrotate straight",
				"servo leftarm exchange",
				"servo rightarm exchange",
				"servo leftdrop exchange",
				"servo rightdrop exchange",
				"servo toprotate straight",
				"crservo intake 0.0",
				"motor sliderpickleft -1.0",
				"motor sliderpickright -1.0",
				"sleep %%arg1=400",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
				// exchange
				"sleep 100",
				"crservo outtake @0.5",
				"sleep 100", //"sleep 300",
				"crservo intake @-0.5",
				"sleep 300",
				"crservo intake 0.0",
				"sleep 500",
				"crservo outtake 0.0",
		};

		String[] presetActionsExchange = new String[]{
				"servo lowrotate straight",
				"servo leftarm exchange",
				"servo rightarm exchange",
				"servo leftdrop exchange",
				"servo rightdrop exchange",
				"servo toprotate straight",
				"sleep %%arg3=100",
				"crservo outtake 0.5", //move to the beginning
				//"thread motor sliderpickleft -1.0 runtoposition 0 800",
				//"thread motor sliderpickright -1.0 runtoposition 0 800",
				"motor sliderpickleft -1.0",
				"motor sliderpickright -1.0",
				"sleep %%arg1=800",
				"crservo intake 0.0",
				"motor sliderpickleft 0.0",
				"motor sliderpickright 0.0",
				// exchange
				//"motor sliderpickleft -1.0",
				//"motor sliderpickright -1.0",
				//"sleep 50",
				//"motor sliderpickleft 0.0",
				//"motor sliderpickright 0.0",
				//"motor sliderpickleft initposition",
				//"motor sliderpickright initposition",

				//"crservo outtake 0.5", //move to the beginning
				//"sleep 100", // might not need wait the slider
				"crservo intake -0.5",
				"sleep 300",
				"crservo intake 0.0",
				"sleep %%arg2=300", //500
				autoMode ? "": "crservo outtake 0.0", // autoMode will stop the outtake when drop basket
		};

		String[] presetActionsDropBasket = new String[] {
				autoMode ? "crservo outtake 0.0" : "", // autoMode will stop the outtake when drop basket
				"motor sliderdropleft 1.0",
				"motor sliderdropright 1.0",
				"servo leftdrop vertical", // servo change to closer position to drop
				"servo rightdrop vertical", // servo change to closer position to drop
				"servo toprotate straight",
				"sleep sliderhightime",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
				"nextstep presetActionsDropBrick",
		};

		String[] presetActionsDropBrick = new String[] {
				"servo leftdrop dropbasket",
				"servo rightdrop dropbasket",
				"servo toprotate straight",
				//"sleep %%arg1=dropwaitservofromvertical",
				"crservo outtake -1.0", //out
				"sleep 300",
				"crservo outtake 0.0",
				"servo leftdrop exchange",
				"servo rightdrop exchange",
				"sleep %%arg2=100", //wait for the servo to back a little before move the slider down
				"motor sliderdropleft -1.0",
				"motor sliderdropright -1.0",
				"sleep sliderhightime",
				"motor sliderdropleft 0.0",
				"motor sliderdropright 0.0",
		};

		String[] presetActionsHangClip = new String[] {
				"servo leftdrop hang",
				"servo rightdrop hang",
				"servo toprotate hang",
				"motor sliderdropleft 1.0",
				"motor sliderdropright 1.0",
				"sleep 390",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
				"sleep 200", // wait for the servo to reach its position as it might be slow for torque servo
				"crservo outtake powerin",
				"motor sliderdropleft -0.5",
				"motor sliderdropright -0.5",
				//"sleep 500", // change it sleep multiple times and servo change positions
				//"sleep 100",
				//"servo leftdrop servoleftdrophangdelta delta",
				//"servo rightdrop servorightdrophangdelta delta",
				"servo leftdrop 0.82",
				"servo rightdrop 0.20",
				"sleep 200",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
				"sleep 200",
				"motor sliderdropleft -0.5",
				"motor sliderdropright -0.5",
				"servo leftdrop ascent1",
				"servo rightdrop ascent1",
				"sleep 200",
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",
				"crservo outtake -0.5",
				"sleep 300",
				"crservo outtake 0.0",
				"servo leftdrop exchange",
				"servo rightdrop exchange",
				"servo toprotate straight",
				"motor sliderdropleft -1.0",
				"motor sliderdropright -1.0",
				"sleep 200",
				"motor sliderdropleft 0.0",
				"motor sliderdropright 0.0",
		};

		String[] presetActionsAscent1 = new String[] {
				"motor sliderdropleft 1.0",
				"motor sliderdropright 1.0",
				"sleep 50",
				"servo leftdrop nearascent1",
				"servo rightdrop nearascent1",
				"servo toprotate ascent1",
				"sleep 200", //220
				"motor sliderdropleft sliderpowerhold",
				"motor sliderdropright sliderpowerhold",

		};

		InitBase();
		nextstepmap.put("presetActionsGetBrickStraight", presetActionsGetBrickStraight);
		nextstepmap.put("presetActionsGetBrick4", presetActionsGetBrick4);
		nextstepmap.put("presetActionsCheckColorAndTryAgain", presetActionsCheckColorAndTryAgain);
		nextstepmap.put("presetActionsExchange", presetActionsExchange);
		nextstepmap.put("presetActionsExchangeClip", presetActionsExchangeClip);
		nextstepmap.put("presetActionsDropBasket", presetActionsDropBasket);
		nextstepmap.put("presetActionsDropBrick", presetActionsDropBrick);
		nextstepmap.put("presetActionsHangClip", presetActionsHangClip);
		nextstepmap.put("presetActionsAscent1", presetActionsAscent1);
		nextstepmap.put("presetActionsGetClip", presetActionsGetClip);
	}

	private void InitRRActionsNear(MecanumDrive drive) {
		/*
		double xdropbasket = 6.8, xgetbrick1middle = 20, xgetbrick1 = 40, xgetbrick2 = xgetbrick1, xgetbrick3 = xgetbrick1, xparkcenter = 58;
		double ydropbasket = 18.95, ygetbrick1middle = 0.5, ygetbrick1 = 0.5, ygetbrick2 = ygetbrick1 + 10, ygetbrick3 = ygetbrick1 + 18,
				yparkcenter1 = 0.0, yparkcenter = -14;
		double angledropbasket = 140, anglegetbrick1 = 280, anglegetbrick2 = anglegetbrick1, anglegetbrick3 = 285, angleparkcenter = 275;
		 */
		// xdropbasket4 = 6.8, ydropbasket4 = 17.0;
		double xdelta = 0.0, ydelta = -1.0;
		double  xdropbasketbegin = 6.8, xdropbasket1 = 5.8, xdropbasket2 = 5.8,xdropbasket3 = 5.8 - 1.0, xdropbasket4 = 8.8 + 3.0 - 3.0,
				xgetbrick1 = 40, xgetbrick2 = xgetbrick1 - 1.0, xgetbrick3 = xgetbrick1 -2.0,
				xparkcenter = 72.0, xparkcenterfast= 68.0 - 10.0, xgetbrick4left = 66 + xdelta - 10.0, xgetbrick4center = 32;
		double  ydropbasketbegin = 18.75, ydropbasket1 = 18.95, ydropbasket2 = 18.95, ydropbasket3 = 18.95 + 1.0, ydropbasket4 = 18.0 - 4.0 + 3.0,
				ygetbrick1 = 0.5 + ydelta, ygetbrick2 = 10.5 + ydelta, ygetbrick3 = 21.5 + ydelta,
				yparkcenter1 = 0.0, yparkcenter = -14, ygetbrick4left = -14.2, ygetbrick4leftfast = -9.0 - 3.0,  yparkcenterfast = -17.2 - 1.0;
		double  angledropbasket = 140, angledropbasket4 = 140, anglegetbrick1 = 280, anglegetbrick2 = 283, anglegetbrick3 = 286,
				angleparkcenter = 350, anglegetbrick4left = 95; //100

		Pose2d beginPose = new Pose2d(0, 0, 0);
		Pose2d endPoseDropbasketBegin = new Pose2d(xdropbasketbegin, ydropbasketbegin, Math.toRadians(angledropbasket));
		Pose2d endPoseDropbasket1 = new Pose2d(xdropbasket1, ydropbasket1, Math.toRadians(angledropbasket));
		Pose2d endPoseDropbasket2 = new Pose2d(xdropbasket2, ydropbasket2, Math.toRadians(angledropbasket));
		Pose2d endPoseDropbasket3 = new Pose2d(xdropbasket3, ydropbasket3, Math.toRadians(angledropbasket));
		Pose2d endPoseDropbasket4 = new Pose2d(xdropbasket4, ydropbasket4, Math.toRadians(angledropbasket));
		Pose2d endPoseGetBrick1 = new Pose2d(xgetbrick1, ygetbrick1, Math.toRadians(anglegetbrick1));
		Pose2d endPoseGetBrick2 = new Pose2d(xgetbrick2, ygetbrick2, Math.toRadians(anglegetbrick2));
		Pose2d endPoseGetBrick3 = new Pose2d(xgetbrick3, ygetbrick3, Math.toRadians(anglegetbrick3));
		//Pose2d endPoseGetBrick4left = new Pose2d(xgetbrick4left, ygetbrick4left, Math.toRadians(anglegetbrick4left));
		Pose2d endPoseParkCenterFast = new Pose2d(xparkcenterfast, yparkcenterfast, Math.toRadians(angleparkcenter));

		String name;
		Trajectory t;
		TrajectoryActionBuilder tab;
		Action action;

		/*
		name = "dropbasketandall";
		tab = drive.actionBuilder(beginPose)
				.strafeToLinearHeading(new Vector2d(xdropbasket1, ydropbasket1), Math.toRadians(angledropbasket));
				.waitSeconds(1.5)
				.strafeToLinearHeading(new Vector2d(xgetbrick1, ygetbrick1middle), Math.toRadians(anglegetbrick1))
				.strafeToLinearHeading(new Vector2d(xgetbrick1, ygetbrick1), Math.toRadians(anglegetbrick1))
				.waitSeconds(1.5)
				.strafeToLinearHeading(new Vector2d(xdropbasket, ydropbasket), Math.toRadians(angledropbasket))
				.waitSeconds(1.5)
				.strafeToLinearHeading(new Vector2d(xgetbrick2, ygetbrick2), Math.toRadians(anglegetbrick2))
				.waitSeconds(1.5)
				.strafeToLinearHeading(new Vector2d(xdropbasket, ydropbasket), Math.toRadians(angledropbasket))
				.waitSeconds(1.5)
				.strafeToLinearHeading(new Vector2d(xgetbrick3, ygetbrick3), Math.toRadians(anglegetbrick3))
				.waitSeconds(1.5)
				.strafeToLinearHeading(new Vector2d(xdropbasket, ydropbasket), Math.toRadians(angledropbasket))
				.waitSeconds(1.5)
				.strafeToLinearHeading(new Vector2d(xparkcenter, yparkcenter1), Math.toRadians(angleparkcenter))
				.strafeToLinearHeading(new Vector2d(xparkcenter, yparkcenter), Math.toRadians(angleparkcenter));
		rrActionsName.add(name);
		rrActions.add(tab.build());
		 */

		name = "dropbasketbegin";
		tab = drive.actionBuilder(beginPose)
				.strafeToLinearHeading(new Vector2d(xdropbasket1, ydropbasket1), Math.toRadians(angledropbasket));
				//.strafeToLinearHeading(new Vector2d(xdropbasket4, ydropbasket4), Math.toRadians(angledropbasket));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "manualbegin";
		tab = drive.actionBuilder(beginPose)
				.strafeToLinearHeading(new Vector2d(1, 0), Math.toRadians(0));
		//.strafeToLinearHeading(new Vector2d(xdropbasket4, ydropbasket4), Math.toRadians(angledropbasket));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "getbrick1frombasket";
		tab = drive.actionBuilder(endPoseDropbasketBegin)
				.splineToLinearHeading(endPoseGetBrick1, 45);
				//.strafeToLinearHeading(new Vector2d(xgetbrick1middle, ygetbrick1middle), Math.toRadians(anglegetbrick1))
				//.strafeToLinearHeading(new Vector2d(xgetbrick1, ygetbrick1), Math.toRadians(anglegetbrick1));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "dropbasket1";
		tab = drive.actionBuilder(endPoseGetBrick1)
				.strafeToLinearHeading(new Vector2d(xdropbasket1, ydropbasket1), Math.toRadians(angledropbasket));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "getbrick2frombasket";
		tab = drive.actionBuilder(endPoseDropbasket1)
				.strafeToLinearHeading(new Vector2d(xgetbrick2, ygetbrick2), Math.toRadians(anglegetbrick2));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "dropbasket2";
		tab = drive.actionBuilder(endPoseGetBrick2)
				.strafeToLinearHeading(new Vector2d(xdropbasket2, ydropbasket2), Math.toRadians(angledropbasket));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "getbrick3frombasket";
		tab = drive.actionBuilder(endPoseDropbasket2)
				.strafeToLinearHeading(new Vector2d(xgetbrick3, ygetbrick3), Math.toRadians(anglegetbrick3));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "dropbasket3";
		tab = drive.actionBuilder(endPoseGetBrick3)
				.strafeToLinearHeading(new Vector2d(xdropbasket3, ydropbasket3), Math.toRadians(angledropbasket));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		/*
		name = "getbrick4frombasket"; //not used, replaced by the fast version
		tab = drive.actionBuilder(endPoseDropbasket3)
				.strafeToLinearHeading(new Vector2d(xgetbrick4left, ygetbrick4left), Math.toRadians(anglegetbrick4left));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "dropbasket4"; // not used, replaced by the fast version
		tab = drive.actionBuilder(endPoseGetBrick4left)
				.strafeToLinearHeading(new Vector2d(xdropbasket4, ydropbasket4), Math.toRadians(angledropbasket4));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "parkcenter"; // not used, replaced by the fast version
		tab = drive.actionBuilder(endPoseDropbasket4)
				.strafeToLinearHeading(new Vector2d(xparkcenter, yparkcenter1), Math.toRadians(angleparkcenter))
				.strafeToLinearHeading(new Vector2d(xparkcenter, yparkcenter), Math.toRadians(angleparkcenter));
		rrActionsName.add(name);
		rrActions.add(tab.build());
		 */

		name = "getbrick4frombasketfast";
		tab = drive.actionBuilder(endPoseDropbasket3)
				.strafeToLinearHeading(new Vector2d(xgetbrick4left + xdelta, ygetbrick4leftfast), Math.toRadians(anglegetbrick4left),
				        new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-100, 100));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "dropbasket4fast";
		tab = drive.actionBuilder(new Pose2d(xgetbrick4left + xdelta, ygetbrick4leftfast, Math.toRadians(anglegetbrick4left)))
				.splineToLinearHeading(endPoseDropbasket4, -45,
				//.strafeToLinearHeading(new Vector2d(xdropbasket4, ydropbasket4), Math.toRadians(angledropbasket4),
					new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-100, 100));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "parkcenterfast";
		//tab = drive.actionBuilder(new Pose2d(xdropbasket4, ydropbasket4, Math.toRadians(angledropbasket)))
		//		.strafeToLinearHeading(new Vector2d(xparkcenterfast, yparkcenterfast), Math.toRadians(angleparkcenter),
		//				new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-100, 100))
		//		.strafeToLinearHeading(new Vector2d(xparkcenterfast, yparkcenterfast - 11 - 2.0), Math.toRadians(angleparkcenter),
		//				new TranslationalVelConstraint(75.0), new ProfileAccelConstraint(-30, 50));
		tab = drive.actionBuilder(new Pose2d(xdropbasket4, ydropbasket4, Math.toRadians(angledropbasket)))
				.splineToLinearHeading(endPoseParkCenterFast, -45);
				//.strafeToLinearHeading(new Vector2d(xparkcenterfast, yparkcenterfast), Math.toRadians(angleparkcenter));
						//new TranslationalVelConstraint(75.0), new ProfileAccelConstraint(-100, 100));
		rrActionsName.add(name);
		rrActions.add(tab.build());
	}

	private void InitRRActionsFar(MecanumDrive drive) {
		double  xhang = 32.5 + 1.0 - 1.0, xhang1 = xhang - 1.5, xhang2 = xhang - 1.5, xfarbrickmiddle = 26.8 - 5.0,
				xfarbrick1 = 51.85, xfarbrick2 = xfarbrick1, xfarbrick3 = xfarbrick2, xgetclip1 = 16.5, xgetclip2 = xgetclip1,
				xfarhome = 9.5, xpark = 6.5 + 3.0;
		double  yhang = 20.5, yhang1 = 17.5, yhang2 = 14.5, yfarbrickmiddle = -16.4 + 7.0 - 5.0,
				yfarbrick1 = -30.17, ypark = yfarbrick1, yfarbrick2 = -44.2157 + 3.0, yfarbrick3 = -48,
				yfarbrick3home = -46, ygetclip1 = -30.5, ygetclip2 = ygetclip1 - 1.0;
		// try to push the brick just use the front side (intake side),
		double anglehang = 0, anglefarbrickmiddle = 0.0, anglehang1 = 0.0, anglefarbrick1 = 0.0, anglefarbrick3 = 0.0, anglegetfarclip1 = 0.0;

		String name;
		Trajectory t;
		TrajectoryActionBuilder tab;
		Action action;

		Pose2d beginPose = new Pose2d(0, 0, 0);
		Pose2d endPose = new Pose2d(0,0,0);
		Pose2d endPoseHangBegin = new Pose2d(xhang, yhang, Math.toRadians(anglehang));
		Pose2d endPoseHang1 = new Pose2d(xhang1, yhang1, Math.toRadians(anglehang1));
		Pose2d endPoseHang2 = new Pose2d(xhang2, yhang2, Math.toRadians(anglehang1));
		Pose2d endPoseGetClip1 = new Pose2d(xgetclip1, ygetclip1, Math.toRadians(anglegetfarclip1));
		Pose2d endPoseGetClip2 = new Pose2d(xgetclip2, ygetclip2, Math.toRadians(anglegetfarclip1));

		name = "hangclipbegin";
		tab = drive.actionBuilder(beginPose)
				.strafeTo(new Vector2d(xhang, yhang));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		/*
		// not used as push brick 2
		name = "getclip1";
		tab = drive.actionBuilder(endPoseHang)
				.strafeToLinearHeading(new Vector2d(xgetclip, ygetclip), Math.toRadians(anglegetfarclip1));
		rrActionsName.add(name);
		rrActions.add(tab.build());
		 */

		name = "farbrick2";
		tab = drive.actionBuilder(endPoseHangBegin)
				.strafeToLinearHeading(new Vector2d(xfarbrickmiddle, yfarbrickmiddle), Math.toRadians(anglehang),
						new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-30, 50))
				.strafeToLinearHeading(new Vector2d(xfarbrick1, yfarbrick2), Math.toRadians(anglefarbrick1),
						new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-30, 50))
				.strafeToLinearHeading(new Vector2d(xfarhome, yfarbrick2), Math.toRadians(anglefarbrick1),// push brick2
						new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-30, 50))
				.strafeToLinearHeading(new Vector2d(xgetclip1, ygetclip1), Math.toRadians(anglegetfarclip1)); // go to pick clip position
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "hangclip1";
		tab = drive.actionBuilder(endPoseGetClip1)
				.strafeToLinearHeading(new Vector2d(xhang1, yhang1), Math.toRadians(anglehang));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "getclip2";
		tab = drive.actionBuilder(endPoseHang1)
				.strafeToLinearHeading(new Vector2d(xgetclip2, ygetclip2), Math.toRadians(anglegetfarclip1));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "hangclip2";
		tab = drive.actionBuilder(endPoseGetClip2)
				.strafeToLinearHeading(new Vector2d(xhang2, yhang2), Math.toRadians(anglehang));
		rrActionsName.add(name);
		rrActions.add(tab.build());

		name = "parkfarfromhang";
		tab = drive.actionBuilder(endPoseHang2)
				.strafeTo(new Vector2d(xpark, ypark),
						new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-100, 100));
		rrActionsName.add(name);
		rrActions.add(tab.build());
	}

	@Override
	void InitRRActions(MecanumDrive drive) {
		InitRRActionsNear(drive);
		InitRRActionsFar(drive);
	}

}
