package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.roadrunner.Action;

public class RobotDataArm extends RobotDataBase {

    public RobotDataArm(boolean autoMode, String workingMode, boolean debugMode) {

        type = "arm";
        this.autoMode = autoMode;
        this.workingMode = workingMode;
        this.debugMode = debugMode;
        useLifter = false;
        useDeadWheel = false;

        // TODO: roadrunner params are set at class RoadRunnerTuningParams which is at RobotDataBase.java
        //       Actions/TrajectoryActionBuilder are defined inside abstract function InitRRActions()

        // TODO: if have some values used multiple time, can be defined with a name here
        generalStringParams = new String[][] {
                {"sliderpickpowerin",     "-0.25"},
                {"sliderpickpowerout",     "0.25"},
                {"sliderpowerpickback", "-0.5"},
                {"sliderpowerdown",     "0.2"},
                {"sliderpowerhold",     "0.0005"},
                {"sliderhightime",  "800"},
        };

        //
        // TODO: configure the general components like servo and mortor, crservo
        //
        ServoInfo[] servolist = new ServoInfo[] {
            new ServoInfo("leftpto", true, 0.0, 1.0,"up=0.659", "down=0.349", "", "", "", "", "", ""),
            new ServoInfo("rightpto",   true, 0.0, 1.0,"up=0.209", "down=0.449", "", "", "", "", "", ""),
            new ServoInfo("lowrotate",   true, 0.0, 1.0,
                  "straight=0.53", "exchange=0.53", "ready=0.53", "drop=0.50", "getbrick3=0.71", "backpick=0.0", "", ""),
            new ServoInfo("lowwrist",   true, 0.0, 1.0,
                  "pick=0.42", "exchange=0.649", "ready=0.649", "exchangeprepare=0.669", "", "picknear=0.41", "", ""),
            new ServoInfo("lowarm",   true, 0.0, 1.0,
                  "pick=0.489", "exchange=0.189", "ready=0.189", "exchangeprepare=0.159", "getbrick3=0.71", "backpick=0.0", "", ""),
            new ServoInfo("grip",   true, 0.0, 1.0,
                        "exchange=0.37", "ready=0.52", "close=0.63", "drop=0.37", "beforeclose=0.61", "open=0.37", "", ""),
            new ServoInfo("highrotate",   true, 0.0, 1.0,
						//"exchange=0.768", "ready=0.768", "drop=0.469", "getclip=0.479", "hangclip=0.479", "", "", ""),
					"exchange=0.337", "ready=0.337", "drop=0.618", "getclip=0.618", "hangclip=0.618", "", "", ""),
            new ServoInfo("highwrist",   true, 0.0, 1.0,
                  "exchange=0.688", "ready=0.688", "pullsample=0.560", "drop=0.04", "throw=0.25", "getclip=0.26", "hangclip=0.257", "hangclipdrag=0.357"),
            new ServoInfo("higharm",   true, 0.0, 1.0,
                  "exchange=0.91", "ready=0.91", "pullsample=0.22", "drop=0.97", "getclip=0.83", "hangclip=1.00", "", ""),
        };
        this.servolist = servolist;

        CRServoInfo[] crservolist = new CRServoInfo[] {
             new CRServoInfo("intake",       -1.00,           1.000,         false,      false),
        };
        this.crservolist = crservolist;

        MotorInfo[] motorlist = new MotorInfo[] {
             new MotorInfo("frontleft", true, true,100, 1200, null,"", "", "", "", ""),
             new MotorInfo("frontright",false, true,100, 2780, null,"", "", "", "", ""),
             new MotorInfo("rearleft", true, true,100, 1200, null,"", "", "", "", ""),
             new MotorInfo("rearright",false, true,100, 2780, null,"", "", "", "", ""),
             //new MotorInfo("sliderpickfront",true, true,100, 2780, null,"", "", "", "", ""),
             new MotorInfo("sliderpickrear",true, true,100, 2780, null,"", "", "", "", ""),
             new MotorInfo("sliderdropfront",true, true,100, 2780, null,"hangclip=400", "", "", "", ""),
             //new MotorInfo("sliderdroprear",false, true,100, 2780, null,"", "", "", "", ""),
        };
        this.motorlist = motorlist;

        this.colorsensorlist = new ColorSensorInfo[] {
                //new ColorSensorInfo("pickcolor", true),
        };

        this.distancesensorlist = new DistanceSensorInfo[] {};

        //
        // TODO: configure the script bind to the keys or combo keys
        //
        presetActionsAutoModeInitBeforeStart = new String[]{
				"servo leftpto up",
				"servo rightpto up",
				"servo higharm ready",
				"servo highwrist ready",
				"servo highrotate ready",
				"servo grip ready",
				"sleep 500",
				"servo lowarm ready",
				"servo lowwrist ready",
				"servo lowrotate ready",
        };
        presetActionsManualModeInitBeforeStart = new String[]{
				"servo leftpto up",
				"servo rightpto up",
				"servo higharm exchange",
				"servo highwrist exchange",
				"servo highrotate exchange",
				"servo grip close",
				"sleep 500",
				"servo lowarm exchange",
				"servo lowwrist exchange",
				"servo lowrotate exchange",
        };

        presetActionsInitAfterStartNonAutoMode = new String[]{
                //"set @imu",
                debugMode ? "exit" : "",
        };

        presetActionsRedNear = new String[]{
        };

        presetActionsRedFar = new String[]{
        };

        presetActionsBlueNear = presetActionsRedNear;

        presetActionsBlueFar = presetActionsRedFar;

        presetActionsPad1Up = new String[]{
                "if oneinstance sliderup == off", // when there is another instance running, will not run even hit the button
                    "set oneinstance sliderup 1000", // until 1000ms later, another instance can be run
                    "thread nextstep presetActionsSliderUpHighBasket 900",
                "exit",
        };

        presetActionsPad1Down = new String[]{
				"motor sliderdropfront -1.0",
				"motor sliderdroprear -1.0",
				"sleep %%arg1=800",
				"motor sliderdropfront 0.0",
				"motor sliderdroprear 0.0",

                "exit",

                "motor frontleft 1.0",
                "motor frontright 1.0",
                "motor rearleft 1.0",
                "motor rearright 1.0",
                "sleep 100",
                "motor frontleft 0.0",
                "motor frontright 0.0",
                "motor rearleft 0.0",
                "motor rearright 0.0",
        };

        presetActionsPad1Left = new String[]{
                "if oneinstance sliderup == off", // when there is another instance running, will not run even hit the button
                    "set oneinstance sliderup 1000", // until 1000ms later, another instance can be run
                    "thread nextstep presetActionsSliderUpHighBasket 300 0.0",

                "exit",
                "motor frontleft 0.4",
                "motor frontright 0.4",
                "motor rearleft 0.4",
                "motor rearright 0.4",
        };

        presetActionsPad1Right = new String[]{
                "if oneinstance sliderup == off", // when there is another instance running, will not run even hit the button
                    "set oneinstance sliderup 1000", // until 1000ms later, another instance can be run
                    "thread nextstep presetActionsSliderUpHighBasket 500 0.0",

                "exit",
        };

        presetActionsPad1X = new String[]{
				"servo grip close",
				"motor sliderdropfront 1.0",
				"motor sliderdropback 1.0",
				"sleep 390",
				"motor sliderdropfront sliderpowerhold",
				"motor sliderdropback sliderpowerhold",

                "exit",

                "servo higharm exchange",
                "servo highrotate exchange",
                "servo highwrist exchange",
                "servo grip exchange",

                "crservo intake 0.0",
                "servo lowrotate exchange",
                "servo lowwrist exchange",
                "servo lowarm exchange",
        };

        presetActionsPad1Y = new String[]{
                "if oneinstance exchange == off", // when there is another instance running, will not run even hit the button
                    "set oneinstance exchange 1500", // until 1500ms later, another instance can be run
                    "thread nextstep presetActionsExchange 800 300",
        };

        presetActionsPad1A = new String[]{
                "motor sliderpickfront 1.0",
                "motor sliderpickrear 1.0",
                "sleep 200",
                "motor sliderpickfront 0.0",
                "motor sliderpickrear 0.0",
                "servo lowarm pick",
                "servo lowwrist pick",
                "servo lowrotate straight",
                "crservo intake -0.5",
        };

        presetActionsPad1B = new String[]{
                "if oneinstance dropbrick == off", // when there is another instance running, will not run even hit the button
                    "set oneinstance dropbrick 1000", // until 1000ms later, another instance can be run.
                    "thread nextstep presetActionsDropBrick",

                "exit",
                "servo grip beforeclose",
                "crservo intake -0.5",
                "sleep 100",
                "servo grip close",
                "servo highwrist pullsample",
                "sleep 100",
                "servo higharm drop",
                "servo highwrist drop",
                "crservo intake 0.0",
        };

        presetActionsPad1UpWithRightBumper = new String[]{
				"servo leftpto 0.01 delta",
        };

        presetActionsPad1DownWithRightBumper = new String[]{
				"servo leftpto -0.01 delta",
        };

        presetActionsPad1LeftWithRightBumper = new String[]{
				"servo rightpto 0.01 delta",
        };
        presetActionsPad1RightWithRightBumper = new String[]{
				"servo rightpto -0.01 delta",
        };
        presetActionsPad1UpWithLeftBumper = new String[]{
        };
        presetActionsPad1DownWithLeftBumper = new String[]{
        };
        presetActionsPad1LeftWithLeftBumper = new String[]{
        };
        presetActionsPad1RightWithLeftBumper = new String[]{
        };
        presetActionsPad1XWithLeftBumper = new String[]{
                "servo highrotate 0.01 delta"
        };
        presetActionsPad1YWithLeftBumper = new String[]{
                "servo highwrist 0.01 delta"
        };
        presetActionsPad1AWithLeftBumper = new String[]{
                "servo highwrist -0.01 delta"
        };

        presetActionsPad1BWithLeftBumper = new String[]{
                "servo highrotate -0.01 delta"
        };

        presetActionsPad1XWithRightBumper = new String[]{
                "servo grip 0.01 delta"
        };

        presetActionsPad1YWithRightBumper = new String[]{
                "servo higharm 0.01 delta"
        };
        presetActionsPad1AWithRightBumper = new String[]{
                "servo higharm -0.01 delta"
        };
        presetActionsPad1BWithRightBumper = new String[]{
                "servo grip -0.01 delta"
        };

        presetActionsPad1Back = new String[]{
                "log",
        };

        presetActionsPad1Start = new String[] {
        };

        presetActionsPad1LeftTrigger = new String[]{
                "motor sliderpickfront sliderpickpowerin",
                "motor sliderpickrear sliderpickpowerin",
                "sleep 1",
                "if key pad1lefttrigger == off", // when still hold the key, will return as check fail here
                    "motor sliderpickfront 0.0",  // only stop motor when key is off
                    "motor sliderpickrear 0.0",
        };

        presetActionsPad1RightTrigger = new String[]{
                "motor sliderpickfront sliderpickpowerout",
                "motor sliderpickrear sliderpickpowerout",
                "sleep 1",
                "if key pad1righttrigger == off", // when still hold the key, will return as check fail here
                    "motor sliderpickfront 0.0",  // only stop motor when key is off
                    "motor sliderpickrear 0.0",
        };

        presetActionsPad1LeftBumper = new String[]{
        };
        presetActionsPad1RightBumper = new String[]{
        };

        presetActionsPad2Up = new String[]{
				"if oneinstance getclipup == off", // when there is another instance running, will not run even hit the button
					"set oneinstance getclipup 1000", // until 1000ms later, another instance can be run.
					"servo grip close",
					"sleep 200",
					//"thread motor sliderdropfront 0.8 runtoposition hangclip 1500 hold",
					"motor sliderdropfront 1.0",
					"motor sliderdropback 1.0",
					"sleep 390",
					"motor sliderdropfront sliderpowerhold",
					"motor sliderdropback sliderpowerhold",
					"servo higharm hangclip",
					"servo highwrist hangclip",
					"servo highrotate hangclip",

				"exit",
                "motor sliderdropfront 0.4",
                "motor sliderdroprear 0.4",
                "sleep 200",
                "motor sliderdropfront sliderpowerhold",
                "motor sliderdroprear sliderpowerhold",
        };
        presetActionsPad2Down = new String[]{
                "motor sliderdropfront -0.4",
                "motor sliderdroprear -0.4",
                "sleep 200",
                "motor sliderdropfront 0.0",
                "motor sliderdroprear 0.0",
        };
        presetActionsPad2Left = new String[]{
				"if oneinstance getclip == off", // when there is another instance running, will not run even hit the button
					"set oneinstance getclip 1000", // until 1000ms later, another instance can be run.
					"thread nextstep presetActionsGetClip",
        };
        presetActionsPad2Right = new String[]{
				"if oneinstance hangclip == off", // when there is another instance running, will not run even hit the button
					"set oneinstance hangclip 1000", // until 1000ms later, another instance can be run.
					"thread nextstep presetActionsHangClip"
        };

        presetActionsPad2UpWithLeftBumper = new String[]{
        };
        presetActionsPad2DownWithLeftBumper = new String[]{
        };
        presetActionsPad2LeftWithLeftBumper = new String[]{
        };
        presetActionsPad2RightWithLeftBumper = new String[]{
        };

        presetActionsPad2UpWithRightBumper = new String[]{
        };
        presetActionsPad2DownWithRightBumper = new String[]{
        };

        presetActionsPad2LeftWithRightBumper = new String[]{
                "manualwheel disable",
        };
        presetActionsPad2RightWithRightBumper = new String[]{
        };

        presetActionsPad2X = new String[]{
				"motor frontleft 0.0",
				"motor frontright 0.0",
				"motor rearleft 0.0",
				"motor rearright 0.0",
        };
        presetActionsPad2Y = new String[]{

				"motor frontleft 0.2",
				"motor frontright 0.2",
				"motor rearleft 0.2",
				"motor rearright 0.2",
				"sleep 200",
				"motor frontleft 0.0",
				"motor frontright 0.0",
				"motor rearleft 0.0",
				"motor rearright 0.0",
        };
        presetActionsPad2A = new String[]{
				"motor frontleft 0.2",
				"motor frontright 0.2",
				"motor rearleft 0.2",
				"motor rearright 0.2",
        };
        presetActionsPad2B = new String[]{
				"servo higharm exchange",
				"servo highrotate exchange",
				"servo highwrist exchange",
				"servo grip exchange",

				"servo lowrotate exchange",
				//"servo lowwrist exchangeprepare",
				//"servo lowarm exchangeprepare",
				"servo lowwrist exchange",
				"servo lowarm exchange",
				"crservo intake 0.0",
				"sleep %%arg2=100",

				//"servo lowwrist exchange",
				//"servo lowarm exchange",
				//"sleep 100",
       };

        presetActionsPad2XWithRightBumper = new String[]{
                "servo lowwrist 0.01 delta",
        };

        presetActionsPad2YWithRightBumper = new String[]{
                "servo lowarm 0.01 delta",
        };
        presetActionsPad2AWithRightBumper = new String[]{
                "servo lowarm -0.01 delta",
       };
        presetActionsPad2BWithRightBumper = new String[]{
                "servo lowwrist -0.01 delta",
        };
        presetActionsPad2XWithLeftBumper = new String[]{
                "servo lowrotate 0.01 delta",
        };
        presetActionsPad2YWithLeftBumper = new String[]{
                "manualwheel disable",
        };
        presetActionsPad2AWithLeftBumper = new String[]{
                "manualwheel disable",
                //"rr dropbasketdynamic strafetolinear",
        };
        presetActionsPad2BWithLeftBumper = new String[]{
                "servo lowrotate -0.01 delta",
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
                "servo leftpto down",
                "servo rightpto down",
				"manualwheel disable",
        };
        presetActionsPad2RightStick = new String[]{
				"servo leftpto up",
				"servo rightpto up",
				"manualwheel enable",
        };
        presetActionsPad2LeftTrigger = new String[]{
				"servo grip drop",
				"crservo intake 0.5 1000",
        };
        presetActionsPad2RightTrigger = new String[]{
				"servo grip close",
				"crservo intake -0.5 1000",
        };
        presetActionsPad2Back = new String[]{
                "log"
        };
        presetActionsPad2BackWithLeftBumper = new String[]{
                "set debug on",
        };
        presetActionsPad2BackWithRightBumper = new String[]{
                "set debug off",
        };
        presetActionsPad2Start = new String[]{
        };
        presetActionsDefault = new String[]{
        };
        presetActionsPad2LeftstickRightStick = new String[]{
        };

        // TODO: define local preset actions and add to the nextstepmap to get it able to be called
		presetActionsManualCommand1 = new String[] {
				//"motor hanglow -1.0 sliderhanglowafterautotime",
				//"motor hanglow initposition",
		};

		presetActionsManualAfter10Seconds = new String[] {
				//"thread nextstep presetActionsManualCommand1",
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

		String[] presetActionsGetClip = new String[] {
				"servo higharm getclip",
				"servo highwrist getclip",
				"servo highrotate getclip",
				"servo grip open",
		};

		String[] presetActionsHangClip = new String[]{
				//"servo higharm hangclip",
				//"servo highwrist hangclip",
				//"servo highrotate hangclip",

				//"motor sliderdropfront 1.0",
				//"motor sliderdropback 1.0",
				//"sleep 390",
				//"motor sliderdropfront sliderpowerhold",
				//"motor sliderdropback sliderpowerhold",
				//"sleep 200", // wait for the servo to reach its position as it might be slow for torque servo

				//"motor sliderdropfront -0.5",
				//"motor sliderdropback -0.5",
				//"servo highwrist 0.82",
				//"sleep 200",
				//"motor sliderdropfront sliderpowerhold",
				//"motor sliderdropback sliderpowerhold",
				//"sleep 200",
				"motor sliderdropfront -0.5",
				"motor sliderdropback -0.5",
				"sleep 100",
				"servo highwrist hangclipdrag",
				"sleep 200",
				"motor sliderdropfront sliderpowerhold",
				"motor sliderdropback sliderpowerhold",
				"servo grip drop",
				"sleep 200",
				"servo higharm exchange",
				"servo highwrist exchange",
				"servo highrotate exchange",
				"motor sliderdropfront -1.0",
				"motor sliderdropback -1.0",
				"sleep 250",
				"motor sliderdropfront 0.0",
				"motor sliderdropback 0.0",
		};

		String[] presetActionsExchange = new String[]{
                "servo higharm exchange",
                "servo highrotate exchange",
                "servo highwrist exchange",
                "servo grip exchange",

                "servo lowrotate exchange",
				//"servo lowwrist exchangeprepare",
				"servo lowarm exchangeprepare", // to prevent hit the edge of the slider, like screws
                "servo lowwrist exchange",
                //"servo lowarm exchange",
                "crservo intake 0.0",
                "sleep %%arg2=100",

				"motor sliderpickfront -0.5",
				"motor sliderpickrear -0.5",
				"sleep %%arg1=800",
				"motor sliderpickfront 0.0",
				"motor sliderpickrear 0.0",

				//"servo lowwrist exchange",
				//"servo lowarm exchange",
				//"sleep 100",

				//"exit",

				"servo lowarm exchange",
				"sleep 200",

                "servo grip beforeclose",
                "crservo intake -0.5",
                "sleep 100",
                "servo grip close",
				"sleep 50",
                "servo highwrist pullsample",
                "sleep %%arg3=50",
                "servo higharm drop",
                "servo highwrist drop",
                "crservo intake 0.0",
				"sleep 200",
				"servo highrotate drop"
		};

		String[] presetActionsDropBasket = new String[] {
		};

		String[] presetActionsDropBrick = new String[] {
				"servo highwrist throw",
				"sleep %%arg1=50",
				"servo grip drop",
				"sleep %%arg2=200",
				"servo highwrist drop",
				"servo higharm exchange",
				"servo highrotate exchange",
				"sleep %%arg3=100",
				"motor sliderdropfront -1.0",
				"motor sliderdroprear -1.0",
				"sleep sliderhightime",
				"motor sliderdropfront 0.0",
				"motor sliderdroprear 0.0",
				"servo highwrist exchange",
		};

		String[] presetActionsAscent1 = new String[] {

		};

        String[] presetActionsSliderUpHighBasket = new String[] {
                "motor sliderdropfront 1.0",
                "motor sliderdroprear 1.0",
                //"servo higharm drop", // servo change to closer position to drop
                //"servo highwrist drop", // servo change to closer position to drop
                //"servo highrotate straight",
                "sleep %%arg1=sliderhightime",
                "motor sliderdropfront %%arg2=sliderpowerhold",
                "motor sliderdroprear %%arg2=sliderpowerhold",
        };

		InitBase();
		nextstepmap.put("presetActionsGetBrickStraight", presetActionsGetBrickStraight);
		nextstepmap.put("presetActionsGetBrick4", presetActionsGetBrick4);
		nextstepmap.put("presetActionsExchange", presetActionsExchange);
		nextstepmap.put("presetActionsHangClip", presetActionsHangClip);
		nextstepmap.put("presetActionsDropBasket", presetActionsDropBasket);
		nextstepmap.put("presetActionsDropBrick", presetActionsDropBrick);
		nextstepmap.put("presetActionsAscent1", presetActionsAscent1);
		nextstepmap.put("presetActionsGetClip", presetActionsGetClip);
        nextstepmap.put("presetActionsSliderUpHighBasket", presetActionsSliderUpHighBasket);
    }

    @Override
    void InitRRActions(MecanumDrive drive) {

        Pose2d beginPose = new Pose2d(0, 0, 0);
        double xhang = 26, xgetbricks = 39, xdropbasket = 8.0, xdropbasketmiddlepoint = 12; //7.8; //12;
        double yhang = -32, ygetbrick1 = 4, ydropbasket = 19.6; //20.2; //22;
        double xfarhome = 5, xfar = 45, xfarmiddle = 40;
        double yfarbrick1 = -10.0, yfarbrick2 = -20.0, yfarbrick3 = -24;

        String name;
        Trajectory t;
        TrajectoryActionBuilder tab;
        Action action;
        Pose2d endPose = new Pose2d(0,0,0);

        name = "attachhigh";
        tab = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(xhang, yhang, Math.PI), 0);
        endPose = new Pose2d(xhang, yhang, Math.PI);
        //action = tab.build();

        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "getbrick1";
        tab = drive.actionBuilder(endPose)
            .splineToLinearHeading(new Pose2d(xhang - 6, yhang, Math.PI), 0)
            //.splineTo(new Vector2d(xhang - 6, ygetbrick1), Math.PI / 2)
            .splineToLinearHeading(new Pose2d(xhang - 6, ygetbrick1, Math.PI/2), 0)
            .splineToLinearHeading(new Pose2d(xgetbricks, ygetbrick1, Math.PI/2), 0);
        endPose = new Pose2d(xgetbricks, ygetbrick1, Math.PI / 2);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "dropbasketbegin";
        tab = drive.actionBuilder(beginPose)
                // this will hit the wall
                .splineToLinearHeading(new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4),0);
                //.splineToLinearHeading(new Pose2d(24, 24, -Math.PI / 4),0)
                //.lineToXConstantHeading(10.8); // back to the position
        endPose = new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "dropbasketbeginwithmiddlepoint";
        tab = drive.actionBuilder(beginPose)
            .splineToLinearHeading(new Pose2d(xdropbasketmiddlepoint, ydropbasket, -Math.PI / 4),0,
                new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-30, 100))
            //.splineToLinearHeading(new Pose2d(xdropbasketmiddlepoint, ydropbasket, -Math.PI / 4),0)
            .lineToXConstantHeading(xdropbasket); // back to the position
        endPose = new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "getbrick1basket";
        tab = drive.actionBuilder(endPose)
                .splineToLinearHeading(new Pose2d(xgetbricks + 2, ygetbrick1, Math.toRadians(96)), 0,
                        // this will cause X error > 3 inches, and heading error > 6 degrees. change radians to 96, and add 2 to x.
                        new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-30, 100));
        endPose = new Pose2d(xgetbricks, ygetbrick1, Math.PI / 2);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        /*
        name = "dropbasket1reverse"; // reverse not working as expected
        endPose = new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4);
        tab = drive.actionBuilder(endPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(xgetbricks, ygetbrick1, Math.PI/2), 0);
        endPose = new Pose2d(xgetbricks, ygetbrick1, Math.PI / 2);
        rrActionsName.add(name);
        rrActions.add(tab.build());
         */

        name = "dropbasket1";
        tab = drive.actionBuilder(endPose)
                //.splineToLinearHeading(new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4),0);
                .splineToLinearHeading(new Pose2d(xdropbasketmiddlepoint, ydropbasket, Math.toRadians(-45)),0)
                .lineToXConstantHeading(xdropbasket); // back to the position
        endPose = new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "getbrick2";
        tab = drive.actionBuilder(endPose)
                .splineToLinearHeading(new Pose2d(xgetbricks + 2, ygetbrick1 + 10, Math.toRadians(96)), 0,
                        // this will cause X error > 3 inches, and heading error > 6 degrees. change radians to 96, and add 2 to x.
                        new TranslationalVelConstraint(100.0), new ProfileAccelConstraint(-30, 100));
        endPose = new Pose2d(xgetbricks, ygetbrick1 + 10, Math.PI / 2);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "dropbasket2";
        tab = drive.actionBuilder(endPose)
                .splineToLinearHeading(new Pose2d(xdropbasket, ydropbasket, Math.toRadians(-45)),0);
                //.splineToLinearHeading(new Pose2d(xdropbasketmiddlepoint, ydropbasket, -Math.PI / 4),0)
                //.lineToXConstantHeading(xdropbasket); // back to the position
        endPose = new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "getbrick3";
        tab = drive.actionBuilder(endPose)
                .splineToLinearHeading(new Pose2d(xgetbricks + 2, ygetbrick1 + 20, Math.toRadians(96)),0);
        endPose = new Pose2d(xgetbricks, ygetbrick1 + 20, Math.PI / 2);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "dropbasket3";
        tab = drive.actionBuilder(endPose)
                .splineToLinearHeading(new Pose2d(xdropbasketmiddlepoint, ydropbasket, -Math.PI / 4),0)
                .lineToXConstantHeading(xdropbasket); // back to the position
        endPose = new Pose2d(xdropbasket, ydropbasket, -Math.PI / 4);
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "farbrick1";
        tab = drive.actionBuilder(beginPose)
                .lineToXConstantHeading(xfarmiddle)
                .splineToLinearHeading(new Pose2d(xfar, yfarbrick1, Math.toRadians(-90)), 0);
        endPose = new Pose2d(xfar, yfarbrick1, Math.toRadians(-90));
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "pushbrick1";
        tab = drive.actionBuilder(endPose)
                .setTangent(0)
                .lineToXConstantHeading(xfarhome);
        endPose = new Pose2d(xfarhome, yfarbrick1, Math.toRadians(-90));
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "farbrick2";
        tab = drive.actionBuilder(endPose)
                .setTangent(0)
                .lineToXConstantHeading(xfar)
                .splineToConstantHeading(new Vector2d(xfar, yfarbrick2), 0);
        endPose = new Pose2d(xfar, yfarbrick2, Math.toRadians(-90));
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "pushbrick2";
        tab = drive.actionBuilder(endPose)
                .setTangent(0)
                .lineToXConstantHeading(xfarhome);
        endPose = new Pose2d(xfarhome, yfarbrick2, Math.toRadians(-90));
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "farbrick3";
        tab = drive.actionBuilder(endPose)
                .setTangent(0)
                .lineToXConstantHeading(xfar)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(xfar, yfarbrick3), 0);
        endPose = new Pose2d(xfar, yfarbrick3, Math.toRadians(-90));
        rrActionsName.add(name);
        rrActions.add(tab.build());

        name = "pushbrick3";
        tab = drive.actionBuilder(endPose)
                .setTangent(0)
                .lineToXConstantHeading(xfarhome);
        endPose = new Pose2d(xfarhome, yfarbrick3, Math.toRadians(-90));
        rrActionsName.add(name);
        rrActions.add(tab.build());
    }

}
