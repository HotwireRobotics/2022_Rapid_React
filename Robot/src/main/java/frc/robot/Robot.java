package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DriverStation;

import java.applet.AudioClip;
import java.nio.Buffer;
import java.rmi.server.Operation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.hal.PDPJNI;
import frc.robot.autostep.*;
import edu.wpi.first.wpilibj.Compressor;
import java.util.*;

//import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

//BUTTONS
//		TELEOP

// 	Right: 
// ?6 Shooter with PowerAxis

// 	Left:
// 6 LL Forward
// 7 LL Backward

// ?8 Color Wheel # reset
// ?9 Color Wheel Spin X times

// ?10 Color Wheel Spin
// ?11 Color Wheel Find

//		TEST
//	Right:

//	Left:

public class Robot extends TimedRobot {

	// Sensors
	public AHRS navx = new AHRS(SPI.Port.kMXP);

	// private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
	public DigitalInput ColorWheelLimit = new DigitalInput(7);
	public float encoderSpeed1 = 0;
	public float encoderSpeed2 = 0;
	// public Encoder shooterEncoder = new Encoder();

	public boolean beamReset = true;

	// Drivetrain
	public DriveTrain driveTrain = new DriveTrain(54, 55, 56, 57, navx);

	// Nuemataics
	public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(4, 5);

	// Logic
	public boolean speedToggle = false;

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick flightStickLeft;
	public Joystick flightStickRight;

	public Limelight limelight = new Limelight();
	public Shooter shooter = new Shooter(limelight);
	public Indexer indexer = new Indexer(shooter);
	public Climber climber = new Climber();

	// Motors
	public TalonSRX intakeSeven = new TalonSRX(5);

	public TalonSRX MotorSeven = new TalonSRX(30);
	public TalonSRX MotorEight = new TalonSRX(31);
	public TalonSRX ColorTwo = new TalonSRX(2);

	boolean limitPressed;

	public enum DriveScale {
		linear, parabala, tangent, inverse, cb, cbrt,
	}

	enum AutoChoice {
		// Left, Right
	}

	enum ColorWheel {
		Red, Green, Blue, Yellow, Unknown, Broken;
	}

	public ColorWheel LastColor;
	public int colorChangsCount;

	public DriverStation driverStation;
	public RobotState currentState;

	// Auto
	public LinkedList<AutoStep> autonomousSelected;
	// start on the line, backup, and shoot
	public LinkedList<AutoStep> mostBasicShoot;
	// start on the right, get two balls, and shoot five
	public LinkedList<AutoStep> rightFiveBall;
	// start on line then move back grab 2 balls from center, then go to trench to
	// grab three balls
	public LinkedList<AutoStep> centerEightBall;

	public LinkedList<AutoStep> centerFiveBall;

	public LinkedList<AutoStep> rightSixBall;
	public int currentAutoStep;
	// public float auto = 0;

	public int accum;

	public String autoSelectKey = "autoMode";

	public void robotInit() {
		limelight.SetLight(false);
		shooter.Init();

		// SmartDashboard.putNumber("AutoMode", 0);
		indexer.indexerFive.set(ControlMode.PercentOutput, 0.0f);

		SmartDashboard.putNumber(autoSelectKey, 0);
	}

	public void disabledInit() {
		// Controllers
		driveTrain.SetBreak();
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void disabledPeriodic() {
		driveTrain.SendData();
		SendPDPData();
		SmartDashboard.putBoolean("RobotEnabled", false);
	}

	public void autonomousInit() {

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
		// autonomousSelected = SmartDashboard.getNumber("/SmartDashboard/autoMode", 0);

		limelight.AutoSettings();
		driveTrain.SetBreak();
		limelight.SetLight(true);

		indexer.ballCounter = 2;
		indexer.firstAutomatic = true;

		currentAutoStep = 0;

		// most basic shoot ---------
		mostBasicShoot = new LinkedList<AutoStep>();
		mostBasicShoot.add(new NavxReset(navx));
		mostBasicShoot.add(new ShooterRev(shooter, 4100.0));
		// Configuration Complete
		mostBasicShoot.add(new EncoderForward(driveTrain, 50000f, -0.15f));
		mostBasicShoot.add(new LimelightTrack(driveTrain, shooter, limelight, 0.0f));
		mostBasicShoot.add(new Shoot(shooter, indexer, 4100.0, 3));

		// right five ball ---------------
		rightFiveBall = new LinkedList<AutoStep>();
		rightFiveBall.add(new NavxReset(navx));
		rightFiveBall.add(new ShooterRev(shooter, 3900.0));
		rightFiveBall.add(new IntakeDrop(intakeSolenoid));
		rightFiveBall.add(new IntakeRun(intakeSeven, 1.0f));
		// Configuration Complete
		rightFiveBall.add(new EncoderForward(driveTrain, 120000f, -0.3f));
		// 5 balls intaked
		rightFiveBall.add(new NavxTurn(driveTrain, navx, -30f, 0.35f, 2f));
		rightFiveBall.add(new EncoderForward(driveTrain, 60000f, 0.5f));
		rightFiveBall.add(new NavxTurn(driveTrain, navx, 0, 0.3f, 3f));
		rightFiveBall.add(new Wait(driveTrain, 0.1f));
		// Shooting
		rightFiveBall.add(new LimelightTrack(driveTrain, shooter, limelight, 0.0f));
		rightFiveBall.add(new Shoot(shooter, indexer, 3900.0, 5));

		// right six ball ---------------
		rightSixBall = new LinkedList<AutoStep>();
		rightSixBall.add(new NavxReset(navx));
		rightSixBall.add(new ShooterRev(shooter, 3900.0));
		rightSixBall.add(new IntakeDrop(intakeSolenoid));
		rightSixBall.add(new IntakeRun(intakeSeven, 1.0f));
		// Configuration Complete
		rightSixBall.add(new EncoderForward(driveTrain, 120000f, -0.3f));
		// 5 balls intaked
		rightSixBall.add(new LimelightTrack(driveTrain, shooter, limelight, 0.0f));
		rightSixBall.add(new Shoot(shooter, indexer, 3900.0, 5));
		// shooting done

		centerFiveBall = new LinkedList<AutoStep>();
		centerFiveBall.add(new NavxReset(navx));
		centerFiveBall.add(new ShooterRev(shooter, 4100.0));
		centerFiveBall.add(new IntakeDrop(intakeSolenoid));
		centerFiveBall.add(new IntakeRun(intakeSeven, 1.0f));
		// Configuration Done
		centerFiveBall.add(new EncoderForward(driveTrain, 65000f, -0.3f));
		centerFiveBall.add(new NavxTurn(driveTrain, navx, -25, 0.4f, 4.0f));
		centerFiveBall.add(new EncoderForward(driveTrain, 10000f, -0.2f));
		centerFiveBall.add(new EncoderForward(driveTrain, 10000f, 0.2f));
		centerFiveBall.add(new NavxTurn(driveTrain, navx, -50, 0.4f, 4.0f));
		centerFiveBall.add(new EncoderForward(driveTrain, 20000f, -0.2f));
		centerFiveBall.add(new EncoderForward(driveTrain, 2500f, 0.2f));
		centerFiveBall.add(new NavxTurn(driveTrain, navx, 30, 0.4f, 4.0f));
		centerFiveBall.add(new LimelightTrack(driveTrain, shooter, limelight, 1.0f));
		centerFiveBall.add(new Shoot(shooter, indexer, 4100.0, 5));

		// center eight ball -------------
		centerEightBall = new LinkedList<AutoStep>();
		centerEightBall.add(new NavxReset(navx));
		centerEightBall.add(new ShooterRev(shooter, 3900.0));
		centerEightBall.add(new IntakeDrop(intakeSolenoid));
		centerEightBall.add(new IntakeRun(intakeSeven, 1.0f));
		// Configuration Done
		centerEightBall.add(new EncoderForward(driveTrain, 72000f, -0.85f)); // 85
		centerEightBall.add(new NavxTurn(driveTrain, navx, 75, .4f, 4.0f));
		centerEightBall.add(new EncoderForward(driveTrain, 14000f, -0.3f)); // 16000
		centerEightBall.add(new EncoderForward(driveTrain, 10000f, 0.3f));
		centerEightBall.add(new NavxTurn(driveTrain, navx, 90, .3f, 5.0f));
		centerEightBall.add(new EncoderForward(driveTrain, 20000f, -0.3f));
		// Balls intaked
		centerEightBall.add(new EncoderForward(driveTrain, 10000f, 0.3f));
		centerEightBall.add(new NavxTurn(driveTrain, navx, 10, .9f, 4.0f));// 15
		centerEightBall.add(new Wait(driveTrain, 0.1f));
		centerEightBall.add(new LimelightTrack(driveTrain, shooter, limelight, 0.0f));
		// First Shooting Session
		centerEightBall.add(new Shoot(shooter, indexer, 3900.0, 5));
		// centerEightBall.add(new ShooterRev(shooter, 3600.0));
		// centerEightBall.add(new NavxTurn(driveTrain, navx, -50, .4f, 5.0f)); // -85,
		// .3
		// centerEightBall.add(new EncoderForward(driveTrain, 20000f, -0.4f));
		// centerEightBall.add(new NavxTurn(driveTrain, navx, 0, .5f, 5.0f)); // -5
		// centerEightBall.add(new Wait(driveTrain, 0.05f));
		// centerEightBall.add(new NavxTurn(driveTrain, navx, 0, .3f, 1.0f)); // -5
		// centerEightBall.add(new EncoderForward(driveTrain, 60000f, -0.6f));// 75000
		// centerEightBall.add(new LimelightTrack(driveTrain, shooter, limelight,
		// 0.0f));
		// centerEightBall.add(new Shoot(shooter, indexer, 3600.0, 4));

		double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);

		// Overides Dashboard.
		// autoChoice = 4;

		if (autoChoice == 0) {
			autonomousSelected = mostBasicShoot;

		} else if (autoChoice == 1) {
			autonomousSelected = rightFiveBall;// TrenchFiveBall

		} else if (autoChoice == 2) {
			autonomousSelected = centerEightBall;// RightEightBall

		} else if (autoChoice == 3) {
			autonomousSelected = centerFiveBall;

		} else if (autoChoice == 4) {
			autonomousSelected = rightSixBall;// TrenchSixBall

		} else {
			autonomousSelected = mostBasicShoot;
		}

		System.out.println("Selected auto " + autoChoice);

		// autonomousSelected = centerEightBall;
		autonomousSelected.get(0).Begin();
	}

	public void autonomousPeriodic() {
		SendPDPData();
		SmartDashboard.putBoolean("RobotEnabled", true);

		// System.out.println(driveTrain.GetEncoder());

		shooter.Update();
		shooter.UpdatePID();

		// autonomous loop
		System.out.println("Current auto step " + currentAutoStep);
		if (currentAutoStep < autonomousSelected.size()) {

			autonomousSelected.get(currentAutoStep).Update();

			if (autonomousSelected.get(currentAutoStep).autoIndex) {
				indexer.RunAutomatic(true);
			}

			if (autonomousSelected.get(currentAutoStep).isDone) {
				currentAutoStep = currentAutoStep + 1;
				if (currentAutoStep < autonomousSelected.size()) {
					autonomousSelected.get(currentAutoStep).Begin();
				}
			}
		} else {
			// System.out.println("Autonomous Done");
			driveTrain.SetBothSpeed(0.0f);
			// currentState = RobotState.Teleop;
		}

		UpdateMotors();
	}

	public void teleopInit() {

		limelight.TeleopSettings();
		limelight.SetLight(false);

		// colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		shooter.Init();

		driveTrain.SetCoast();
		climber.lock();

		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	// Drive Scale
	boolean slow = false;
	public float DriveScaleSelector(float ControllerInput, DriveScale selection) {

		float multiplier = (ControllerInput / (float) Math.abs(ControllerInput));

		if (selection == DriveScale.parabala) {
			float output = multiplier * (float) Math.pow(ControllerInput, 2);

			if (flightStickLeft.getRawButtonPressed(1)) {
				slow = !slow;
			}

			if (slow) {
				output = output * 0.5f;
			}
			
			return output;
		} else if (selection == DriveScale.tangent) {
			return multiplier * (0.4f * (float) Math.tan(1.8 * (multiplier * ControllerInput) - .9) + 0.5f);
		} else if (selection == DriveScale.inverse) {
			return (float) Math.pow(ControllerInput, 1 / 2);
		} else if (selection == DriveScale.cb) {
			return (float) Math.pow(ControllerInput, 3);
		} else if (selection == DriveScale.cbrt) {
			return multiplier * (0.63f * (float) Math.cbrt((multiplier * ControllerInput) - 0.5f) + 0.5f);
		} else {
			return ControllerInput;
		}
	}

	public void teleopPeriodic() {
		driveTrain.SendData();
		SendPDPData();
		SmartDashboard.putBoolean("RobotEnabled", true);

		indexer.DebugPrint();
		shooter.Update();

		// Match Time
		// SmartDashboard.putNumber("MatchTime",
		// DriverStation.getInstance().getMatchTime());

		// Color Wheel

		// Color Wheel
		if (flightStickLeft.getRawButton(1)) {
			ColorTwo.set(ControlMode.PercentOutput, 0.5f);
		} else {
			ColorTwo.set(ControlMode.PercentOutput, 0.0f);
		}

		if (false) {
			// Intake Sensor (31)
			if (ColorWheelLimit.get() == true && !limitPressed) {
				limitPressed = true;
				colorChangsCount = colorChangsCount + 1;
			}
			if (ColorWheelLimit.get() == false) {
				limitPressed = false;
			}
			// Color Sensor (25)
			// ColorWheel currentColor = GetCurrentColor();
			// if (currentColor != LastColor && currentColor != ColorWheel.Unknown) {
			// colorChangsCount = colorChangsCount + 1;
			// LastColor = currentColor;
			// }

			// Reset Color Changes Number
			if (flightStickLeft.getRawButton(8)) {
				colorChangsCount = 0;
			}

			// Color Wheel Spin
			if (flightStickLeft.getRawButton(9)) {
				if (colorChangsCount <= 31) {// 25
					MotorSeven.set(ControlMode.PercentOutput, 1.0f);
				} else {
					MotorSeven.set(ControlMode.PercentOutput, 0.0f);
				}

			} else {
				if (flightStickLeft.getRawButton(10)) {
					MotorSeven.set(ControlMode.PercentOutput, 0.5f);
				} else {
					MotorSeven.set(ControlMode.PercentOutput, 0.0f);
				}
			}

			// Color Wheel Find
			if (flightStickLeft.getRawButton(11)) {
				if (GetTargetColor() != LastColor) {
					MotorSeven.set(ControlMode.PercentOutput, 1.0f);
				} else {
					MotorSeven.set(ControlMode.PercentOutput, 0.0f);
				}
			}
		}

		// Intake 2=B, 4=Y
		double intakeSpeed = 0.8f;
		if (operator.getRawButton(2)) {
			intakeSeven.set(ControlMode.PercentOutput, intakeSpeed);
			SmartDashboard.putNumber("intakeMotor", intakeSpeed);
		} else if (operator.getRawButton(4)) {
			intakeSeven.set(ControlMode.PercentOutput, -intakeSpeed);
			SmartDashboard.putNumber("intakeMotor", -intakeSpeed);
		} else {
			intakeSeven.set(ControlMode.PercentOutput, 0.0f);
			SmartDashboard.putNumber("intakeMotor", 0.0f);
		}

		// Intake Solenoid 1=A
		if (operator.getRawButtonPressed(1)) {
			if (intakeSolenoid.get() == Value.kForward) {
				intakeDown();
			} else if (intakeSolenoid.get() == Value.kReverse) {
				intakeUp();
			} else {
				intakeUp();
			}
		}

		// Shooter 5=Left Bumper
		shooter.rpmTarget = SmartDashboard.getNumber(shooter.shooterRPMKey, shooter.shooterRPMTarget);
		if (operator.getRawButton(5)) {
			//shooter.rpmTarget = 2990;
					//Original: 3900
					// 2 (Sweet spot) 2990
					// 1 (Farthest): 2830

				
			if (flightStickRight.getRawButton(6)){

				shooter.rpmTarget = 2877; //2830 //2870
			
			} else if (flightStickRight.getRawButton(7)){
			
				shooter.rpmTarget = 2990;
			
			} else if (flightStickRight.getRawButton(10)){
			
				shooter.rpmTarget = 3900;
			
			} else {

				shooter.rpmTarget = 3900;
			} 


			// SmartDashboard.putNumber(shooter.shooterRPMKey, 5700);
			limelight.SetLight(true);
		} else {
			limelight.SetLight(false);
		}
		shooter.UpdatePID();
		// TODO
		// Indexer 3=X 6=Right Bumper 8=Start
		if (operator.getRawButton(6)) {
			indexer.RunManualForward(0.6f, 0.05f);
			driveTrain.SetBreak();
			SmartDashboard.putNumber("ballCounter", 0);
		} else if (operator.getRawButton(8)) { // Manual Override Backwards
			indexer.RunManualForward(-0.6f, 0.05f);
			SmartDashboard.putNumber("ballCounter", 0);
		} else {
			indexer.RunAutomatic(operator.getRawButton(2));
			SmartDashboard.putNumber("ballCounter", indexer.ballCounter);
		}

		if (operator.getRawButtonPressed(7)) {
			// indexer.floorBeltToggle = !indexer.floorBeltToggle;
		}

		// Climber
		// Button9=LeftJoystickClick
		// Button10=RightJoystickClick

		if (operator.getRawButton(9)) {
			if (operator.getRawButtonPressed(9)) {
				climber.Reset();
			}
			climber.climbMotors(0.5f);
		} else if (operator.getRawButton(10)) {
			if (operator.getRawButtonPressed(10)) {
				climber.Reset();
			}
			climber.climbMotors(-0.5f);
		} else {
			if (operator.getRawButtonReleased(9) || operator.getRawButtonReleased(10)) {
				climber.Reset();
			}
			climber.climbMotors(0.0f);
		}
		// Climber alt Controls
		/*
		 * if (operator.getRawButton(9)) { if (operator.getRawButtonPressed(9)) {
		 * climber.Reset(); } climber.climbMotors(0.5f); } else if
		 * (operator.getRawButton(10)) { if (operator.getRawButtonPressed(10)) {
		 * climber.Reset(); } climber.climbMotors(-0.5f); } else { if
		 * (operator.getRawButtonReleased(9) || operator.getRawButtonReleased(10)) {
		 * climber.Reset(); } climber.climbMotors(0.0f); }
		 */
		// Lime Light
		if (flightStickLeft.getRawButton(6) || (flightStickLeft.getRawButton(7))) {
			if (flightStickLeft.getRawButton(6)) {
				limelight.Position(driveTrain, 1, 0);
			} else {
				limelight.Position(driveTrain, -1, 0);
			}
			driveTrain.SetBreak();
		} else {
			if (!operator.getRawButton(6)) {
				driveTrain.SetCoast();
			}
			ControllerDrive();
		}

		UpdateMotors();
	}

	public void testInit() {

		navx.reset();
		climber.unlock();
		climber.coastMode();

		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);

		shooter.Init();

	}

	public float Lerp(float v0, float v1, float t) {

		if (t < 0) {
			t = 0;

		} else if (t > 1) {
			t = 1;
		}

		return (v0 + t * (v1 - v0));
	}

	public void testPeriodic() {
		System.out.println(SmartDashboard.getNumber("autoMode", 0));
		limelight.SetLight(true);
		System.out.println(navx.getYaw() + " Navx");
		// driveTrain.SetBothSpeed(0.0f);
	}

	public void ControllerDrive() {
		if (arcadeDrive) {

			// Arcade
			float horJoystick = TranslateController((float) driver.getRawAxis(4));
			float verJoystick = TranslateController((float) driver.getRawAxis(1));

			driveTrain.SetRightSpeed(-verJoystick + -horJoystick);
			driveTrain.SetLeftSpeed(-verJoystick + horJoystick);
			// driveTrain.SetCoast();
		} else {

			// tank
			float leftJoystick = DriveScaleSelector((float) flightStickLeft.getRawAxis(1), DriveScale.linear);
			float rightJoystick = DriveScaleSelector((float) flightStickRight.getRawAxis(1), DriveScale.linear);
			driveTrain.SetRightSpeed(-rightJoystick);
			driveTrain.SetLeftSpeed(-leftJoystick);
			// driveTrain.SetCoast();
		}
	}

	/*
	 * public ColorWheel GetCurrentColor() {
	 * 
	 * Color detectedColor = colorSensor.getColor();
	 * //System.out.println(detectedColor.red + " - " + detectedColor.green + " - "
	 * + detectedColor.blue);
	 * 
	 * // detecting blue { float redMin = 0.15f; float greenMin = 0.45f; float
	 * blueMin = 0.4f;
	 * 
	 * if (detectedColor.red < redMin && detectedColor.green < greenMin &&
	 * detectedColor.blue > blueMin) { return ColorWheel.Blue; } }
	 * 
	 * // detecting green { float redMin = 0.2f; float greenMin = 0.54f; float
	 * blueMin = 0.28f;
	 * 
	 * if (detectedColor.red < redMin && detectedColor.green > greenMin &&
	 * detectedColor.blue < blueMin) { return ColorWheel.Green; }
	 * 
	 * }
	 * 
	 * // detecting Red { float redMin = 0.45f; float greenMin = 0.4f; float blueMin
	 * = 0.16f;
	 * 
	 * if (detectedColor.red > redMin && detectedColor.green < greenMin &&
	 * detectedColor.blue < blueMin) { return ColorWheel.Red; } }
	 * 
	 * // detecting yellow { float redMin = 0.3f; float greenMin = 0.53f; float
	 * blueMin = 0.1f;
	 * 
	 * if (detectedColor.red > redMin && detectedColor.green > greenMin &&
	 * detectedColor.blue > blueMin) { return ColorWheel.Yellow; } }
	 * 
	 * if (detectedColor.red == 0 && detectedColor.green == 0 && detectedColor.blue
	 * == 0) { return ColorWheel.Broken; }
	 * 
	 * return ColorWheel.Unknown; }
	 */

	public void UpdateMotors() {
		driveTrain.Update();
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}

	// Intake Solenoid
	public void intakeDown() {
		System.out.println("Enabled");
		SmartDashboard.putBoolean("intakeExtended", true);
		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void intakeUp() {
		System.out.println("Disabled");
		SmartDashboard.putBoolean("intakeExtended", false);
		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	public ColorWheel GetTargetColor() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			switch (gameData.charAt(0)) {
			case 'B':
				return ColorWheel.Blue;
			case 'G':
				return ColorWheel.Green;
			case 'R':
				return ColorWheel.Red;
			case 'Y':
				return ColorWheel.Yellow;
			}
		}

		return ColorWheel.Unknown;
	}

	public void SendPDPData() {
		PowerDistributionPanel pdp = new PowerDistributionPanel();
		SmartDashboard.putNumber("PDP_Temperature", pdp.getTemperature());
		SmartDashboard.putNumber("PDP_Voltage", pdp.getVoltage());
		for (int index = 0; index < 15; index++) {
			SmartDashboard.putNumber("PDP_" + index, pdp.getCurrent(index));
		}
		pdp.close();
	}
}