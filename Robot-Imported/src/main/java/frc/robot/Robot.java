package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.cscore.UsbCamera;
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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
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

	// Drivetrain
	public DriveTrain driveTrain = new DriveTrain(54, 55, 56, 57, navx);

	// Nuemataics
	public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

	// Logic
	public boolean speedToggle = false;

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick flightStickLeft;
	public Joystick flightStickRight;

	public Limelight limelight = new Limelight();
	public PreShooterpid preshooterpid = new PreShooterpid(limelight);
	public Shooter shooter = new Shooter(limelight, preshooterpid);
	public HotPID navxPID = new HotPID("navx", 0.0005, 0, 0);
	public NavxTurnPID navxturnpid = new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID);
	public Climber climber = new Climber();

	// Motors
	// public TalonSRX forTesting = new TalonSRX(51);
	public TalonSRX intakeSeven = new TalonSRX(2);
	public TalonSRX ShooterLeft = new TalonSRX(50);
	public TalonSRX ShooterRight = new TalonSRX(6); 
	public TalonSRX preShooterFive = new TalonSRX(4);

	public TalonSRX MotorSeven = new TalonSRX(30);
	public TalonSRX MotorEight = new TalonSRX(31);

	public Indexer indexer = new Indexer(shooter, preshooterpid);

	boolean limitPressed;

	public HotPID turnPID;

	public enum DriveScale {
		linear, parabala, tangent, inverse, cb, cbrt,
	}

	enum AutoChoice {
		// Left, Right
	}

	public DriverStation driverStation;
	public RobotState currentState;

	// Auto
	public LinkedList<AutoStep> autonomousSelected;
	public LinkedList<AutoStep> autoFourBall;
	public int currentAutoStep = 0;

	// start on the line, backup, and shoot

	public String autoSelectKey = "autoMode";

	public void robotInit() {
		CameraServer.startAutomaticCapture();
		limelight.SetLight(false);
		limelight.Init();
		shooter.Init();
		preshooterpid.Init();
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
		SmartDashboard.putBoolean("RobotEnabled", false);
	}

	public void autonomousInit() {
		currentAutoStep = 0;
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

		driveTrain.SetBreak();
		limelight.SetLight(true);
		//TODO
		/*
		autoFourBall = new LinkedList<AutoStep>();
		autoFourBall.add(new NavxReset(navx));
		autoFourBall.add(new IntakeDrop(intakeSolenoid));
		autoFourBall.add(new IntakeRun(intakeSeven, 0.8f));
		// pickup first ball
		autoFourBall.add(new EncoderForward(driveTrain, 35000, -0.2f));
		// move forward towards goal
		autoFourBall.add(new EncoderForward(driveTrain, 52000, 0.35f)); // 55000 .2
		*/
		//TODO
		autoFourBall.add(new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID));
		//TODO
		/*
		autoFourBall.add(new EncoderForward(driveTrain, 5000, 0.2f));
		// autoFourBall.add(new Wait(driveTrain, 0.5f));
		autoFourBall.add(new Shoot(shooter, indexer));
		autoFourBall.add(new IntakeRun(intakeSeven, 0.0f));
		autoFourBall.add(new NavxTurn(driveTrain, navx, 72, 0.3f, 2.5f));
		autoFourBall.add(new Wait(driveTrain, 0.5f));
		autoFourBall.add(new IntakeRun(intakeSeven, 0.8f));
		autoFourBall.add(new EncoderForward(driveTrain, 150000, -0.5f));
		*/
		//TODO

		/*
		 * autoFourBall.add(new LimelightTrack(driveTrain, shooter, limelight, 0));
		 * // shoot
		 * autoFourBall.add(new Shoot(shooter, indexer));
		 * autoFourBall.add(new IntakeRun(intakeSeven, 0.0f));
		 * autoFourBall.add(new NavxTurn(driveTrain, navx, 72, 0.3f, 5));
		 * autoFourBall.add(new Wait(driveTrain, 0.5f));
		 * autoFourBall.add(new IntakeRun(intakeSeven, 0.8f));
		 * autoFourBall.add(new EncoderForward(driveTrain, 150000, -0.5f));
		 */

		double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);

		autonomousSelected = autoFourBall;
		autonomousSelected.get(0).Begin();
	}

	public void autonomousPeriodic() {
		SmartDashboard.putBoolean("RobotEnabled", true);

		// System.out.println(driveTrain.GetEncoder());

		shooter.Update();
		preshooterpid.Update();

		// autonomous loop
		System.out.println("Current auto step " + currentAutoStep);
		if (currentAutoStep < autonomousSelected.size()) {

			autonomousSelected.get(currentAutoStep).Update();

			if (autonomousSelected.get(currentAutoStep).autoIndex) {
				indexer.RunAutomatic();
			}
			if (autonomousSelected.get(currentAutoStep).runShooter) {
				shooter.Update();
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
		navx.reset();
		limelight.SetLight(false);

		turnPID = new HotPID("turn", 0, 0, 0);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		shooter.Init();
		preshooterpid.Init();

		driveTrain.SetCoast();

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
		// System.out.println(navx.getYaw() + " yaw");
		// System.out.println(navx.getPitch() + " pitch");
		// System.out.println(navx.getRoll() + " roll");
		driveTrain.SendData();
		SmartDashboard.putBoolean("RobotEnabled", true);

		// Intake 3=B, 4=Y
		double intakeSpeed = 0.75f;
		if (operator.getRawButton(3)) {
			intakeSeven.set(ControlMode.PercentOutput, intakeSpeed);
			SmartDashboard.putNumber("intakeMotor", intakeSpeed);
		} else if (operator.getRawButton(4)) {
			intakeSeven.set(ControlMode.PercentOutput, -intakeSpeed);
			SmartDashboard.putNumber("intakeMotor", -intakeSpeed);
		} else {
			intakeSeven.set(ControlMode.PercentOutput, 0.0f);
			SmartDashboard.putNumber("intakeMotor", 0.0f);
		}

		// Intake Solenoid 2=A
		if (operator.getRawButtonPressed(2)) {
			if (intakeSolenoid.get() == Value.kForward) {
				intakeDown();
			} else if (intakeSolenoid.get() == Value.kReverse) {
				intakeUp();
			} else {
				intakeUp();
			}
		}

		// Indexer 5=Left Bumper
		// int indexerAxis = 2;

		// System.out.println(firstBeam.get() + " - " + secondBeam.get());

		// System.out.println("beam1 " + firstBeam + "beam2 " + secondBeam);
		int shootButton = 5;
		if (operator.getRawButtonPressed(shootButton)) {
			shooter.pid.reset();
			preshooterpid.preshooterPid.reset();

		}

		if (operator.getRawButton(shootButton)) {

			shooter.Update();

			// get target distance from limelight
			// run indexer
			float buffer = 0.05f;
			float speed = -0.5f;

			if (operator.getRawButton(7)) {
				indexer.RunManualForward(speed, buffer);
			}

		} else {
			preshooterpid.PowerManual(0);
			shooter.PowerManual(0);

			if (operator.getRawButton(7)) {
				indexer.RunAutomatic();
			} else {
				indexer.RunManualForward(0, 0);
			}
		}

		// Climber
		// Button11=LeftJoystickClick
		// Button12=RightJoystickClick
		int pov = operator.getPOV();
		System.out.println(pov + "pov");

		
		if (pov != -1){
		if (280 < pov || pov < 80) {
			climber.brakeMode();
			climber.climbMotors(-0.5f);
		} else if (100 < pov && pov < 260) {
			climber.climbMotors(0.5f);
		}
		}else{
		climber.climbMotors(0.0f);
		}
	
		if (operator.getRawButton(10)) {
			climber.Tilt();
		}
		/*
		if (operator.getRawButton(11)) {
			climber.climbMotors(0.5f);
		} else if (operator.getRawButton(12)) {
			climber.brakeMode();
			climber.climbMotors(-0.5f);
		} else {
			climber.climbMotors(0.0f);
		}
		if (operator.getRawButtonPressed(10)) {
			climber.Tilt();
		}
		*/

		// Lime Light
		if (flightStickLeft.getRawButton(6) || driver.getRawButton(6)) {
			limelight.Position(driveTrain);
			driveTrain.SetBreak();
		} else {
			driveTrain.SetCoast();

			System.out.println("yaw " + navx.getYaw());

			if (flightStickLeft.getRawButton(0)) {
				double output = turnPID.Calculate(navx.getYaw());
				driveTrain.SetBothSpeed((float)output);
				driveTrain.SetLeftSpeed((float)-output);
			} else {
				ControllerDrive();
			}
		}

		UpdateMotors();
	}

	public void testInit() {

		navx.reset();
		climber.coastMode();

		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
		preshooterpid.Init();
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
		// navxturnpid.Update();
		climber.coastMode();
		System.out.println(indexer.firstBeam.get() + " First beam");
		System.out.println(indexer.secondBeam.get() + " Second beam");
		

		// forTesting.set(ControlMode.PercentOutput, 0.5f);

		// if (flightStickRight.getRawButton(1)) {
		// 	driveTrain.SetRightSpeed(0.5f);
		// 	driveTrain.SetLeftSpeed(-0.5f);
		// } else if (flightStickLeft.getRawButton(1)) {
		// 	driveTrain.SetRightSpeed(-0.5f);
		// 	driveTrain.SetLeftSpeed(0.5f);
		// } else {
		// 	driveTrain.SetRightSpeed(0.0f);
		// 	driveTrain.SetLeftSpeed(0.0f);
		// }

		// double shooterspeed = -0.0;
		// shooterspeed = -((flightStickLeft.getRawAxis(2)-1)/2);
		// ShooterLeft.set(ControlMode.PercentOutput, shooterspeed);
		// ShooterRight.set(ControlMode.PercentOutput, -shooterspeed);
		// limelight.SetLight(true);

		UpdateMotors();
	}

	public void ControllerDrive() {
		if (arcadeDrive) {

			// Arcade
			// float horJoystick = TranslateController((float) driver.getRawAxis(2));
			// float verJoystick = TranslateController((float) driver.getRawAxis(1));

			float horJoystick = DriveScaleSelector((float) driver.getRawAxis(2), DriveScale.cb);
			float verJoystick = DriveScaleSelector((float) driver.getRawAxis(1), DriveScale.cb);

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
}