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

    public DigitalInput firstBeam = new DigitalInput(1);
    public DigitalInput secondBeam = new DigitalInput(0);

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
	public Shooter shooter = new Shooter(limelight);
	public Climber climber = new Climber();

	// Motors
	public TalonSRX intakeSeven = new TalonSRX(5);
	public TalonSRX ShooterLeft = new TalonSRX(50);
	public TalonSRX ShooterRight = new TalonSRX(4); // 51
	public TalonSRX preShooter = new TalonSRX(9);

	public TalonSRX MotorSeven = new TalonSRX(30);
	public TalonSRX MotorEight = new TalonSRX(31);
	public TalonSRX indexer = new TalonSRX(2);

	boolean limitPressed;

	public int ballCount = 0;

	public enum DriveScale {
		linear, parabala, tangent, inverse, cb, cbrt,
	}

	enum AutoChoice {
		// Left, Right
	}

	public DriverStation driverStation;
	public RobotState currentState;

	public boolean firstInitialTrigger = false;
	public boolean secondInitialTrigger = false;

	// Auto
	public LinkedList<AutoStep> autonomousSelected;
	// start on the line, backup, and shoot

	public String autoSelectKey = "autoMode";

	public void robotInit() {
		limelight.SetLight(false);
		shooter.Init();
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

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
		// autonomousSelected = SmartDashboard.getNumber("/SmartDashboard/autoMode", 0);

		limelight.AutoSettings();
		driveTrain.SetBreak();
		limelight.SetLight(true);

		double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);

		autonomousSelected.get(0).Begin();
	}

	public void autonomousPeriodic() {
		SmartDashboard.putBoolean("RobotEnabled", true);

		// System.out.println(driveTrain.GetEncoder());

		shooter.Update();

		/*
		 * // autonomous loop
		 * System.out.println("Current auto step " + currentAutoStep);
		 * if (currentAutoStep < autonomousSelected.size()) {
		 * 
		 * autonomousSelected.get(currentAutoStep).Update();
		 * 
		 * if (autonomousSelected.get(currentAutoStep).autoIndex) {
		 * indexer.RunAutomatic(true);
		 * }
		 * 
		 * if (autonomousSelected.get(currentAutoStep).isDone) {
		 * currentAutoStep = currentAutoStep + 1;
		 * if (currentAutoStep < autonomousSelected.size()) {
		 * autonomousSelected.get(currentAutoStep).Begin();
		 * }
		 * }
		 * } else {
		 * // System.out.println("Autonomous Done");
		 * driveTrain.SetBothSpeed(0.0f);
		 * // currentState = RobotState.Teleop;
		 * }
		 */
		UpdateMotors();
	}

	public void teleopInit() {

		ballCount = 0;

		limelight.TeleopSettings();
		limelight.SetLight(false);

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
		SmartDashboard.putBoolean("RobotEnabled", true);

		// Intake 2=B, 4=Y
		double intakeSpeed = 0.75f;
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

		// Indexer 5=Left Bumper
		float indexSpeed = -0.4f;
		float indexerTargetSpeed = 0;
		int indexerAxis = 2;

		//System.out.println(firstBeam.get() + " - " + secondBeam.get());
		System.out.println(ballCount);

		// beam false when broken
		if (operator.getRawAxis(indexerAxis) > 0.5f) {
			if (ballCount == 0) {

				if (!firstBeam.get()) {
					firstInitialTrigger = true;
				}

				if (firstInitialTrigger && firstBeam.get()) {
					ballCount = 1;
				}

				indexerTargetSpeed = indexSpeed;

			}

			if (ballCount == 1) {

				if (!firstBeam.get() && secondBeam.get()) {
					indexerTargetSpeed = indexSpeed;
				}

			}
		}

		/*
		if (operator.getRawAxis(2) > 0.5f && firstBeam.get()) {
			indexer.set(ControlMode.PercentOutput, indexerSpeed);
		} else if (operator.getRawAxis(3) > 0.5f && secondBeam.get()) {
			indexer.set(ControlMode.PercentOutput, indexerSpeed);
		} else {
		}
		*/

		if (operator.getRawButton(5)) {
			preShooter.set(ControlMode.PercentOutput, -0.5f); // Close:0.8 //0.3
			shooter.rpmTarget = 2500;

			ballCount = 0;
			firstInitialTrigger = false;
			if (operator.getRawAxis(indexerAxis) > 0.5f) {
				indexerTargetSpeed = indexSpeed;
			}

		} else {
			shooter.rpmTarget = 0;
			preShooter.set(ControlMode.PercentOutput, 0.0f);
		}

		indexer.set(ControlMode.PercentOutput, indexerTargetSpeed);


		if (shooter.rpmTarget > 0.0) {
			shooter.Update();
		} else {
			ShooterLeft.set(ControlMode.PercentOutput, 0);
			ShooterRight.set(ControlMode.PercentOutput, 0);
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

		ControllerDrive();
		UpdateMotors();
	}

	public void teleopPeriodic_() {

		/*
		 * 
		 * // Lime Light
		 * if (flightStickLeft.getRawButton(6) || (flightStickLeft.getRawButton(7))) {
		 * if (flightStickLeft.getRawButton(6)) {
		 * limelight.Position(driveTrain, 1, 0);
		 * } else {
		 * limelight.Position(driveTrain, -1, 0);
		 * }
		 * driveTrain.SetBreak();
		 * } else {
		 * if (!operator.getRawButton(6)) {
		 * driveTrain.SetCoast();
		 * }
		 * ControllerDrive();
		 * }
		 * 
		 */

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

		driveTrain.SetRightSpeed(0.0f);
		driveTrain.SetLeftSpeed(0.0f);
		double shooterspeed = -0.5;
		// shooterspeed = -((flightStickLeft.getRawAxis(2)-1)/2);
		System.out.println(shooterspeed);
		ShooterLeft.set(ControlMode.PercentOutput, shooterspeed);
		ShooterRight.set(ControlMode.PercentOutput, -shooterspeed);
		limelight.SetLight(true);
		UpdateMotors();
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