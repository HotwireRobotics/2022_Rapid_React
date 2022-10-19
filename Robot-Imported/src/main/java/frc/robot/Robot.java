package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
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
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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
	public double trackWidth = 0.61;

	public boolean toggleReset = true;

	// Sensors
	public AnalogGyro headinGryo = new AnalogGyro(0);
	public AHRS navx = new AHRS(SPI.Port.kMXP);

	// private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
	public float encoderSpeed1 = 0;
	public float encoderSpeed2 = 0;

	// public Encoder shooterEncoder = new Encoder();

	// Drivetrain
	DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(headinGryo.getAngle()));

	public DriveTrain driveTrain = new DriveTrain(54, 55, 56, 57, navx);
	public TalonSRX frontLeft = new TalonSRX(54);
	public TalonSRX backLeft = new TalonSRX(55);
	public TalonSRX frontRight = new TalonSRX(56);
	public TalonSRX backRight = new TalonSRX(57);

	// Nuemataics
	public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

	// Logic
	public SimpleMotorFeedforward preShooterFeed = new SimpleMotorFeedforward(0.48691, 0.11446, 0.015037);
	public boolean indexToggle = true;
	public boolean indexToggle2 = true;

	public int pov;
	public boolean speedToggle = false;

	// Joysticks
	//public Joystick driver;
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick flightStickLeft;
	public Joystick flightStickRight;

	public Limelight limelight = new Limelight();
	public PreShooterpid preshooterpid = new PreShooterpid(limelight);
	public Shooter shooter = new Shooter(limelight, preshooterpid);

	public HotPID navxPID = new HotPID("navx", 0.01, 0.01, 0.0);// was 0.0005

	public NavxTurnPID navxturnpid = new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID);
	public Climber climber = new Climber();
	public TalonSRX climberOne = new TalonSRX(35);
	public TalonSRX climberTwo = new TalonSRX(52);

	// Motors

	// Swervy Auto Stuff

	public boolean opToggle;
	// public TalonSRX forTesting = new TalonSRX(51);
	public TalonSRX intakeSeven = new TalonSRX(2);
	public TalonSRX ShooterLeft = new TalonSRX(50);
	public TalonSRX ShooterRight = new TalonSRX(6);
	public TalonSRX preShooterFive = new TalonSRX(4);

	public TalonSRX MotorSeven = new TalonSRX(30);
	public TalonSRX MotorEight = new TalonSRX(31);

	public Indexer indexer = new Indexer(shooter, preshooterpid);
	public TalonSRX indexerMotor = new TalonSRX(9);
	boolean limitPressed;
	public boolean toggleClimbers = false;

	public HotPID turnPID;

	public enum DriveScale {
		linear, parabala, tangent, inverse, cb, cbrt,
	}

	public DriverStation driverStation;
	public RobotState currentState;

	// Auto
	public LinkedList<AutoStep> autonomousSelected;
	public LinkedList<AutoStep> autoFourBallRed;
	public LinkedList<AutoStep> autoFourBallBlue;
	public LinkedList<AutoStep> autoTwoBall;
	public LinkedList<AutoStep> autoTest;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public void robotInit() {
		SmartDashboard.putNumber("Ballcount", 0);
		SmartDashboard.putBoolean("TwoBall", false);
		SmartDashboard.putBoolean("FourBallBlue", false);
		SmartDashboard.putBoolean("FourBallRed", false);
		SmartDashboard.putBoolean("Test", false);
		CameraServer.startAutomaticCapture();
		limelight.SetLight(false);
		limelight.Init();
		shooter.Init();
		preshooterpid.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);

	}

	public void periodic() {
		var heading = Rotation2d.fromDegrees(headinGryo.getAngle());

	}

	public void disabledInit() {

		// Controllers
		driveTrain.SetBreak();
		//driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void disabledPeriodic() {
		driveTrain.SendData();
		SmartDashboard.putBoolean("RobotEnabled", false);
	}

	public void autonomousInit() {
		climber.errorSet();
		currentAutoStep = 0;
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

		driveTrain.SetBreak();
		limelight.SetLight(true);

		// Twoball outside

		autoTwoBall = new LinkedList<AutoStep>();
		autoTwoBall.add(new NavxReset(navx));
		autoTwoBall.add(new IntakeDrop(intakeSolenoid, true));
		autoTwoBall.add(new IntakeRun(intakeSeven, 0.8f));
		// pickup first ball
		autoTwoBall.add(new EncoderForward(driveTrain, 36000, -0.2f));
		// move forward towards goal
		autoTwoBall.add(new EncoderForward(driveTrain, 3500, 0.35f));
		autoTwoBall.add(new LimelightTrack(driveTrain, shooter, limelight, 0));
		// shoot
		autoTwoBall.add(new Shoot(shooter, indexer));
		autoTwoBall.add(new IntakeDrop(intakeSolenoid, false));
		autoTwoBall.add(new EncoderForward(driveTrain, 15000, -0.2f));

		// FourBallRed
		autoFourBallRed = new LinkedList<AutoStep>();
		autoFourBallRed.add(new Print("red ran"));
		autoFourBallRed.add(new NavxReset(navx));
		autoFourBallRed.add(new IntakeDrop(intakeSolenoid, true));
		autoFourBallRed.add(new IntakeRun(intakeSeven, 0.8f));
		// pickup first ball
		autoFourBallRed.add(new EncoderForward(driveTrain, 65296, -0.8f));
		// move forward towards goal
		// autoFourBall.add(new EncoderForward(driveTrain, 2500, 0.35f)); // 55000 .2
		autoFourBallRed.add(new LimelightTrack(driveTrain, shooter, limelight, 0));
		autoFourBallRed.add(new Wait(driveTrain, 0.3f));
		// shoot
		autoFourBallRed.add(new Shoot(shooter, indexer));
		autoFourBallRed.add(new NavxTurn(driveTrain, navx, 14f, 0.15f, 2.0f));// -speed 14.15c
		// autoFourBall.add(new Wait(driveTrain, 0.5f));
		autoFourBallRed.add(new EncoderForwardFeet(driveTrain, 11.5f, -0.8f));// 12
		// autoFourBallRed.add(new NavxTurn(driveTrain, navx, -20f, 0.2f, 5f));// -speed
		// 14.15
		autoFourBallRed.add(new NavxTurn(driveTrain, navx, -5, 0.25f, 5.0f));// added
		autoFourBallRed.add(new EncoderForwardFeet(driveTrain, 2f, 0.8f));
		autoFourBallRed.add(new Wait(driveTrain, 0.8f));
		// autoFourBallRed.add(new NavxTurn(driveTrain, navx, 0f, 0.2f, 5f));// -speed
		// 14.15
		autoFourBallRed.add(new EncoderForwardFeet(driveTrain, 9f, 0.8f));// 6
		autoFourBallRed.add(new LimelightTrack(driveTrain, shooter, limelight, 0));
		autoFourBallRed.add(new Wait(driveTrain, 0.3f));
		autoFourBallRed.add(new Shoot(shooter, indexer));

		// FourBallBlue
		autoFourBallBlue = new LinkedList<AutoStep>();
		autoFourBallBlue.add(new Print("blue ran"));
		autoFourBallBlue.add(new NavxReset(navx));
		autoFourBallBlue.add(new IntakeDrop(intakeSolenoid, true));
		autoFourBallBlue.add(new IntakeRun(intakeSeven, 0.8f));
		// pickup first ball
		autoFourBallBlue.add(new EncoderForward(driveTrain, 65296, -0.8f));
		// move forward towards goal
		// autoFourBall.add(new EncoderForward(driveTrain, 2500, 0.35f)); // 55000 .2
		autoFourBallBlue.add(new LimelightTrack(driveTrain, shooter, limelight, 0));
		autoFourBallBlue.add(new Wait(driveTrain, 0.3f));
		// shoot
		autoFourBallBlue.add(new Shoot(shooter, indexer));
		// TODO
		autoFourBallBlue.add(new NavxTurn(driveTrain, navx, 11f, 0.15f, 3.0f));// -speed 14
		// autoFourBall.add(new Wait(driveTrain, 0.5f));
		autoFourBallBlue.add(new EncoderForwardFeet(driveTrain, 11.5f, -0.8f));// 12
		autoFourBallBlue.add(new EncoderForwardFeet(driveTrain, 1f, 0.8f));
		autoFourBallBlue.add(new Wait(driveTrain, 1f));
		autoFourBallBlue.add(new EncoderForwardFeet(driveTrain, 7f, 0.8f));
		autoFourBallBlue.add(new LimelightTrack(driveTrain, shooter, limelight, 0));
		autoFourBallBlue.add(new Wait(driveTrain, 0.3f));
		autoFourBallBlue.add(new Shoot(shooter, indexer));

		// auto test
		autoTest = new LinkedList<AutoStep>();
		autoTest.add(new NavxReset(navx));
		autoTest.add(new EncoderForwardFeet(driveTrain, 5f, -0.3f));
		autoTest.add(new LimelightTrack(driveTrain, shooter, limelight, 0));
		autoTest.add(new Wait(driveTrain, 0.3f));
		autoTest.add(new Shoot(shooter, indexer));
		autoTest.add(new NavxTurn(driveTrain, navx, 0f, 0.15f, 3.0f));// -speed 14
		autoTest.add(new EncoderForwardFeet(driveTrain, 5f, 0.3f));

		// autoTest.add(new NavxReset(navx));
		// autoTest.add(new Wait(driveTrain, 3f));
		// autoTest.add(new Shoot(shooter, indexer));
		// autoTest.add(new NavxTurn(driveTrain, navx, 11f, 0.15f, 2.0f));// -speed
		// 14.15

		// autoFourBall.add(new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID));

		double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);
		boolean TwoBall = SmartDashboard.getBoolean("TwoBall", false);
		boolean FourBallRed = SmartDashboard.getBoolean("FourBallRed", false);
		boolean FourBallBlue = SmartDashboard.getBoolean("FourBallBlue", false);
		boolean Test = SmartDashboard.getBoolean("Test", false);

		if (Test) {
			autonomousSelected = autoTest;
		} else if (FourBallRed) {
			autonomousSelected = autoFourBallRed;
		} else if (FourBallBlue) {
			autonomousSelected = autoFourBallBlue;
		} else {
			autonomousSelected = autoTwoBall;
		}
		autonomousSelected.get(0).Begin();
	}

	public void autonomousPeriodic() {
		if (indexToggle && indexer.firstBeam.get()) {
			indexToggle = false;
			SmartDashboard.putNumber("Ballcount", ((SmartDashboard.getNumber("Ballcount", 0)) + 1));
		}
		if (!indexer.firstBeam.get()) {
			indexToggle = true;
		}
		if (indexToggle2 && indexer.secondBeam.get()) {
			indexToggle2 = false;
			SmartDashboard.putNumber("Ballcount", ((SmartDashboard.getNumber("Ballcount", 0)) - 1));
		}
		if (!indexer.secondBeam.get()) {
			indexToggle2 = true;
		}
		SmartDashboard.putBoolean("RobotEnabled", true);

		// System.out.println(driveTrain.GetEncoder());
		// System.out.println(autonomousSelected);

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
			driveTrain.SetBothSpeed(0.0f);
			// currentState = RobotState.Teleop;
		}

		UpdateMotors();
	}

	public void teleopInit() {

		climber.errorSet();
		// navx.reset();
		limelight.SetLight(false);

		turnPID = new HotPID("turn", 0, 0, 0);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		shooter.Init();
		preshooterpid.Init();

		driveTrain.SetCoast();

		// Controllers
		//driver = new Joystick(0);
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

		if (indexToggle && indexer.firstBeam.get()) {
			indexToggle = false;
			SmartDashboard.putNumber("Ballcount", ((SmartDashboard.getNumber("Ballcount", 0)) + 1));
		}
		if (!indexer.firstBeam.get()) {
			indexToggle = true;
		}
		if (indexToggle2 && indexer.secondBeam.get()) {
			indexToggle2 = false;
			SmartDashboard.putNumber("Ballcount", ((SmartDashboard.getNumber("Ballcount", 0)) - 1));
		}
		if (!indexer.secondBeam.get()) {
			indexToggle2 = true;
		}

		// System.out.println(navx.getYaw() + " yaw");
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

		int shootButton = 5;
		if (operator.getRawButtonReleased(shootButton)) {
			// shooter.Reset();
			// preshooterpid.Reset();
		} else {

			if (operator.getRawButtonPressed(9)) {
				if (!opToggle) {
					opToggle = true;
				} else {
					shooter.Reset();
					preshooterpid.Reset();
					opToggle = false;
				}
			}
			if (!opToggle) {
				SmartDashboard.putBoolean("Shooter running?", true);
				shooter.Update();
			} else {
				SmartDashboard.putBoolean("Shooter running?", false);
				preshooterpid.PowerManual(0);
				shooter.PowerManual(0);
			}
		}
		if (operator.getRawButton(shootButton)) {
			toggleReset = true;
			shooter.Update();
			// get target distance from limelight
			// run indexer
			float buffer = 0.035f;// 0.02
			float speed = 0.6f;
			indexer.RunManualForward(speed, buffer);

		} else {
			// preshooterpid.PowerManual(0);
			// shooter.PowerManual(0);

			if (operator.getRawButton(7) && indexer.secondBeam.get()) {
				if (toggleReset) {
					toggleReset = false;
					shooter.Reset();
					// preshooterpid.preshooterPid.reset();
				}
				indexer.RunAutomatic();
			} else if (operator.getRawButton(1)) {
				SmartDashboard.putNumber("Ballcount", 0);
				indexer.resetBallCountD();
				indexerMotor.set(ControlMode.PercentOutput, 0.6f);
				// shooter.pid.reset();
				// preshooterpid.preshooterPid.reset();
			} else {
				indexer.RunManualForward(0, 0);
			}
		}

		// Climber Uses POV

		pov = operator.getPOV();
		if (pov != -1) {
			if (280 < pov || pov < 80) {

				climber.climbMotors(1.0f);

			} else if (100 < pov && pov < 260) {
				climber.brakeMode();
				climber.climbMotors(-0.5f);
			}
		} else {
			climber.climbMotors(0.0f);
		}

		if (operator.getRawButtonPressed(10)) {
			climber.Tilt();
		}

		// Lime Light
		if (flightStickLeft.getRawButton(2)) {
			limelight.Position(driveTrain);
			driveTrain.SetBreak();
		} else {
			driveTrain.SetCoast();

			ControllerDrive();
		}

		UpdateMotors();
	}

	public void testInit() {

		// navx.reset();
		// climber.coastMode();

		// Controllers
		//driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
		preshooterpid.Init();
		shooter.Init();
		// encodererror1 = climberOne.getSelectedSensorPosition();
		// encodererror2 = climberTwo.getSelectedSensorPosition();

	}

	public float Lerp(float v0, float v1, float t) {

		if (t < 0) {
			t = 0;

		} else if (t > 1) {
			t = 1;
		}

		return (v0 + t * (v1 - v0));
	}

	// ffloat navxTarget;
	NavxTurnPID testTurn = new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID);

	// DigitalInput beamTest = new DigitalInput(1);
	// beamTest.get();
	// intakeSeven.set(ControlMode.PercentOutput, 0.5f);
	// flightStickLeft.getRawButtonPressed(0);

	/*
	 * - run intake when flight stick top button pressed
	 * - unless the ball is in beam break then don't run the intake
	 * - run intake backwards always, regardless of ball when trigger button is
	 * pressed
	 */
	//DigitalInput beamTest = new DigitalInput(1);

	public void testPeriodic() {
		/*
		if(beamTest.get()){ 
			if (flightStickLeft.getRawButton(2)) {
				intakeSeven.set(ControlMode.PercentOutput, 0.5f);
			} else if(flightStickLeft.getRawButton(1)) {
				intakeSeven.set(ControlMode.PercentOutput, -0.5f);
			} else {
				intakeSeven.set(ControlMode.PercentOutput, 0f);
			}

		}
		else {
			intakeSeven.set(ControlMode.PercentOutput, 0f);
		}
		*/
		

	

		/*
		 * frontLeft.getSelectedSensorPosition();
		 * 
		 * 
		 * preShooterFive.set(ControlMode.PercentOutput, (preShooterFeed.calculate(15,
		 * 100)/RobotController.getBatteryVoltage()));
		 * 
		 * 
		 * // System.out.println(preShooterFive.getSelectedSensorVelocity()/2048*8.19 +
		 * "Calc");
		 * // System.out.println(preShooterFive.getSelectedSensorVelocity()/2048*600 +
		 * "RPM");
		 * // preShooterFive.set(ControlMode.PercentOutput, 0);
		 * SmartDashboard.putNumber("Shooter Rot Target", 0);
		 * 
		 * shooter.PowerManual(0);
		 * System.out.println(frontLeft.getSelectedSensorPosition());
		 * 
		 * 
		 * driveTrain.SetBothSpeed(0);
		 * 
		 * driveTrain.SetCoast();
		 * climber.coastMode();
		 * 
		 * // forTesting.set(ControlMode.PercentOutput, 0.5f);
		 * // limelight.SetLight(true);
		 * 
		 * if (flightStickLeft.getRawButtonPressed(0)) {
		 * // navxTarget
		 * navx.reset();
		 * testTurn = new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID);
		 * }
		 * 
		 * // testTurn.Update();
		 */
		shooter.PowerManual(0);
		System.out.println(frontLeft.getSelectedSensorPosition());
		driveTrain.SetBothSpeed(0);
		driveTrain.SetCoast();
		climber.coastMode();
		intakeSeven.set(ControlMode.PercentOutput, 0f);
		 UpdateMotors();
	}

	public void ControllerDrive() {
		if (arcadeDrive) {

			// Arcade
			// float horJoystick = TranslateController((float) driver.getRawAxis(2));
			// float verJoystick = TranslateController((float) driver.getRawAxis(1));

			//float horJoystick = DriveScaleSelector((float) driver.getRawAxis(2), DriveScale.parabala);
			//float verJoystick = DriveScaleSelector((float) driver.getRawAxis(1), DriveScale.parabala);

			//driveTrain.SetRightSpeed(-verJoystick + -horJoystick);
			//driveTrain.SetLeftSpeed(-verJoystick + horJoystick);
			// driveTrain.SetCoast();
		} else {
			// tank
			// float leftJoystick = DriveScaleSelector((float)
			// flightStickLeft.getRawAxis(1), DriveScale.parabala);
			// float rightJoystick = (DriveScaleSelector((float)
			// flightStickRight.getRawAxis(1), DriveScale.parabala));
			// driveTrain.SetRightSpeed((-rightJoystick));
			// driveTrain.SetLeftSpeed((-leftJoystick));

			float leftJoystick = DriveScaleSelector((float) flightStickLeft.getRawAxis(1), DriveScale.linear);
			float rightJoystick = (DriveScaleSelector((float) flightStickRight.getRawAxis(0), DriveScale.linear));

			driveTrain.SetRightSpeed(-leftJoystick + -rightJoystick);
			driveTrain.SetLeftSpeed(-leftJoystick + rightJoystick);
			//System.out.println("Right " + (-leftJoystick + -rightJoystick));
			//System.out.println("Left " + (-leftJoystick + rightJoystick));
			driveTrain.Update();

			// driveTrain.SetRightSpeed(leftJoystick);
			// driveTrain.SetLeftSpeed(rightJoystick);
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
		SmartDashboard.putBoolean("intakeExtended", true);
		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void intakeUp() {
		SmartDashboard.putBoolean("intakeExtended", false);
		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}
}