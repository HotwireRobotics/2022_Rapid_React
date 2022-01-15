package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
	JoshMotorControllor joshmotorcontrollorLeftBottomOne;
	JoshMotorControllor joshmotorcontrollorLeftBottomTwo;
	JoshMotorControllor joshmotorcontrollorRightBottomOne;
	JoshMotorControllor joshmotorcontrollorRightBottomTwo;
	float lerpSpeed = 0.8f;
	public AHRS navx;

	public DriveTrain(int pwm1, int pwm2, int pwm3, int pwm4, AHRS navx) {
		joshmotorcontrollorLeftBottomOne = new JoshMotorControllor(pwm1, lerpSpeed);
		joshmotorcontrollorLeftBottomTwo = new JoshMotorControllor(pwm2, lerpSpeed);
		joshmotorcontrollorRightBottomOne = new JoshMotorControllor(pwm3, lerpSpeed);
		joshmotorcontrollorRightBottomTwo = new JoshMotorControllor(pwm4, lerpSpeed);

		this.navx = navx;
	}

	public void Update() {
		joshmotorcontrollorLeftBottomOne.UpdateMotor();
		joshmotorcontrollorLeftBottomTwo.UpdateMotor();
		joshmotorcontrollorRightBottomOne.UpdateMotor();
		joshmotorcontrollorRightBottomTwo.UpdateMotor();
	}

	public void SendData() {
		SmartDashboard.putNumber("DriveTrain_LeftOne", joshmotorcontrollorLeftBottomOne.talon.getTemperature());
		SmartDashboard.putNumber("DriveTrain_LeftTwo", joshmotorcontrollorLeftBottomTwo.talon.getTemperature());
		SmartDashboard.putNumber("DriveTrain_RightOne", joshmotorcontrollorRightBottomOne.talon.getTemperature());
		SmartDashboard.putNumber("DriveTrain_RightTwo", joshmotorcontrollorRightBottomTwo.talon.getTemperature());
	}

	public void SetLeftSpeed(float Speed) {
		joshmotorcontrollorLeftBottomOne.target = Speed;
		joshmotorcontrollorLeftBottomTwo.target = Speed;
	}

	public void SetRightSpeed(float Speed) {
		joshmotorcontrollorRightBottomOne.target = -Speed;
		joshmotorcontrollorRightBottomTwo.target = -Speed;
	}

	public void SetBothSpeed(float Speed) {
		SetLeftSpeed(Speed);
		SetRightSpeed(Speed);
	}

	public void SetBreak() {
		joshmotorcontrollorRightBottomOne.SetBrake();
		joshmotorcontrollorRightBottomTwo.SetBrake();
		joshmotorcontrollorLeftBottomOne.SetBrake();
		joshmotorcontrollorLeftBottomTwo.SetBrake();
	}

	public void SetCoast() {
		joshmotorcontrollorRightBottomOne.SetCoast();
		joshmotorcontrollorRightBottomTwo.SetCoast();
		joshmotorcontrollorLeftBottomOne.SetCoast();
		joshmotorcontrollorLeftBottomTwo.SetCoast();

	}

	public void DriveStraight(float speed, boolean reverse) {
		// float pidError = (float) turnController.get();
		float pidError = 0f;
		SetLeftSpeed((speed * pidError) + speed); // 0.6972
		SetRightSpeed(((speed) - (speed * pidError)) * -1); // -0.583

		speed = -speed;
		if (reverse) {
			speed = -speed;
		}
	}

	public double GetEncoder() {
		return joshmotorcontrollorLeftBottomOne.talon.getSelectedSensorPosition();
	}
}