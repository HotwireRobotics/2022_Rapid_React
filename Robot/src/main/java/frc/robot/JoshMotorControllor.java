package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.VictorSP;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class JoshMotorControllor {

	public TalonSRX talon;
	public float accelValue;
	public float target;

	public JoshMotorControllor(int motorpwm, float AcelerationMax) {
		talon = new TalonSRX(motorpwm);
		accelValue = AcelerationMax;
	}

	public void UpdateMotor() {
		if (talon != null) {
			double curr = talon.getMotorOutputPercent();

			float newValue = Lerp((float) curr, target, accelValue);

			float epsilon = 0.001f;
			if (newValue < epsilon && newValue > -epsilon) {
				newValue = 0.0f;
			}

			talon.set(ControlMode.PercentOutput, target);
		}
	}

	public float Lerp(float v0, float v1, float t) {
		return (v0 + t * (v1 - v0));
	}

	public void SetBrake() {
		talon.setNeutralMode(NeutralMode.Brake);
	}

	public void SetCoast() {
		talon.setNeutralMode(NeutralMode.Coast);
	}
}