package frc.robot.autostep;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.DriveTrain;

public class IntakeDrop extends AutoStep {

public DoubleSolenoid intakeSolenoid;

    public IntakeDrop(DoubleSolenoid intakeSolenoid) {
        super();
        this.intakeSolenoid = intakeSolenoid;
    }

    public void Begin() {     
		SmartDashboard.putBoolean("intakeExtended", true);
		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void Update() {
        isDone = true;
    }
}