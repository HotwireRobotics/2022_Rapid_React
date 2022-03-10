package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Shooter {

    public TalonSRX shooterTwo = new TalonSRX(6);  
    public TalonSRX shooterFive = new TalonSRX(50);
    
    public Limelight limelight;
    public PreShooterpid preshooterpid;

    public HotPID pid;

    public boolean didHitSpeed = true;

    public double rpmTarget = 0;

    public float rpmCurrent = 0;

    public Shooter(Limelight limelight, PreShooterpid preshooterpid) {

        this.limelight = limelight;
        this.preshooterpid = preshooterpid;
    } 

    public void Init() {
        pid = new HotPID("shooter", 0.0001, 0.0004, 0);
    }
    public void Reset(){
        pid.reset();
    }
    public void Update() {

        if (limelight.gety() >= -1.9) {
            rpmTarget = 2000;
            preshooterpid.preRpmTarget = 3000;
        } else if (limelight.gety() > -100) {
            rpmTarget = 2750;
            //really good possible bounce 2750 2000
            preshooterpid.preRpmTarget = 2000;
            // rpmTarget = 2300;
            // preshooterpid.preRpmTarget = 2500;
        } else {
            rpmTarget = 2150;
            preshooterpid.preRpmTarget = 4000;
            // rpmTarget = 2700;
            // preshooterpid.preRpmTarget = 2222;
        }
        preshooterpid.Update();

        rpmCurrent = TalonVelocityToRPM((float)shooterTwo.getSelectedSensorVelocity());

        // Shooter code pid
        SmartDashboard.putNumber("Shooter_RPM", rpmCurrent);

        pid.setpoint = rpmTarget;
        double motorSpeed = pid.Calculate(rpmCurrent);

        if (motorSpeed < 0) {
            motorSpeed = 0;
        }
        float max = 0.95f;
        if (motorSpeed >= max) {
            motorSpeed = max;
        }
        // System.out.println("Speed " + motorSpeed + " RPM " + rpm);
        SmartDashboard.putNumber("Shooter_Speed", motorSpeed);
        if (rpmTarget == 0.0) {
            PowerManual(0);
        } else {
            PowerManual((float) motorSpeed);
        }
    }

    public boolean UpToSpeed(float RPMBuffer) {
        double distance = Math.abs(rpmCurrent - rpmTarget);
        return distance < (rpmTarget * RPMBuffer);
    }

    public void PowerManual(float power) {
        shooterFive.set(ControlMode.PercentOutput, power);
        shooterTwo.set(ControlMode.PercentOutput, power * -1);
    }

    public float TalonVelocityToRPM(float ticks) {
        float rpm = ((ticks / 2048) * 600);
        return Math.abs(rpm);
    }
}
