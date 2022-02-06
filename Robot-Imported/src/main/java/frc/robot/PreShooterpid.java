package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class PreShooterpid {
    
    public TalonSRX preShooterFive = new TalonSRX(6);
    
    public Limelight limelight;

    public double preshooterP = 0.0001;
    public double preshooterI = 0.0004;
    public double preshooterD = 0.00001;
    public double preshooterRPMTarget;

    public boolean didHitSpeed = true;

    String preshooterPKey = "preShooter_P";
    String preshooterIKey = "preShooter_I";
    String preshooterDKey = "preShooter_D";
    String preshooterRPMKey = "preShooter_RPMTarget";

    public PIDController preshooterPid = new PIDController(preshooterP, preshooterI, preshooterD);

    public double preRpmTarget = 0;

    public float preRpmCurrent = 0;

    public PreShooterpid(Limelight limelight) {
        this.limelight = limelight;
    } 

    public void Init() {
        SmartDashboard.putNumber(preshooterPKey, preshooterP);
        SmartDashboard.putNumber(preshooterIKey, preshooterI);
        SmartDashboard.putNumber(preshooterDKey, preshooterD);
        SmartDashboard.putNumber(preshooterRPMKey, preshooterRPMTarget);

        preshooterPid = new PIDController(preshooterP, preshooterI, preshooterD);
    }

    public void Update() {


        preRpmCurrent = TalonVelocityToRPM((float)preShooterFive.getSelectedSensorVelocity());
        System.out.println(preRpmTarget + " prerpmtarget");

        // Shooter code pid
        SmartDashboard.putNumber("preShooter_RPM", preRpmCurrent);

        preshooterP = SmartDashboard.getNumber(preshooterPKey, preshooterP);
        preshooterI = SmartDashboard.getNumber(preshooterIKey, preshooterI);
        preshooterD = SmartDashboard.getNumber(preshooterDKey, preshooterD);

        preshooterPid.setP(preshooterP);
        preshooterPid.setI(preshooterI);
        preshooterPid.setD(preshooterD);

        double motorSpeed = preshooterPid.calculate(preRpmCurrent, preRpmTarget);

        if (motorSpeed < 0) {
            motorSpeed = 0;
        }
        float max = 0.95f;
        if (motorSpeed >= max) {
            motorSpeed = max;
        }

        // System.out.println("Speed " + motorSpeed + " RPM " + rpm);
        SmartDashboard.putNumber("PreShooter_Speed", -motorSpeed);
        PowerManual((float) -motorSpeed);
    }

    public boolean UpToSpeed(float RPMBuffer) {
        double distance = Math.abs(preRpmCurrent - preRpmTarget);
        return distance < (preRpmTarget * RPMBuffer);
    }

    public void PowerManual(float power) {
        preShooterFive.set(ControlMode.PercentOutput, power);
    }

    public float TalonVelocityToRPM(float ticks) {
        float rpm = ((ticks / 2048) * 600);
        return Math.abs(rpm);
    }
}
