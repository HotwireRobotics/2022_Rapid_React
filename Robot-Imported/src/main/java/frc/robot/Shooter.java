package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Shooter {

    public TalonSRX shooterTwo = new TalonSRX(4);  //51
    public TalonSRX shooterFive = new TalonSRX(50);
    
    public Limelight limelight;
    public PreShooterpid preshooterpid;

    public double shooterP = 0.0001;
    public double shooterI = 0.0004;
    public double shooterD = 0.000;
    public double shooterRPMTarget;

    public boolean didHitSpeed = true;

    String shooterPKey = "Shooter_P";
    String shooterIKey = "Shooter_I";
    String shooterDKey = "Shooter_D";
    String shooterRPMKey = "Shooter_RPMTarget";

    public PIDController shooterPid = new PIDController(shooterP, shooterI, shooterD);

    public double rpmTarget = 0;

    public float rpmCurrent = 0;

    public Shooter(Limelight limelight, PreShooterpid preshooterpid) {
        this.limelight = limelight;
        this.preshooterpid = preshooterpid;
    } 

    public void Init() {
        SmartDashboard.putNumber(shooterPKey, shooterP);
        SmartDashboard.putNumber(shooterIKey, shooterI);
        SmartDashboard.putNumber(shooterDKey, shooterD);
        SmartDashboard.putNumber(shooterRPMKey, shooterRPMTarget);

        shooterPid = new PIDController(shooterP, shooterI, shooterD);
    }

    public void Update() {

        if (limelight.gety() >= 0) {
            rpmTarget = 2150;
            preshooterpid.preRpmTarget = 3000;
        } else if (limelight.gety() > -11.5) {
            rpmTarget = 2300;
            preshooterpid.preRpmTarget = 2500;
        } else {
            rpmTarget = 2700;
            preshooterpid.preRpmTarget = 2222;
        }
        preshooterpid.Update();

        rpmCurrent = TalonVelocityToRPM((float)shooterTwo.getSelectedSensorVelocity());

        // Shooter code pid
        SmartDashboard.putNumber("Shooter_RPM", rpmCurrent);

        shooterP = SmartDashboard.getNumber(shooterPKey, shooterP);
        shooterI = SmartDashboard.getNumber(shooterIKey, shooterI);
        shooterD = SmartDashboard.getNumber(shooterDKey, shooterD);

        shooterPid.setP(shooterP);
        shooterPid.setI(shooterI);
        shooterPid.setD(shooterD);

        double motorSpeed = shooterPid.calculate(rpmCurrent, rpmTarget);

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
