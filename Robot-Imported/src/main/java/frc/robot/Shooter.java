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
    public double shooterP = 0.0001;
    public double shooterI = 0.0004;
    public double shooterD = 0.0;
    

    public HotPID pid;

    public boolean didHitSpeed = true;

    public double rpmTarget = 0;

    public float rpmCurrent = 0;

    public Shooter(Limelight limelight, PreShooterpid preshooterpid) {


        this.limelight = limelight;
        this.preshooterpid = preshooterpid;
    }

    public void Init() {
        SmartDashboard.putNumber("ShooterP", shooterP);
        SmartDashboard.putNumber("ShooterI", shooterI);
        SmartDashboard.putNumber("ShooterD", shooterD);
        shooterP = SmartDashboard.getNumber("ShooterP", shooterP);
        shooterI = SmartDashboard.getNumber("ShooterI", shooterI);
        shooterD = SmartDashboard.getNumber("ShooterD", shooterD);


        pid = new HotPID("shooter", shooterP, shooterI, shooterD);
    }

    public void Reset() {

        pid.reset();
    }
    

    public void Update() {

        // if (limelight.gety() >= -1.9) {
        // rpmTarget = 2000;
        // preshooterpid.preRpmTarget = 3000;
        // } else if (limelight.gety() > -100) {
        System.out.println("Second zone");
        rpmTarget = 1650;// 2750 act 2650//2000far//1800//mid bumper line1600,2000
        preshooterpid.preRpmTarget = 2000;// 1800far
        // } else {
        // rpmTarget = 2150;
        // preshooterpid.preRpmTarget = 4000;
        // // rpmTarget = 2700;
        // // preshooterpid.preRpmTarget = 2222;
        // }

        float nearY =35.7f;//16
        float farY = 19f;//20.7

        float dist = Math.abs(farY - nearY);
        float relDist = Math.abs((float)limelight.gety() - nearY) / dist;

        // shooter
        float nearShooterRPM = 1600;//1650
        float farShooterRPM = 2100;//hangar//2100//2050
        float shooterSpeed = Lerp(nearShooterRPM, farShooterRPM, relDist);
        rpmTarget = shooterSpeed;

        // preshooter
        float nearPreRPM = 1700;
        float farPreRPM = 1800;//1817
        float preShooterSpeed = Lerp(nearPreRPM, farPreRPM, relDist);
        preshooterpid.preRpmTarget = preShooterSpeed;

        preshooterpid.Update();

        rpmCurrent = TalonVelocityToRPM((float) shooterTwo.getSelectedSensorVelocity());

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

    public float Lerp(float v0, float v1, float t) {

        if (t <= 0) {
            t = 0;
        } else if (t >= 1) {
            t = 1;
        }

        return (v0 + t * (v1 - v0));
    }
}
