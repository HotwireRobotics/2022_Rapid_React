package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.counter.UpDownCounter;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import javax.lang.model.util.ElementScanner6;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Indexer {

    public int ballCount = 0;

    public boolean firstInitialTrigger = false;

    public DigitalInput firstBeam = new DigitalInput(1);
    public DigitalInput secondBeam = new DigitalInput(0);

    public TalonSRX indexerMotor = new TalonSRX(9);

    private Shooter shooter;
    private PreShooterpid preShooter;

    boolean didPause = true;
    boolean shooting = false;
    Timer timer = new Timer();

    private float pauseTime = 0.2f;

    public Indexer(Shooter shooter, PreShooterpid preShooter) {
        this.shooter = shooter;
        this.preShooter = preShooter;
    }

    public void RunManualForward(float speed, float RPMBuffer) {
        ballCount = 0;
        indexerMotor.setNeutralMode(NeutralMode.Brake);
        /*
        if(secondBeam.get()){
            indexerMotor.set(ControlMode.PercentOutput, -speed);
            return;
        }
        */

        /* 
        boolean toggleencoder;
        if(secondBeam.get()){
            toggleencoder = false
            indexerMotor.set(ControlMode.PercentOutput, -speed);
        }else if(upToSpeed && encoder.getStopped || toggleencoder) {
            toggleencoder = true
            indexerMotor.set(ControlMode.PercentOutput, -speed);
        }else{
            indexerMotor.set(ControlMode.PercentOutput, 0);
        }
        */
        boolean upToSpeed = shooter.UpToSpeed(RPMBuffer) && preShooter.UpToSpeed(RPMBuffer);

        if (!upToSpeed) {
            indexerMotor.set(ControlMode.PercentOutput, 0.0);
            return;
        }

        if (shooting) {
            indexerMotor.set(ControlMode.PercentOutput, -speed);
            if (secondBeam.get()) {
                shooting = false;
            }
        } else {

            if (!secondBeam.get() && !didPause) {
                speed = 0;
                if (timer.get() > pauseTime) {
                    shooting = true;
                    didPause = true;
                } else {
                    didPause = false;
                }

            } else {
                timer.reset();
                timer.start();
                didPause = false;
            }

            System.out.println(speed);
            indexerMotor.set(ControlMode.PercentOutput, -speed);
        }
    }

    public void RunAutomatic() {

        float targetSpeed = 0;

        if (ballCount == 0) {
            if (!firstBeam.get()) {
                firstInitialTrigger = true;
            }

            if (firstInitialTrigger && firstBeam.get()) {
                ballCount = 1;
            }

            targetSpeed = 0.2f;
        }

        if (ballCount == 1) {
            if (!firstBeam.get() && secondBeam.get()) {
                targetSpeed = 0.4f;
            }
        }
        System.out.println(targetSpeed + " target speed");
        indexerMotor.set(ControlMode.PercentOutput, targetSpeed);
    }

    public void DebugPrint() {
        // System.out.println(" 1-" + beamBreakOne.get() + " top-" +
        // beamBreakTop.get());
    }
}