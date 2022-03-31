package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import javax.lang.model.util.ElementScanner6;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Indexer {
    public int ballCountD;
    public int ballCount = 0;

    public boolean firstInitialTrigger = false;

    public DigitalInput firstBeam = new DigitalInput(1);
    public DigitalInput secondBeam = new DigitalInput(0);

    public TalonSRX indexerMotor = new TalonSRX(9);

    private Shooter shooter;
    private PreShooterpid preShooter;

    boolean toggleencoder = false;
    boolean didPause = true;
    boolean shooting = false;
    Timer timer = new Timer();
    boolean toggle2 = true;

    private float pauseTime = 0.2f;

    public Indexer(Shooter shooter, PreShooterpid preShooter) {
        this.shooter = shooter;
        this.preShooter = preShooter;
    }
    public void resetBallCountD(){
        ballCountD = 0;
    }
    public void RunManualForward(float speed, float RPMBuffer) {
        ballCount = 0;
 
        SmartDashboard.putNumber("ball counter", ballCountD);
        indexerMotor.setNeutralMode(NeutralMode.Brake);
        boolean upToSpeed = shooter.UpToSpeed(RPMBuffer) && preShooter.UpToSpeed(RPMBuffer);
        if (!secondBeam.get() && toggle2){
            toggle2 = false;
            timer.reset();
            timer.start();
        }

        if (secondBeam.get()) {
            
            System.out.println(speed + " speed");

            toggle2 = true;
            toggleencoder = false;
            indexerMotor.set(ControlMode.PercentOutput, -speed);

        } else if (timer.hasElapsed(0.25) && upToSpeed && (indexerMotor.getSelectedSensorVelocity() == 0) || toggleencoder) {
            System.out.println("shooting!!!");
            ballCountD = 0;
            toggleencoder = true;
            indexerMotor.set(ControlMode.PercentOutput, -speed);
        } else {
            indexerMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void RunAutomatic() {
        SmartDashboard.putNumber("ball counter", ballCountD);
        toggleencoder = false;
        if (secondBeam.get()) {
            float targetSpeed = 0;

            if (ballCount == 0) {
                if (!firstBeam.get()) {
                    firstInitialTrigger = true;
                    ballCountD = 1;
                }

                if (firstInitialTrigger && firstBeam.get()) {
                    ballCount = 1;
                }

                targetSpeed = 0.4f;
            }

            if (ballCount == 1) {
                if (!firstBeam.get() && secondBeam.get()) {
                    ballCountD = 2;
                    targetSpeed = 0.5f;
                }
            }
            System.out.println(targetSpeed + " target speed");
            indexerMotor.set(ControlMode.PercentOutput, -targetSpeed);
        } else {
            indexerMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void DebugPrint() {
        // System.out.println(" 1-" + beamBreakOne.get() + " top-" +
        // beamBreakTop.get());
    }
}