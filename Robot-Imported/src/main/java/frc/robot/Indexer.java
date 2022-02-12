package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import javax.lang.model.util.ElementScanner6;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Indexer {

	public int ballCount = 0;
    
	public boolean firstInitialTrigger = false;
    
    public DigitalInput firstBeam = new DigitalInput(1);
    public DigitalInput secondBeam = new DigitalInput(0);
        
    public TalonSRX indexerMotor = new TalonSRX(2);

    private Shooter shooter;
    private PreShooterpid preShooter;

    public Indexer(Shooter shooter, PreShooterpid preShooter) {
        this.shooter = shooter;
        this.preShooter = preShooter;
    }

    public void RunManualForward(float speed, float RPMBuffer) {
        ballCount = 0;

        if (shooter.UpToSpeed(RPMBuffer) && preShooter.UpToSpeed(RPMBuffer)) {
            indexerMotor.set(ControlMode.PercentOutput, speed);
        } else {
            indexerMotor.set(ControlMode.PercentOutput, 0);
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

            targetSpeed = -0.2f;
        }

        if (ballCount == 1) {
            if (!firstBeam.get() && secondBeam.get()) {
                targetSpeed = -0.4f;
            }
        }
        System.out.println(targetSpeed+ " target speed");
        indexerMotor.set(ControlMode.PercentOutput, targetSpeed);
    }

    public void DebugPrint() {
        //System.out.println(" 1-" + beamBreakOne.get() + " top-" + beamBreakTop.get());
    }
}