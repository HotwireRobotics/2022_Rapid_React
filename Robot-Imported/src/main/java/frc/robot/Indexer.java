package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Indexer {

    public DigitalInput beamBreakOne = new DigitalInput(3); // 3
    public DigitalInput beamBreakTop = new DigitalInput(1); // 1

    boolean beamBreakClear = true;
    int ballCounter = 0;
    boolean shooterStarted = false;
    int loopCount = 0;

    public boolean manualFloorBelt = true;

    public TalonSRX indexerFive = new TalonSRX(9);
    public TalonSRX floorBeltEight = new TalonSRX(1);

    // public double indexerSpeed = 0.4f;

    public boolean firstAutomatic = false;
    public boolean firstClear = true;

    private Shooter shooter;
    double ticksStart = 0;
    private double ticksTarget;

    private Timer fiveBallTimer = new Timer();
    private boolean initBallFive = true;

    public Indexer(Shooter shooter) {
        this.shooter = shooter;
        firstAutomatic = true;
    }

    public void RunManualForward(float speed, float RPMBuffer) {
        ballCounter = 0;
        loopCount = 0;
        shooterStarted = false;
        firstAutomatic = true;
        initBallFive = true;

        // DebugPrint();

        // ball - false
        // no ball - true
        //System.out.println(" SHOOTER " + shooter.UpToSpeed(RPMBuffer) + " - BEAM " + beamBreakTop.get());
        if (shooter.UpToSpeed(RPMBuffer) || beamBreakTop.get()) {
            //System.out.println("MOVING");
            indexerFive.set(ControlMode.PercentOutput, -speed);
            // floorBeltEight.set(ControlMode.PercentOutput, speed * 0.5f);
        } else {
            //System.out.println("STOPING");
            indexerFive.set(ControlMode.PercentOutput, 0.0f);
            // floorBeltEight.set(ControlMode.PercentOutput, 0.0f);
        }

        if (beamBreakOne.get()) {
            floorBeltEight.set(ControlMode.PercentOutput, speed);
        } else {
            floorBeltEight.set(ControlMode.PercentOutput, 0.0f);
        }
    }

    public void RunAutomatic(boolean runFloorbelt) {

        double floorBeltSpeed = 1.0f;

        // DebugPrint();

        if (!runFloorbelt) {
            floorBeltEight.set(ControlMode.PercentOutput, 0.0f);
        }

        // Stop the indexer the first time we start automatic control
        if (firstAutomatic) {
            firstAutomatic = false;
            indexerFive.set(ControlMode.PercentOutput, 0.0f);
            if (runFloorbelt) {
                floorBeltEight.set(ControlMode.PercentOutput, floorBeltSpeed);
            }
            ticksTarget = indexerFive.getSelectedSensorPosition();
        }

        if (beamBreakOne.get()) {
            firstClear = true;
        }

        if (firstClear && !beamBreakOne.get()) {
            ballCounter++;
            firstClear = false;
            ticksTarget = indexerFive.getSelectedSensorPosition() + 110000;
        }

        //System.out.println("ballcounter " + ballCounter);

        if (ballCounter <= 4) {

            double buffer = 5000;  //2000
            double distance = Math.abs(indexerFive.getSelectedSensorPosition() - ticksTarget);
            double pValue = 0.000014f;// 0.000012f

            if (ballCounter == 1) {
                pValue = pValue * 0.9f;
            }

            if (ballCounter == 4) {
                pValue = pValue * 1.1;
            }

            double speed = distance * pValue;
            speed = MathUtil.clamp(speed, 0.4f, 1.0f);

            double floorBackSpeed = -0.2f;
            //System.out.println("speed of index " + speed);
            if (indexerFive.getSelectedSensorPosition() > (ticksTarget + buffer)) {
                indexerFive.set(ControlMode.PercentOutput, speed);
                if (runFloorbelt) {
                    floorBeltEight.set(ControlMode.PercentOutput, floorBackSpeed);
                }
            } else if (indexerFive.getSelectedSensorPosition() < (ticksTarget - buffer)) {
                indexerFive.set(ControlMode.PercentOutput, -speed);
                if (runFloorbelt) {
                    floorBeltEight.set(ControlMode.PercentOutput, floorBackSpeed);
                }
            } else {
                //System.out.println("hit target");
                indexerFive.set(ControlMode.PercentOutput, 0.0f);
                if (runFloorbelt) {
                    floorBeltEight.set(ControlMode.PercentOutput, floorBeltSpeed);
                }
            }

            /*
             * if (Math.abs(indexerFive.getSelectedSensorPosition() - ticksStart) <
             * ticksToMove) { indexerFive.set(ControlMode.PercentOutput, -indexerSpeed);
             * floorBeltEight.set(ControlMode.PercentOutput, 0.0f); } else {
             * indexerFive.set(ControlMode.PercentOutput, 0.0f);
             * floorBeltEight.set(ControlMode.PercentOutput, floorBeltSpeed); }
             */

        } else if (ballCounter == 5) {

            if (initBallFive) {
                initBallFive = false;
                fiveBallTimer.reset();
                fiveBallTimer.start();
            }

            if (fiveBallTimer.get() < 0.1f) {
                indexerFive.set(ControlMode.PercentOutput, -0.3f);
                floorBeltEight.set(ControlMode.PercentOutput, 0.0f);
            } else {
                indexerFive.set(ControlMode.PercentOutput, 0.0f);
                floorBeltEight.set(ControlMode.PercentOutput, 0.0f);
            }
        } else {
            indexerFive.set(ControlMode.PercentOutput, 0.0f);
            floorBeltEight.set(ControlMode.PercentOutput, 0.0f);
        }
    }

    public void DebugPrint() {
        //System.out.println(" 1-" + beamBreakOne.get() + " top-" + beamBreakTop.get());
    }
}