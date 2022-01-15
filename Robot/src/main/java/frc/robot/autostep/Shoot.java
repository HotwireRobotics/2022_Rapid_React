package frc.robot.autostep;

import frc.robot.Shooter;

//import javax.swing.Timer;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Indexer;

public class Shoot extends AutoStep {

    public Shooter shooter;
    public double rpmTarget;
    public Indexer indexer;
    private int ballsShot = 0;
    public boolean beamBreakClear = true;
    public Timer fifthBallTimer;
    public int ballsShotTarget = 5;
    private int tickStart = 0;

    public boolean doingEnd = false;

    public Shoot(Shooter shooter, Indexer indexer, double rpmTarget, int ballsShotTarget) {
        super();
        this.shooter = shooter;
        this.rpmTarget = rpmTarget;
        this.indexer = indexer;
        this.ballsShotTarget = ballsShotTarget;

        autoIndex = false;
    }

    public void Begin() {
        tickStart = indexer.indexerFive.getSelectedSensorPosition();
        shooter.rpmTarget = rpmTarget;
        fifthBallTimer = new Timer();
        fifthBallTimer.stop();

        if (!indexer.beamBreakTop.get()) {
            beamBreakClear = false;
            ballsShot++;
        }
    }

    public void Update() {
        if (Math.abs(tickStart - indexer.indexerFive.getSelectedSensorPosition()) < 700000){
            indexer.RunManualForward(0.6f, 0.03f);
        }else{
            isDone = true;
        }  

    }
}