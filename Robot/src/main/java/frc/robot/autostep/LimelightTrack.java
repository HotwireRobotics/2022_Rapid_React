package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Shooter;
import frc.robot.Limelight;

public class LimelightTrack extends AutoStep {

    public DriveTrain driveTrain;
    public Shooter shooter;
    public Limelight limelight;
    public double xAdjust;

    public LimelightTrack(DriveTrain driveTrain, Shooter shooter, Limelight limelight, double xAdjust) {
        super();
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.limelight = limelight;
        this.xAdjust = xAdjust;
    }

    public void Begin() {

    }

    public void Update() {
        if (limelight.Position(driveTrain, 1, xAdjust)) {
          isDone = true;
        }
    }
}