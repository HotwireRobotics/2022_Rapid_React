package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Indexer;

public class EncoderForward extends AutoStep {

    public float encoderTarget;
    public float speed;
    public double encoderStart;

    public DriveTrain driveTrain;

    public EncoderForward(DriveTrain driveTrain, float encoderTarget, float speed) {
        super();
        this.encoderTarget = encoderTarget;
        this.speed = speed;
        this.driveTrain = driveTrain;
    }

    public void Begin() {
        driveTrain.SetBothSpeed(speed);
        encoderStart = driveTrain.GetEncoder();
    }

    public void Update() {

        double currentError = Math.abs(encoderStart - driveTrain.GetEncoder());
        if (currentError > 150000) {
            System.out.println("ERROR - Encoder target is too large. Something probably broke!");
            driveTrain.SetBothSpeed(0.0f);
        } else {
            if (currentError > encoderTarget) {
                isDone = true;
                driveTrain.SetBothSpeed(0.0f);
            }
            System.out.println(encoderStart - driveTrain.GetEncoder());
        }
    }
}