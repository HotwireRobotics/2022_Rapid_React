package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DriveTrain;

public class Limelight {

    float inverted = 1;

    public float turnBuffer;

    private Timer timer = new Timer();

    // max is when the robot is far from the goal, min is when we're near the goal
    public float maxSpeed;
    public float minSpeed;

    public float xAdjust;

    public Limelight() {
        timer.reset();
        timer.start();
    }

    public void AutoSettings() {
        turnBuffer = 0.55f;
        maxSpeed = 0.25f;
        minSpeed = 0.15f;
    }

    public void TeleopSettings() {
        turnBuffer = 0.2f;  //0.7  //0.4
        maxSpeed = 0.4f;
        minSpeed = 0.25f;  //0.15
    }

    public void SetLight(boolean turnOn) {
        if (turnOn) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);// 1
        }
    }

    public double GetArea() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        if (tv.getDouble(0.0f) > 0) {
            return ta.getDouble(0.0);
        }

        return 0.0;
    }

    public boolean OnTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        // Make sure we have valid targets first
        if (tv.getDouble(0.0f) > 0) {

            double x = tx.getDouble(0.0) + xAdjust;
            double y = ty.getDouble(0.0);
            double area = ta.getDouble(0.0);

            NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

            if (x * inverted > turnBuffer) {
                return false;
            } else if (x * inverted < -turnBuffer) {
                return false;
            } else {
                return true;
            }
        } else {
        }
        return true;
    }

    public boolean Position(DriveTrain driveTrain, float inverted, double xAdj) {

        this.xAdjust = (float)xAdj;

        SetLight(true);
        driveTrain.SetBreak();

        // Flip xadjust for easier tuning. Set xadjust negative to aim rigt, positive to
        // aim left.
        xAdjust = xAdjust * -1;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        // Make sure we have valid targets first
        if (tv.getDouble(0.0f) > 0) {

            double x = tx.getDouble(0.0) + xAdjust;
            double y = ty.getDouble(0.0);
            double area = ta.getDouble(0.0);

            NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

            float maxArea = 3.4f;
            float minArea = 0.26f;
            float currentAreaPercentage = ((float) area - minArea) / (maxArea - minArea);

            // float currentSpeed = Lerp(minSpeed, maxSpeed, currentAreaPercentage);
            float minspeed = 0.1f;
            float p = 0.04f;
            float currentSpeed = (float) Math.abs(x) * p; 
            if (currentSpeed < minspeed) {
                currentSpeed = minspeed;
            }
            float turnSpeedSlow = -currentSpeed * 0.25f;

            if (x * inverted > turnBuffer) {

                driveTrain.SetLeftSpeed(currentSpeed * inverted);
                driveTrain.SetRightSpeed(turnSpeedSlow * inverted);

                timer.reset();
                timer.start();

            } else if (x * inverted < -turnBuffer) {

                driveTrain.SetLeftSpeed(turnSpeedSlow * inverted);
                driveTrain.SetRightSpeed(currentSpeed * inverted);

                timer.reset();
                timer.start();
            } else {

                driveTrain.SetLeftSpeed(0.0f);
                driveTrain.SetRightSpeed(0.0f);

                if (timer.get() > 0.1) {
                    return true;
                }
            }
        } else {
            driveTrain.SetBothSpeed(0.0f);
        }

        return false;
    }

    public float Lerp(float v0, float v1, float t) {

        if (t < 0) {
            t = 0;

        } else if (t > 1) {
            t = 1;
        }

        return (v0 + t * (v1 - v0));
    }
}