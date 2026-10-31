
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.GlobalConstants;

public class Turret {

    private final Servo turret1;
    private final Servo turret2;

    private double targetAngle = 0;

    public static final double GOAL_X = GlobalConstants.GOAL_X;
    public static final double GOAL_Y = GlobalConstants.GOAL_Y;

    private static final double MIN_ANGLE = -120;
    private static final double MAX_ANGLE = 140;

    // Full positional servo rotation
    private static final double SERVO_RANGE = 350.0;

    // Servo positions when the turret faces forward
    public static double SERVO1_CENTER = 0.5;
    public static double SERVO2_CENTER = 0.5;

    public Turret(HardwareMap hardwareMap) {

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret1.setDirection(Servo.Direction.FORWARD);
        turret2.setDirection(Servo.Direction.REVERSE);

        // This assumes both servos are mechanically aligned at center.
        setTargetAngle(0);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(double angle) {

        angle = normalizeDegrees(angle);

        // Limit turret rotation
        angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));

        targetAngle = angle;

        update();
    }

    public void update() {

        // Convert target angle to servo positions
        double position1 = SERVO1_CENTER + targetAngle / SERVO_RANGE;
        double position2 = SERVO2_CENTER + targetAngle / SERVO_RANGE;

        position1 = Math.max(0.0, Math.min(1.0, position1));
        position2 = Math.max(0.0, Math.min(1.0, position2));

        turret1.setPosition(position1);
        turret2.setPosition(position2);
    }

    public double autoAim(double robotX, double robotY, double robotHeadingDeg) {
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngle = fieldAngleToGoal - robotHeadingDeg + 180;
        return normalizeDegrees(turretAngle);
    }

    public double normalizeDegrees(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    // Manual turret angle control
    public void manual(double angle) {
        setTargetAngle(angle);
    }
}
