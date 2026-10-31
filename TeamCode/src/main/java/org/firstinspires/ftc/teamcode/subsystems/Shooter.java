package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    public enum ShooterState {
        IDLE, SPINNING_UP, READY, FEEDING
    }
    private ShooterState state = ShooterState.IDLE;
    private final DcMotorEx flywheelMotorLeft;
    private final DcMotorEx flywheelMotorRight;
    private final Servo hoodServo;
    public final Servo gate;
    private final RGB stateLight;
    private final ElapsedTime feedTimer = new ElapsedTime();


    private double targetVelocity = 0;
    private double velocityTolerance = 50;

    public double gateClosed = 0.0;
    public double gateOpen = 0.5;
    private double feedTime = 1.0;
    private double gateOpenDelay = 0.25;

    private double kP = 0.0008;
    private double kS = 0.01;
    private double kV = 0.00036;

    public Shooter(HardwareMap hardwareMap) {
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");
        Servo rgbServo = hardwareMap.get(Servo.class, "rgb2");

        flywheelMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        gate.setPosition(gateClosed);
        stateLight = new RGB(rgbServo);
    }

    private void updateFlywheelController() {
        double currentVelocity = Math.abs(flywheelMotorLeft.getVelocity());
        double error = targetVelocity - currentVelocity;

        double output = kS * Math.signum(targetVelocity)
                + kV * targetVelocity
                + kP * error;

        output = Range.clip(output, 0.0, 1.0);

        flywheelMotorLeft.setPower(output);
        flywheelMotorRight.setPower(output);
    }


    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void requestSpinUp(double velocity) {
        targetVelocity = velocity;
        state = ShooterState.SPINNING_UP;
    }
    public void requestStop() {
        state = ShooterState.IDLE;
    }
    public void requestFeed() {
        if (state == ShooterState.READY) {
            gate.setPosition(gateOpen);
            feedTimer.reset();
            state = ShooterState.FEEDING;
        }
    }
    public void update() {
        switch (state) {
            case IDLE:
                flywheelMotorLeft.setPower(0);
                flywheelMotorRight.setPower(0);
                gate.setPosition(gateClosed);
                stateLight.blue();
                break;

            case SPINNING_UP:
                updateFlywheelController();
                gate.setPosition(gateClosed);
                stateLight.azure();

                if (atSpeed()) {
                    state = ShooterState.READY;
                }
                break;

            case READY:
                updateFlywheelController();
                gate.setPosition(gateClosed);
                stateLight.green();

                // if speed drops, go back to spinning up
                if (!atSpeed()) {
                    state = ShooterState.SPINNING_UP;
                }
                break;

            case FEEDING:
                updateFlywheelController();
                gate.setPosition(gateOpen);
                stateLight.orange();

                if (feedTimer.seconds() >= feedTime) {
                    gate.setPosition(gateClosed);

                    if (atSpeed()) {
                        state = ShooterState.READY;
                    } else {
                        state = ShooterState.SPINNING_UP;
                    }
                }
                break;
        }
    }

    public boolean shouldRunTransfer() {
        return state == ShooterState.FEEDING && feedTimer.seconds() >= gateOpenDelay;
    }

    public boolean atSpeed() {
        return Math.abs(targetVelocity - getLeftVelocity()) <= velocityTolerance;
    }

    public double getLeftVelocity() {
        return Math.abs(flywheelMotorLeft.getVelocity());
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public ShooterState getState() {
        return state;
    }

    public double getFlywheelSpeed(double distance) {
        return 7.07817 * distance + 928.14484;

    }

    public double getHoodAngle(double distance) {
        return 0.000000505355  * distance * distance * distance
                - 0.000154644  * distance * distance
                + 0.0161674  * distance
                - 0.127576;
    }

    public void aimForDistance(double distance) {

        double velocity = getFlywheelSpeed(distance);
        double hoodPos = getHoodAngle(distance);

        // clamp between 1200 and 2000
        velocity = Math.max(1200, Math.min(2000, velocity));
        // clamp between 0.33 and 0.47
        hoodPos = Math.max(0.33, Math.min(0.47, hoodPos));

        setTargetVelocity(velocity);
        hoodServo.setPosition(hoodPos);
    }
}