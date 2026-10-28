package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// redundant import because RGB is in the same package as Intake
// import org.firstinspires.ftc.teamcode.config.subsystems.RGB;

public class Intake {
    public enum IntakeState {
        INTAKE, STOP, REVERSE
    }
    private IntakeState state = IntakeState.STOP;
    public final DcMotorEx intake;

    private final RGB ballLight;

    // Tune current levels
    double ONE_BALL = 1.6;
    double TWO_BALLS = 2.5;
    double THREE_BALLS = 4.1;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotor.Direction.FORWARD);

        Servo rgbServo = hardwareMap.get(Servo.class, "rgb1");
        ballLight = new RGB(rgbServo);
    }

    public double getCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
    public int getBallCount() {
        double current = getCurrent();

        if (current > 5.0) {
            return 3;
        } else if (current > 3.5) {
            return 2;
        } else if (current > 2.0) {
            return 1;
        } else {
            return 0;
        }
    }

    public void updateLight() {
        int balls = getBallCount();

        if (balls == 1) {
            ballLight.orange();
        } else if (balls == 2) {
            ballLight.yellow();
        } else if (balls >= 3) {
            ballLight.green();
        } else {
            ballLight.off();
        }
    }

    public void update() {
        switch (state) {
            case INTAKE:
                intake.setPower(1.0);
                break;
            case STOP:
                intake.setPower(0.0);
                break;
            case REVERSE:
                intake.setPower(-1.0);
                break;
        }
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    public void intake() {
        setIntakeState(IntakeState.INTAKE);
    }

    public void stop() {
        setIntakeState(IntakeState.STOP);
    }

    public void reverse() {
        setIntakeState(IntakeState.REVERSE);
    }

}