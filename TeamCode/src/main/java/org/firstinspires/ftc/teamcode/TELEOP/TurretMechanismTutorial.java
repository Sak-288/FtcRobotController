package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanismTutorial {
    private DcMotorEx turretMotor;

    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.8;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hwMap){
        turretMotor = hwMap.get(DcMotorEx.class, "RM");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newKP){
        kP = newKP;
    }
    public void setkD(double newKD){
        kD = newKD;
    }
    public double getkP(){
        return kP;
    }
    public double getkD(){
        return kD;
    }
    public void resetTimer(){
        timer.reset();
    }
    public void update(AprilTagDetection curID){
        double deltaTime = timer.seconds();
        resetTimer();

        if (curID == null){
            turretMotor.setPower(0);
            lastError = 0;
            return;
        }

        double error = goalX - curID.ftcPose.bearing;
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0){
            dTerm = ((error - lastError)/deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance){
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turretMotor.setPower(power);
        lastError = error;
    }
}
