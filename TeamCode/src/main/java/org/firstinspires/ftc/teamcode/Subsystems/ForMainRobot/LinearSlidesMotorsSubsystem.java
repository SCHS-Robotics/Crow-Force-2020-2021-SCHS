package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;


import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlidesMotorsSubsystem extends SubSystem {
    CustomizableGamepad input;
    DcMotor rightM;
    DcMotor leftM;
    public LinearSlidesMotorsSubsystem(Robot r, String rightMotor, String leftMotor) {
        super(r);
        rightM = robot.hardwareMap.dcMotor.get(rightMotor);
        leftM = robot.hardwareMap.dcMotor.get(leftMotor);
        usesConfig = true;
    }
    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        input = robot.pullControls(this);
    }

    @Override
    public void handle() {
        if(input.getInput("DownButton")) {
            rightM.setPower(1);
            leftM.setPower(-1);
        }
        else if(input.getInput("UpButton")) {
            rightM.setPower(-1);
            leftM.setPower(1);
        }
        else {
            rightM.setPower(0);
            leftM.setPower(0);
        }
    }

    @Override
    public void stop() {

    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam("UpButton", Button.BooleanInputs.bool_right_trigger,2),
                new ConfigParam("DownButton", Button.BooleanInputs.bool_left_trigger,2)
        };
    }
}
