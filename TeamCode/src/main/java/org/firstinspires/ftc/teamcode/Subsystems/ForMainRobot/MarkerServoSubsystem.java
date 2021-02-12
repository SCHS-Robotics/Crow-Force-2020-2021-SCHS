package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;


import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.control.Toggle;
import com.qualcomm.robotcore.hardware.Servo;

public class MarkerServoSubsystem extends SubSystem {
    CustomizableGamepad inputs;
    public Servo MarkerServo;
    private final int DOWN = 1;
    private final double UP = 0.5;
    Toggle toggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    static final String MARKERBUTTON = "MarkerButton";

    public MarkerServoSubsystem(Robot r, String markerServo) {
        super(r);
        MarkerServo = robot.hardwareMap.servo.get(markerServo);
        usesConfig = true;
    }

    @Override
    public void init()  {
        MarkerServo.setPosition(1);
    }

    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        inputs = robot.pullControls(this);
    }

    @Override
    public void handle()  {
        robot.telemetry.addData("MarkerButton", inputs.getInput(MARKERBUTTON));
        robot.telemetry.addData("Toggle State", toggle.getCurrentState());
        robot.telemetry.update();
        toggle.updateToggle(inputs.getInput(MARKERBUTTON));
        if (!toggle.getCurrentState()) {
            MarkerServo.setPosition(1);
        }
        else {
            MarkerServo.setPosition(.5);
        }
    }

    @Override
    public void stop()  {

    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(MARKERBUTTON, Button.BooleanInputs.y, 2)
        };
    }
}
