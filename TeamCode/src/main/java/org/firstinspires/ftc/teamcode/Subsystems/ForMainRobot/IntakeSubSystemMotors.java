
package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.gui.menus.TelemetryMenu;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubSystemMotors extends SubSystem {
    CustomizableGamepad input;
    DcMotor InL;
    DcMotor InR;

    private final String INBUTTON = "InButton", OUTBUTTON = "OutButton";

    TelemetryMenu dMenu = new TelemetryMenu();
    //opencvSkystoneDetector_v2 openCV;

    public IntakeSubSystemMotors(Robot r, String inl, String inr) {
        super(r);
        InL = robot.hardwareMap.dcMotor.get(inl);
        InR = robot.hardwareMap.dcMotor.get(inr);
        input = new CustomizableGamepad(r);
        usesConfig = true;
        robot.gui.addRootMenu(dMenu);
        //openCV = new opencvSkystoneDetectorv2(r);
    }


    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        //input = robot.pullControls(this);
        input.addButton("OutButton", Button.BooleanInputs.left_bumper, 2);
        input.addButton("InButton", Button.BooleanInputs.right_bumper, 2);
    }
    @Override
    public void handle ()  {
        dMenu.addData("InButton", input.getInput(INBUTTON));
        dMenu.addData("OutButton", input.getInput(OUTBUTTON));
        if ((boolean)input.getInput(INBUTTON) && (boolean)input.getInput(OUTBUTTON)) {
            InL.setPower(0);
            InR.setPower(0);
        } else if (input.getInput(INBUTTON)) {
            InL.setPower(-.5);
            InR.setPower(.7);
        } else if (input.getInput(OUTBUTTON)) {
            InL.setPower(.5);
            InR.setPower(-.7);
        } else {
            InL.setPower(0);
            InR.setPower(0);
        }
    }

    @Override
    public void stop ()  {

    }

    @Override
    public void init()  {
        //
    }

    public void intake(double ms) {
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime <= ms) {
            InL.setPower(-.5);
            InR.setPower(.5);
        }
        InL.setPower(0);
        InR.setPower(0);
    }

    public void output(double ms) {
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime <= ms) {
            InL.setPower(.5);
            InR.setPower(-.5);
        }
        InL.setPower(0);
        InR.setPower(0);
    }




    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                //new ConfigParam("InButton", Button.BooleanInputs.right_bumper,2),
                //new ConfigParam("OutButton", Button.BooleanInputs.left_bumper,2)
        };
    }
}

