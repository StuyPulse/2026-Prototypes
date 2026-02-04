/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.Intake.IntakePosition.IntakeSetPositionDown;
import com.stuypulse.robot.commands.Intake.IntakePosition.IntakeSetPositionUp;
import com.stuypulse.robot.commands.Intake.IntakeState.IntakeSetStateIntake;
import com.stuypulse.robot.commands.Intake.IntakeState.IntakeSetStateOutake;
import com.stuypulse.robot.commands.Intake.IntakeState.IntakeSetStateStow;
import com.stuypulse.robot.commands.Intake.SuperStructureIntake;
import com.stuypulse.robot.commands.Intake.SuperStructureStow;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        //Sim values are not accurate but prove that it works
        
        driver.getLeftButton().whileTrue(new IntakeSetStateIntake()).whileFalse(new IntakeSetStateStow()); //KEYBIND: Button 1
        driver.getBottomButton().whileTrue(new IntakeSetStateOutake()).whileFalse(new IntakeSetStateStow()); //KEYBIND: Button 2

        driver.getRightButton().onTrue(new IntakeSetPositionUp());//.whileFalse(new IntakeSetPositionStop()); //KEYBIND: Button 3
        driver.getTopButton().onTrue(new IntakeSetPositionDown());//.whileFalse(new IntakeSetPositionStop()); //KEYBIND: Button 4

        driver.getLeftBumper().whileTrue(new SuperStructureIntake()).whileFalse(new SuperStructureStow()); //KEYBIND: Button 5
        
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
