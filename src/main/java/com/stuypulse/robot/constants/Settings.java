/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Intake { //TODO: !! Make suppliers and Smart numbers so that it can be edited through glass
        static SmartNumber INTAKE = new SmartNumber("INTAKE/ SETTINGS ROLLER/ INTAKE (CHANGEABLE)", 1);
        static SmartNumber STOW = new SmartNumber("INTAKE/ SETTINGS ROLLER/ STOW (CHANGEABLE)", 0);
        static SmartNumber OUTTAKE = new SmartNumber("INTAKE/ SETTINGS ROLLER/ OUTTAKE (CHANGEABLE)", -1); //TODO: remove if we aren't doing outtake

        static SmartNumber UP = new SmartNumber("INTAKE/ SETTINGS POSITIONAL/ UP (CHANGEABLE)", 1); //TODO: update values
        static SmartNumber DOWN = new SmartNumber("INTAKE/ SETTINGS POSITIONAL/ DOWN (CHANGEABLE)", -1);  //TODO: update values
        static SmartNumber STOP = new SmartNumber("INTAKE/ SETTINGS POSITIONAL/ STOP (CHANGEABLE)", 0); 
    }
}
