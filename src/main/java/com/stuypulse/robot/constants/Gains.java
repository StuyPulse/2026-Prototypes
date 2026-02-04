package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

public interface Gains {
    public interface Intake {
        SmartNumber kP = new SmartNumber("INTAKE GAINS/ Gains/ kP", 1);
        SmartNumber kI = new SmartNumber("INTAKE GAINS/ Gains/ kI", 0);
        SmartNumber kD = new SmartNumber("INTAKE GAINS/ Gains/ kD", 0);

        //TODO: update the FF gains 
        SmartNumber kS = new SmartNumber("INTAKE GAINS/ Gains/ kS", 0);
        SmartNumber kG = new SmartNumber("INTAKE GAINS/ Gains/ kG", 0);
        SmartNumber kV = new SmartNumber("INTAKE GAINS/ Gains/ kV", 0);
        SmartNumber kA = new SmartNumber("INTAKE GAINS/ Gains/ kA", 0);
    }
}
