package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimImpl extends Sim{
    DCMotorSim motor;
    PIDController pidController;
    ArmFeedforward ffController;

    Mechanism2d canvas;
    MechanismRoot2d rootVector;
    MechanismLigament2d turret;

    //TODO: (IMPORTANT) MIGHT GO HORRIBLY WRONG AND IF SO, RESET STATICS AT START OF PERIODIC??
    static boolean useOther;

    static double pidOutput; 

    static double positiveAngle;
    static double negativeAngle;

    public SimImpl() { 
        //doing Kraken because i remember the 0.001 value from its sim website
        //made random Gear ratio
        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 40), DCMotor.getKrakenX60(1));
        pidController = new PIDController(0.4, 0, 0);
        ffController = new ArmFeedforward(0, 0, 0, 0, 0.02);

        canvas = new Mechanism2d(4, 4);
        rootVector = canvas.getRoot("turret root", 2, 2);
        turret = rootVector.append(new MechanismLigament2d("actual turret representation", 2, 0));

        motor.setAngle(Math.toRadians(0));
    }

    public double getAbsoluteAngularPositionDeg() {
        return Math.abs(Math.toDegrees(motor.getAngularPositionRad()) % 360); 
        //TODO: fix Math.abs causing issue
        //make it so that 0 wraps to 360 as well
    }



    @Override 
    public void periodic() {
        pidOutput = 0;
        positiveAngle = 0;
        negativeAngle = 0;
        useOther = false; //reset value each time
        //TODO: add the actual full turn counters, the negatives side counter or something, and just overall make it work.
        SmartDashboard.putData("turret visualizer", canvas);
        
        double getAAPDeg = getAbsoluteAngularPositionDeg();
        //LOGIC GOES FROM HERE -> right now at find shortest path
        if (getState().getAngle().get() >= getAbsoluteAngularPositionDeg()) {
            positiveAngle = getState().getAngle().get() - getAbsoluteAngularPositionDeg(); //TODO: (IMPORTANT) MIGHT GO HORRIBLY WRONG AND IF SO, RESET STATICS AT START OF PERIODIC??
            if (positiveAngle > 180 || (Math.toDegrees(motor.getAngularPositionRad()) + positiveAngle > 480) /*goes above threshold?*/) { //TODO: update with actual value
                negativeAngle = -1 * (360 - positiveAngle);
                useOther = true;
            }

            if (useOther && (getAAPDeg + negativeAngle < 0)) {
                //change getAAPDeg for PIDController -> apply to getAAPDeg the negative angle
                getAAPDeg = -1000000; //placeholdere for commit just so i remember what i was doing
            }
            else {
                getAAPDeg = getAbsoluteAngularPositionDeg() + negativeAngle;
            }
            //do the same here
            //then input  

            pidOutput = pidController.calculate(getAbsoluteAngularPositionDeg(), (useOther) ? getAAPDeg :  getAbsoluteAngularPositionDeg() + positiveAngle);
            //TODO: put pid stuff in here or make another boolean that shows which if we went through (target is greater or less than)
        }

        else if (getState().getAngle().get() < getAbsoluteAngularPositionDeg()) {
            negativeAngle = getState().getAngle().get() - getAbsoluteAngularPositionDeg();
            if (negativeAngle < -180 || (Math.toDegrees(motor.getAngularPositionRad()) + negativeAngle) < -480  /*goes above threshold?*/) {
                positiveAngle = 360 + negativeAngle;
                useOther = true;
            }
            if (!useOther && true) {
                //change getAAPDeg for PIDController
            }

            pidOutput = pidController.calculate(getAbsoluteAngularPositionDeg(), (useOther) ? getAbsoluteAngularPositionDeg() + positiveAngle : getAAPDeg);
        }    

        double ffOutput = ffController.calculate(pidController.getSetpoint(), 1); //TODO: update the velocity value
        double voltage = pidOutput + ffOutput;
        
        motor.setInputVoltage(voltage * getState().getSpeed());

        turret.setAngle(Math.toDegrees(motor.getAngularPositionRad()));

        motor.update(0.02);



        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Voltage", voltage);
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ ACTUAL Motor Position", Math.toDegrees(motor.getAngularPositionRad()));
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ RELATIVE Motor Position", getAbsoluteAngularPositionDeg());

        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Random Angle", getState().getAngle().get());
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ PID output", pidOutput);
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ FF output", ffOutput);
        SmartDashboard.putString("Turret Simulation/ Sim Values/ State", getState().toString());
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Speed", getState().getSpeed());

        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Positive Angle", positiveAngle);
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Negative Angle", negativeAngle);

        SmartDashboard.putBoolean("Turret Simulation/ Sim Values/ Past limit or greater than 180?", (Math.toDegrees(motor.getAngularPositionRad()) + positiveAngle) > 480);
        SmartDashboard.putBoolean("Turret Simulation/ Sim Values/ Past limit or less than -180?", (Math.toDegrees(motor.getAngularPositionRad()) + negativeAngle) < -480);
    }
}
