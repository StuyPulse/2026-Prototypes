package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;

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
    //stuff neccessary for making sim work:
    DCMotorSim motor;
    PIDController pidController;
    ArmFeedforward ffController;

    Mechanism2d canvas;
    MechanismRoot2d rootVector;
    MechanismLigament2d turret;

    //stuff for math:
    static boolean useNegativeMeasurement;

    static double positiveAngle;
    static double negativeAngle;

    static double positiveMeasurement;
    static double negativeMeasurement;

    

    public SimImpl() { 
        //values based on Kraken and radnom gear ratio
        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 40), DCMotor.getKrakenX60(1));
        pidController = new PIDController(0.2, 0, 0);
        ffController = new ArmFeedforward(0, 0, 0, 0, 0.02);

        canvas = new Mechanism2d(4, 4);
        rootVector = canvas.getRoot("turret root", 2, 2);
        turret = rootVector.append(new MechanismLigament2d("actual turret representation", 2, 0));

        //starting angle
        motor.setAngle(Math.toRadians(0));
    }

    public double getDegrees() { //relative degrees
        return (Math.toDegrees(motor.getAngularPositionRad()) % 360);
    }

    public double getRelativeDegrees() { //WRAPPED degrees
        return (getDegrees() < 0) ? 360 + getDegrees() : getDegrees();
    }

    @Override 
    public void periodic() {
        //resetting static variables:
        positiveAngle = 0;
        negativeAngle = 0;

        positiveMeasurement = 0;
        negativeMeasurement = 0;
        useNegativeMeasurement = false;

        //TODO: add the actual full turn counters, the negatives side counter or something, and just overall make it work.
        SmartDashboard.putData("turret visualizer", canvas);
        
        //This is the math: 
        //SCENARIO ONE: target angle is greater than current angle -> the math was found from a pattern, so give yourself examples and you will find that the math works
        if (getState().getAngle().get() >= getRelativeDegrees())  {
            positiveAngle = getState().getAngle().get() - getRelativeDegrees();
            negativeAngle = getState().getAngle().get() - (getRelativeDegrees() + 360);

            //these are the values I feed into the PIDController because the setpoint is the target angle, so subtraction is already done in PIDController
            positiveMeasurement =  getRelativeDegrees();
            negativeMeasurement = (getRelativeDegrees() + 360);

            //Basically checks if the other angle should be used (either negative angle is a shorter distance, or the positive angle would make it go past the MAX, 
            //but if choosing the negative angle, checks if the angle would make it go below negative limit (not for part where the positive angle would go past the MAX obv, because that would obv not go under negative limit, but mainly for when negative optimal over positive and positive doesn't surpass the MAX))
            if (positiveAngle > Math.abs(negativeAngle) || (Math.toDegrees(motor.getAngularPositionRad()) + positiveAngle) > Settings.Sim.maxAngle) { //get their distances
                if (! ((Math.toDegrees(motor.getAngularPositionRad()) + negativeAngle) < Settings.Sim.minAngle)) {
                    useNegativeMeasurement = true; //Use other refers to negative angle!!!
                }
                
            }
        }
        //SCENARIO TWO: target angle is less than the current angle -> also found from pattern
        else if (getState().getAngle().get() < getRelativeDegrees()) { //
            positiveAngle = getState().getAngle().get() - (getRelativeDegrees() - 360); //TODO: double check java treats subtraction to a negative as addition
            negativeAngle = getState().getAngle().get() - getRelativeDegrees();

            positiveMeasurement =  (getRelativeDegrees() - 360);
            negativeMeasurement = getRelativeDegrees();

        //Same logic as in SCENARIO ONE:
            if (positiveAngle > Math.abs(negativeAngle) || (Math.toDegrees(motor.getAngularPositionRad()) + positiveAngle) > Settings.Sim.maxAngle) { //get their distances
                if (! ((Math.toDegrees(motor.getAngularPositionRad()) + negativeAngle) < Settings.Sim.minAngle)) {
                    useNegativeMeasurement = true; //Use other refers to negative angle!!!
                }
            }
        }

        //There are 2 scenarios because obviously the math/pattern is different but also for when wrapping occurs, the logic should return optimal results.

        double pidOutput = (useNegativeMeasurement) ? pidController.calculate(negativeMeasurement, getState().getAngle().get()) : pidController.calculate(positiveMeasurement, getState().getAngle().get());
        double ffOutput = ffController.calculate(pidController.getSetpoint(), 1); //TODO: update the velocity value
        double voltage =  pidOutput + ffOutput;
        
        motor.setInputVoltage(voltage * getState().getSpeed()); 

        turret.setAngle(Math.toDegrees(motor.getAngularPositionRad()));

        motor.update(0.02);

        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Voltage", voltage);

        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ UNWRAPPED Motor Position", Math.toDegrees(motor.getAngularPositionRad()));
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ WRAPPED Motor Position",getDegrees());
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ RELATIVE Motor Position", getRelativeDegrees() );

        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Random Angle", getState().getAngle().get());
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ PID output", pidOutput);
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ FF output", ffOutput);
        SmartDashboard.putString("Turret Simulation/ Sim Values/ State", getState().toString());
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Speed", getState().getSpeed());

        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Positive Angle", positiveAngle);
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Negative Angle", negativeAngle);

        SmartDashboard.putBoolean("Turret Simulation/ Sim Values/ Past limit positive Side", Math.toDegrees(motor.getAngularPositionRad()) > Settings.Sim.maxAngle);
        SmartDashboard.putBoolean("Turret Simulation/ Sim Values/ Past limit negative Side",  Math.toDegrees(motor.getAngularPositionRad()) < Settings.Sim.minAngle);
    }
}
