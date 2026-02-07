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

    public SimImpl() { 
        //doing Kraken because i remember the 0.001 value from its sim website
        //made random Gear ratio
        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 40), DCMotor.getKrakenX60(1));
        pidController = new PIDController(0.8, 0, 0);
        ffController = new ArmFeedforward(0, 0, 0, 0, 0.02);

        canvas = new Mechanism2d(4, 4);
        rootVector = canvas.getRoot("turret root", 2, 2);
        turret = rootVector.append(new MechanismLigament2d("actual turret representation", 2, 0));

        motor.setAngle(Math.toRadians(0));
    }

    @Override 
    public void periodic() {
        SmartDashboard.putData("turret visualizer", canvas); //TODO: canvas might be the wrong value to send i forgot
        turret.setAngle(Math.toDegrees(motor.getAngularPositionRad())); 
        
        double pidOutput = pidController.calculate(Math.toDegrees(motor.getAngularPositionRad()), getState().getAngle().get()); //TODO: update with actual
        double ffOutput = ffController.calculate(pidController.getSetpoint(), 1); //TODO: update the velocity value
        double voltage = pidOutput + ffOutput;

        motor.setInputVoltage(voltage * getState().getSpeed());
        motor.update(0.02);

        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Voltage", voltage);
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Motor Position", Math.toDegrees(motor.getAngularPositionRad()));
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Random Angle", getState().getAngle().get());
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ PID output", pidOutput);
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ FF output", ffOutput);
        SmartDashboard.putString("Turret Simulation/ Sim Values/ State", getState().toString());
        SmartDashboard.putNumber("Turret Simulation/ Sim Values/ Speed", getState().getSpeed());
    }
}
