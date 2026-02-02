package com.stuypulse.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends Intake {
    private final TalonFX ROLLER_MOTOR_LEADER; 
    private final TalonFX ROLLER_MOTOR_FOLLOWER;
    private final TalonFX ROLLER_POSITION_MOTOR;
    
    public IntakeImpl() {
        ROLLER_MOTOR_LEADER = new TalonFX(Ports.Intake.LEADER_MOTOR); 
        ROLLER_MOTOR_FOLLOWER = new TalonFX(Ports.Intake.FOLLOWER_MOTOR); 
        ROLLER_POSITION_MOTOR = new TalonFX(Ports.Intake.POSITIONAL_MOTOR); 

        Motors.INTAKE.LEADER_MOTOR.configure(ROLLER_MOTOR_LEADER);
        Motors.INTAKE.FOLLOWER_MOTOR.configure(ROLLER_MOTOR_FOLLOWER);
        Motors.INTAKE.POSITIONAL_MOTOR.configure(ROLLER_POSITION_MOTOR);
    }

    @Override 
    public void periodic() {
        ROLLER_MOTOR_LEADER.setControl(new DutyCycleOut(getIntakeState().getDutyCycle()));
        ROLLER_MOTOR_FOLLOWER.setControl(new Follower(Ports.Intake.LEADER_MOTOR, MotorAlignmentValue.Opposed));

        ROLLER_POSITION_MOTOR.setControl(new DutyCycleOut(getIntakePosition().getSpeed())); //TODO: !! change from dutycycle potentially later on
        //TODO: !! update the POSITION MOTOR for MOTION MAGIC type of value

        //TODO: add SmartDashboard stuff
        SmartDashboard.putString("INTAKE/ROLLER ROLLER/ ROLLER STATE", getIntakeState().toString());
        SmartDashboard.putString("INTAKE/ROLLER POSITIONAL/ POSITIONAL STATE", getIntakePosition().toString());

        SmartDashboard.putNumber("INTAKE/ROLLER ROLLER/ LEADER DUTYCYCLE", ROLLER_MOTOR_LEADER.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("INTAKE/ROLLER ROLLER/ FOLLOWER DUTYCYCLE", ROLLER_MOTOR_FOLLOWER.getDutyCycle().getValueAsDouble());

        SmartDashboard.putNumber("INTAKE/ROLLER ROLLER/ LEADER CURRENT", ROLLER_MOTOR_LEADER.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("INTAKE/ROLLER ROLLER/ FOLLOWER CURRENT", ROLLER_MOTOR_FOLLOWER.getSupplyCurrent().getValueAsDouble());

         SmartDashboard.putNumber("INTAKE/ROLLER POSITIONAL/ POSITIONAL DUTYCYCLE", ROLLER_POSITION_MOTOR.getDutyCycle().getValueAsDouble()); //TODO: !! remove later
        SmartDashboard.putNumber("INTAKE/ROLLER POSITIONAL/ POSITIONAL CURRENT", ROLLER_POSITION_MOTOR.getSupplyCurrent().getValueAsDouble());

    }
}
