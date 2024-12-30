// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.MotorSetPoint;

public class Elevator extends SubsystemBase {

  public CANSparkFlex elevatorMotor1 =  new CANSparkFlex (CanIds.ELEVATOR1,MotorType.kBrushless);

  private double setPointPosition = 0;


  private SparkPIDController pid;
  private RelativeEncoder encoder;
  
  public Elevator() {
    elevatorMotor1.restoreFactoryDefaults();
    elevatorMotor1.setInverted(false);
    elevatorMotor1.setIdleMode(IdleMode.kBrake);
    elevatorMotor1.setSmartCurrentLimit(20);

    pid.setP(0);
    pid.setI(0.1);
    pid.setD(0);

    pid = elevatorMotor1.getPIDController();
    encoder = elevatorMotor1.getEncoder();
    encoder.setPosition(0);
  }


  public void setElevatorAtBaseHeight(){
    setPointPosition = MotorSetPoint.ELEVATOR_MIN_HEIGHT;
     setElevatorPotition(setPointPosition);
  }

  public void setElevatorAtMaxHeight (){
    setPointPosition = MotorSetPoint.ELEVATOR_MAX_HEIGHT;
    setElevatorPotition(MotorSetPoint.ELEVATOR_MAX_HEIGHT);
  }

  public void setElevatorPotition( double position){
    pid.setReference(position,ControlType.kPosition);
  }

  public boolean isElevatorAtSetPoint(){

    double encoderPosition = encoder.getPosition();
    boolean isElevatorAtSetPoint = Math.abs(encoderPosition-setPointPosition) <= MotorSetPoint. ELEVATOR_TOLERANCE;
    return isElevatorAtSetPoint;

  }


  @Override
  public void periodic() {
  }
}
