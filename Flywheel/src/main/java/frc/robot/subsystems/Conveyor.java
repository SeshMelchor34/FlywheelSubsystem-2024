// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.MotorSetPoint;

public class Conveyor extends SubsystemBase {
  
  public CANSparkFlex conveyorMotor1 = new CANSparkFlex (CanIds.CONVEYOR1,MotorType.kBrushless);
  private SparkPIDController pid;


  public Conveyor() {
   
    conveyorMotor1.restoreFactoryDefaults();
    conveyorMotor1.setInverted(false);
    conveyorMotor1.setIdleMode(IdleMode.kCoast);

    pid = conveyorMotor1.getPIDController();

    pid.setP(0.1);
    pid.setI(0);
    pid.setD(0);
    conveyorMotor1.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void  ReverseConveyor (){
    setConveyorVelocity(MotorSetPoint.REVERSE_Conveyor);
  }

  public void RunConveyor(){
    setConveyorVelocity(MotorSetPoint.RUN_CONVEYOR);
  }

  public void StopConveyor (){
    setConveyorVelocity(0);
    conveyorMotor1.stopMotor();
  }

  public void MoveNoteToShooter (){
  }

  public void setConveyorVelocity( double velocity){
    pid.setReference(velocity,ControlType.kVelocity);
  }
}
