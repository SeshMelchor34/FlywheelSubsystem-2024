// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class Conveyor extends SubsystemBase {
  
public CANSparkFlex conveyorMotor1 = new CANSparkFlex (CanIds.CONVEYOR1,MotorType.kBrushless);

  public Conveyor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MoveNoteToShooter (){

  }
}
