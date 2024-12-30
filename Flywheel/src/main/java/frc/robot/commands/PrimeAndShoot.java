// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;


public class PrimeAndShoot extends SequentialCommandGroup {
  
  public PrimeAndShoot(Flywheel flywheel,Conveyor conveyor) {
    
    addCommands(
      new InstantCommand (()->flywheel.fulSpeedShot(),flywheel),
      new WaitUntilCommand(()->flywheel.isFlywheelAtSetPoint()),
      new InstantCommand (()->conveyor.MoveNoteToShooter()),
      new WaitCommand(2),
      new InstantCommand(()-> flywheel.stopShooter())
    );
  }
}
