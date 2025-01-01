// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.MotorSetPoint;

public class Flywheel extends SubsystemBase {
  
  private final CANSparkMax flywheelMotor1 = new CANSparkMax (CanIds.FLYWHEEL1,MotorType.kBrushless);
  private final CANSparkMax flywheelMotor2 = new CANSparkMax (CanIds.FLYWHEEL2,MotorType.kBrushless);
  private final CANSparkMax flywheelMotor3 = new CANSparkMax (CanIds.FLYWHEEL3,MotorType.kBrushless);

  private double motorsetPoint = 0 ;

  private SparkPIDController pid;
  private RelativeEncoder encoder;

  public Flywheel() {

    flywheelMotor1.restoreFactoryDefaults();
    flywheelMotor2.restoreFactoryDefaults();
    flywheelMotor3.restoreFactoryDefaults();

    flywheelMotor1.setIdleMode(IdleMode.kCoast);
    flywheelMotor2 .setIdleMode(IdleMode.kCoast);
   flywheelMotor3.setIdleMode(IdleMode.kCoast);

    flywheelMotor1.setInverted(false);
    flywheelMotor2 .setInverted(false);
   flywheelMotor3.setInverted(false);

    flywheelMotor2 .follow(flywheelMotor1);
    flywheelMotor3.follow(flywheelMotor1);

    pid = flywheelMotor1.getPIDController();
    pid = flywheelMotor2 .getPIDController();
    pid = flywheelMotor3.getPIDController();

    encoder = flywheelMotor1.getEncoder();
    encoder = flywheelMotor2 .getEncoder();
    encoder = flywheelMotor3.getEncoder();

    pid.setP(0.1);
    pid.setI(0);
    pid.setD(0);

    flywheelMotor1.burnFlash();
    flywheelMotor2 .burnFlash();
   flywheelMotor3.burnFlash();
  }

  @Override
  public void periodic() {
  }

  public void fulSpeedShot(){
    motorsetPoint = MotorSetPoint.FLYWHEEL_FUll_SPEED;
    setFlywheelVelocity(MotorSetPoint.FLYWHEEL_FUll_SPEED);
  }

  public void halfSpeedShot(){
    motorsetPoint = MotorSetPoint.FLYWHEEL_HALF_SPEED;
    setFlywheelVelocity(MotorSetPoint.FLYWHEEL_HALF_SPEED);
  }

  public void intakePiece(){
    motorsetPoint = MotorSetPoint.FLYWHEEL_REVERSE;
    setFlywheelVelocity(MotorSetPoint.FLYWHEEL_REVERSE);
  }

  public void stopShooter (){
    setFlywheelVelocity(0);
    flywheelMotor1.stopMotor();
  }

  public void setFlywheelVelocity( double velocity){
    pid.setReference(velocity,ControlType.kVelocity);
  }

  public boolean isFlywheelAtSetPoint(){
    double currentVelocity = encoder.getVelocity();
    boolean isFlywheelAtSetPoint = Math.abs(currentVelocity-motorsetPoint) <= MotorSetPoint.FLYWHEEL_TOLERANCE;
    return isFlywheelAtSetPoint;

  }
}
