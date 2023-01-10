// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SparkMaxVoltage;

public class Module extends SubsystemBase {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;
  private final RelativeEncoder tEncoder;
  private final PIDController tPidController;
  private final CANCoder tCanCoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;


  /***
   * 
   * @param driveMotorId Drive motor ID
   * @param turnMotorId Turn motor ID
   * @param driveMotorReversed Is drive motor reversed?
   * @param turnMotorReversed Is turn motor reversed?
   * @param absoluteEncoderID Absolute encoder ID
   * @param absoluteEncoderOffsetRad Absolute encoder offset in Rad
   * @param absoluteEncoderReversed Is absolute encoder reversed?
   */
  public Module(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
   int absoluteEncoderID, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed){

    this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    tCanCoder = new CANCoder(absoluteEncoderID);

    driveMotor = new TalonFX(driveMotorId); // 2048 unit / rev
    turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turnMotorReversed);
   

    turnMotor.setSmartCurrentLimit(SparkMaxVoltage.TurningMotorVoltage);
    turnMotor.burnFlash();

    tEncoder = turnMotor.getEncoder();
    tEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
    tEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);

    tPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    tPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
   }

   public double getDrivePosition(){
      return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter;
   }
   public double getTurnignPosition(){
      return tEncoder.getPosition();
   }
   public double getDriveVelocity(){
      return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
   }
   public double getTurningVelocity(){
      return tEncoder.getVelocity();
   }
   public double getAbsoluteEncoderRad(){
      return (Math.toRadians(tCanCoder.getAbsolutePosition()) - absoluteEncoderOffsetRad)  * (absoluteEncoderReversed ? -1 : 1); 
   }

   public void resetEncoders(){
      driveMotor.setSelectedSensorPosition(0);
    tEncoder.setPosition(getAbsoluteEncoderRad());
   }

   public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnignPosition()));
   }
   public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond) < 0.001 ){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
    turnMotor.set(tPidController.calculate(getTurnignPosition(), state.angle.getRadians()));
  
  }

  public double getDriveVoltage(){
   return driveMotor.getMotorOutputPercent();


  }

   public void stop(){
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(0);
   }
}
