// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_shooter = new TalonFX(ShooterConstants.shooter_ID, ShooterConstants.CANbus);
  private final TalonFX m_follower = new TalonFX(ShooterConstants.shooter2_ID, ShooterConstants.CANbus);
  private final TalonFX m_funnel = new TalonFX(FunnelConstants.FUNNEL_ID, FunnelConstants.CANbus);
  
  private final TalonFX m_feeder = new TalonFX(ShooterConstants.feeder_ID, ShooterConstants.CANbus);
  Follower toFollowLeader = new Follower(m_shooter.getDeviceID(), MotorAlignmentValue.Opposed);
  

  public double shooterRPSTest = 30;

  public ShooterSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kV = ShooterConstants.kV;
    configs.Slot0.kP = ShooterConstants.kP;
    configs.Slot0.kS = ShooterConstants.kS;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configs.Voltage.PeakForwardVoltage = 11.0;
    configs.Voltage.PeakReverseVoltage = -11.0;
    
    configs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    configs.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    m_shooter.getConfigurator().apply(configs);
    m_feeder.getConfigurator().apply(configs);
    m_follower.setControl(toFollowLeader);

    //FUNNEL CONFIGS
    TalonFXConfiguration funnelConfigs = new TalonFXConfiguration();

    funnelConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    funnelConfigs.CurrentLimits.SupplyCurrentLimit = FunnelConstants.SUPPLY_CURRENT_LIMIT;
    funnelConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    funnelConfigs.CurrentLimits.StatorCurrentLimit = FunnelConstants.STATOR_CURRENT_LIMIT;
    funnelConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    funnelConfigs.Feedback.SensorToMechanismRatio = FunnelConstants.GEAR_RATIO;
    m_funnel.getConfigurator().apply(funnelConfigs);


  }

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command setDutyCycle(double dutycycle) { 
    return run(() -> m_shooter.setControl(new DutyCycleOut(dutycycle)))
      .finallyDo(() -> stop()); 
  }

  public void stopFunnel() {
    m_funnel.setControl(new DutyCycleOut(0));
  }

  public boolean nearSetpoint(double targetRPS) {
    return Math.abs(getVelocity() - targetRPS) < FunnelConstants.START_TOLERANCE_RPS;
  }

  public void stop() {
    m_shooter.setControl(new DutyCycleOut(0));
  }

  public Command setDutyCycleFeeder(double dutycycle) { 
    return run(() -> m_feeder.setControl(new DutyCycleOut(dutycycle)))
      .finallyDo(() -> stopFeeder()); 
  }

  public Command runFunnelManual() {
  return run(() -> m_funnel.setControl(new DutyCycleOut(FunnelConstants.FUNNEL_SPEED)))
    .finallyDo(() -> stopFunnel());
}

  public void stopFeeder() {
    m_feeder.setControl(new DutyCycleOut(0));
  }

  public Command setVelocity(double velocityRPS) {
    return run(() -> m_shooter.setControl(new VelocityVoltage(velocityRPS).withSlot(0)))
      .finallyDo((interrupted) -> stop());
  }

  public double getVelocity() {
    // refresh() is called to get the most up-to-date data from the CAN bus
    return m_shooter.getVelocity().refresh().getValueAsDouble();
  }

  public boolean atSetpoint(double targetRPS) {
    return Math.abs(getVelocity() - targetRPS) < 0.3; 
  }

  //MANUAL SHOOTING
  public Command shootSequence(double feederPercent, double shooterRPS) {
  return run(() -> {
    m_shooter.setControl(new VelocityVoltage(shooterRPS).withSlot(0));

    // Funnel starts once the shooter is almost at speed
    if (nearSetpoint(shooterRPS)) {
      m_funnel.setControl(new DutyCycleOut(FunnelConstants.FUNNEL_SPEED));
    } else {
      m_funnel.setControl(new DutyCycleOut(0));
    }

    // Feeder still waits for the right tolerance
    if (atSetpoint(shooterRPS)) {
      m_feeder.setControl(new DutyCycleOut(feederPercent));
    } else {
      m_feeder.setControl(new DutyCycleOut(0));
    }
  })
  .finallyDo((interrupted) -> {
    stop();
    stopFeeder();
    stopFunnel();
  });
}

  //TREE MAP SHOOTING
  public Command shootSequence(double feederPercent, DoubleSupplier rpsSupplier, BooleanSupplier readyToFeed) {
    return run(() -> {
      double rps = rpsSupplier.getAsDouble();
      m_shooter.setControl(new VelocityVoltage(rps).withSlot(0));

      boolean feed = atSetpoint(rps) && readyToFeed.getAsBoolean();
      m_feeder.setControl(new DutyCycleOut(feed ? feederPercent : 0));
    })
    .finallyDo((interrupted) -> {
      stop();
      stopFeeder();
    });
  }

  public void increaseSpeed(){
    shooterRPSTest+= 0.5;
  }
  public void decreaseSpeed(){
    shooterRPSTest-=0.5;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target RPS", shooterRPSTest);
    SmartDashboard.putNumber("Flywheel RPS", getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
