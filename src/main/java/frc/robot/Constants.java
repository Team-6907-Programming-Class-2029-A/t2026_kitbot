// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

/** 全局常量集中放在这里,方便修改和查找。 */
public final class Constants {
  private Constants() {}

  /** 手柄在 Driver Station 里的端口号。通常第一个手柄是 0。 */
  public static final int kControllerPort = 0;

  /** CAN 总线上各个电机控制器的设备 ID。 */
  public static final int kLeftMasterCanId = 1;
  public static final int kLeftFollowerCanId = 2;
  public static final int kRightMasterCanId = 3;
  public static final int kRightFollowerCanId = 4;
  public static final int kFeederCanId = 5;
  public static final int kShooterCanId = 6;

  /** feeder 和 shooter 的电机方向。 */
  public static final InvertedValue kFeederInverted = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kShooterInverted = InvertedValue.Clockwise_Positive;

  /** Shooter 的控制模式:VOLTAGE 直接给电压,VELOCITY 用速度闭环。 */
  public enum ShooterControlMode {
    VOLTAGE,
    VELOCITY
  }

  /** 当前使用的控制模式。 */
  public static final ShooterControlMode kShooterControlMode = ShooterControlMode.VOLTAGE;

  //
  // ---- VOLTAGE 模式参数 ----
  //

  /** 吸球/吐球电压(来自 AdvantageKit kitbot_2026 模板)。 */
  public static final double kIntakingFeederVoltage = -8.0;
  public static final double kIntakingShooterVoltage = 6.0;

  /** 射球电压(沿用本仓库之前的数值)。射球流程同样先升速再 feed。 */
  public static final double kLaunchFastShooterVoltage = 11.0;
  public static final double kLaunchFastFeederVoltage = 9.0;
  public static final double kLaunchSlowShooterVoltage = 7.0;
  public static final double kLaunchSlowFeederVoltage = 6.0;

  /** 射球升速时间(shooter 先转起来,feeder 保持 0)。 */
  public static final double kSpinUpSeconds = 1.0;

  /** B 键只转 feeder 时的电压。 */
  public static final double kFeederVoltage = 9.0; // TODO: Tune

  //
  // ---- VELOCITY 模式参数(单位 rotations per second) ----
  //

  /** 按键触发时的目标速度。 */
  public static final double kFeederVelocityRps = 40.0;
  public static final double kIntakeShooterVelocityRps = 20.0; // TODO: Tune
  public static final double kIntakeFeederVelocityRps = -20.0; // TODO: Tune

  /** 射球目标速度。射球流程先让 shooter 升到目标转速,feeder 才开始 feed。 */
  public static final double kLaunchFastShooterVelocityRps = 20.0; // TODO: Tune
  public static final double kLaunchFastFeederVelocityRps = 40.0; // TODO: Tune
  public static final double kLaunchSlowShooterVelocityRps = 14.0; // TODO: Tune
  public static final double kLaunchSlowFeederVelocityRps = 30.0; // TODO: Tune

  /** shooter 判定“已升到转速”的允许误差。 */
  public static final double kLaunchShooterVelocityToleranceRps = 1.0; // TODO: Tune

  /** feeder 的速度闭环参数。kS/kV 是前馈,kP/kI/kD 是 PID 反馈。 */
  public static final double kFeederKs = 0.20;
  public static final double kFeederKv = 0.12;
  public static final double kFeederKp = 0.15;
  public static final double kFeederKi = 0.0;
  public static final double kFeederKd = 0.0;

  /** shooter 的速度闭环参数。 */
  public static final double kShooterKs = 0.20;
  public static final double kShooterKv = 0.5;
  public static final double kShooterKp = 0.1;
  public static final double kShooterKi = 0.0;
  public static final double kShooterKd = 0.1;
}
