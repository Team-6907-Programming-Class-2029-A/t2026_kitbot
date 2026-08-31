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

  /** 按键触发时的目标电压/速度。速度单位是 rotations per second。 */
  public static final double kFeederVelocityRps = 40.0;
  public static final double kIntakeShooterVelocityRps = 10.0; // TODO: Tune
  public static final double kIntakeFeederVelocityRps = -10.0; // TODO: Tune
  public static final double kShootFastShooterVoltage = 11.0;
  public static final double kShootFastFeederVoltage = 9.0;
  public static final double kShootSlowShooterVoltage = 7.0;
  public static final double kShootSlowFeederVoltage = 6.0;

  /** feeder 的速度闭环参数。kS/kV 是前馈,kP/kI/kD 是 PID 反馈。 */
  public static final double kFeederKs = 0.20;
  public static final double kFeederKv = 0.12;
  public static final double kFeederKp = 0.15;
  public static final double kFeederKi = 0.0;
  public static final double kFeederKd = 0.0;

  /** shooter 的速度闭环参数。 */
  public static final double kShooterKs = 0.20;
  public static final double kShooterKv = 0.5;
  public static final double kShooterKp = 0.7;
  public static final double kShooterKi = 0.0;
  public static final double kShooterKd = 0.0;
}
