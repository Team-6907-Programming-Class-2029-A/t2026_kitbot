// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterControlMode;

/**
 * 射击子系统:包含 feeder(供球)和 shooter(发射)两组电机。
 * feeder 和 shooter 总是配合动作,所以放在同一个 subsystem 里管理。
 * 控制模式由 {@link Constants#kShooterControlMode} 决定:
 * VOLTAGE 直接给电压,VELOCITY 用速度闭环。
 */
public class Shooter extends SubsystemBase {
  private final TalonFX m_feeder = new TalonFX(Constants.kFeederCanId);
  private final TalonFX m_shooter = new TalonFX(Constants.kShooterCanId);

  // Phoenix 6 控制请求对象:VelocityVoltage 表示速度闭环,NeutralOut 表示停止输出。
  private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_shooterVelocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut m_stopRequest = new NeutralOut();

  // AdvantageKit @AutoLog 自动生成的日志类,用于记录所有 input/output 变量。
  @AutoLog
  public static class ShooterIOInputs {
    public double feederVelocityRps = 0.0;
    public double shooterVelocityRps = 0.0;
    public double feederAppliedVolts = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double shooterCurrentAmps = 0.0;
    public double feederSetpoint = 0.0;
    public double shooterSetpoint = 0.0;
    public boolean atTargetSpeed = false;
    public boolean isIntaking = false;
    public boolean isEjecting = false;
    public boolean isLaunching = false;
    public boolean isFeeding = false;
  }

  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  // launch 状态机字段。launch 命令互斥(同一 subsystem),每次调度时由 startRun 的 initialize 重置。
  private boolean m_hasSpunUp = false;
  private boolean m_initialized = false;
  private double m_startTime = 0.0;

  public Shooter() {
    // 创建 feeder 的配置,包括方向、刹车模式和速度闭环参数。
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.MotorOutput.Inverted = Constants.kFeederInverted;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Slot0.kS = Constants.kFeederKs;
    feederConfig.Slot0.kV = Constants.kFeederKv;
    feederConfig.Slot0.kP = Constants.kFeederKp;
    feederConfig.Slot0.kI = Constants.kFeederKi;
    feederConfig.Slot0.kD = Constants.kFeederKd;

    // shooter 使用 Coast,停止输出后可以自然滑行。
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = Constants.kShooterInverted;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Slot0.kS = Constants.kShooterKs;
    shooterConfig.Slot0.kV = Constants.kShooterKv;
    shooterConfig.Slot0.kP = Constants.kShooterKp;
    shooterConfig.Slot0.kI = Constants.kShooterKi;
    shooterConfig.Slot0.kD = Constants.kShooterKd;

    // 把配置真正写入 TalonFX 控制器。
    m_feeder.getConfigurator().apply(feederConfig);
    m_shooter.getConfigurator().apply(shooterConfig);

    // 默认命令:没有按键按下时让 feeder 和 shooter 停止输出。
    setDefaultCommand(run(this::stop));
  }

  @Override
  public void periodic() {
    // 从电机读取实时数据,写入 inputs 供 AdvantageKit 记录。
    m_inputs.feederVelocityRps = m_feeder.getVelocity().getValueAsDouble();
    m_inputs.shooterVelocityRps = m_shooter.getVelocity().getValueAsDouble();
    m_inputs.feederAppliedVolts = m_feeder.getMotorVoltage().getValueAsDouble();
    m_inputs.shooterAppliedVolts = m_shooter.getMotorVoltage().getValueAsDouble();
    m_inputs.feederCurrentAmps = m_feeder.getSupplyCurrent().getValueAsDouble();
    m_inputs.shooterCurrentAmps = m_shooter.getSupplyCurrent().getValueAsDouble();

    // 将当前周期所有 inputs 提交给 AdvantageKit Logger。
    Logger.processInputs("Shooter", m_inputs);
  }

  /** 让 feeder 和 shooter 停止输出。 */
  public void stop() {
    m_shooter.setControl(m_stopRequest);
    m_feeder.setControl(m_stopRequest);
  }

  /** 按当前模式给 feeder 设定输出(电压或目标转速)。 */
  private void setFeeder(double value) {
    m_inputs.feederSetpoint = value;
    if (Constants.kShooterControlMode == ShooterControlMode.VOLTAGE) {
      m_feeder.setControl(new VoltageOut(value));
    } else {
      m_feeder.setControl(m_feederVelocityRequest.withVelocity(value));
    }
  }

  /** 按当前模式给 shooter 设定输出(电压或目标转速)。 */
  private void setShooter(double value) {
    m_inputs.shooterSetpoint = value;
    if (Constants.kShooterControlMode == ShooterControlMode.VOLTAGE) {
      m_shooter.setControl(new VoltageOut(value));
    } else {
      m_shooter.setControl(m_shooterVelocityRequest.withVelocity(value));
    }
  }

  /** 判断 shooter 是否已经升到目标转速(允许一定误差)。 */
  private boolean atShooterSpeed(double targetRps) {
    boolean atSpeed = Math.abs(m_shooter.getVelocity().getValueAsDouble())
        >= Math.abs(targetRps) - Constants.kLaunchShooterVelocityToleranceRps;
    m_inputs.atTargetSpeed = atSpeed;
    return atSpeed;
  }

  /** 按住左 bumper 时吸球。 */
  public Command intakeCommand() {
    return run(() -> {
          m_inputs.isIntaking = true;
          setFeeder(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? Constants.kIntakingFeederVoltage
                  : Constants.kIntakeFeederVelocityRps);
          setShooter(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? Constants.kIntakingShooterVoltage
                  : Constants.kIntakeShooterVelocityRps);
        })
        .finallyDo(() -> m_inputs.isIntaking = false)
        .withName("ShooterIntake");
  }

  /** 反向转动 feeder 和 shooter,把球从吸球口排出(速度/电压取吸球的反向)。 */
  public Command ejectCommand() {
    return run(() -> {
          m_inputs.isEjecting = true;
          setFeeder(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? -Constants.kIntakingFeederVoltage
                  : -Constants.kIntakeFeederVelocityRps);
          setShooter(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? -Constants.kIntakingShooterVoltage
                  : -Constants.kIntakeShooterVelocityRps);
        })
        .finallyDo(() -> m_inputs.isEjecting = false)
        .withName("ShooterEject");
  }

  /**
   * 射球流程(先升速再射):
   * Phase 1: 只转 shooter,feeder 保持 0,等待升速完成。
   * Phase 2: feeder 开始 feed,把球送进已加速的 shooter 射出。
   * 使用 startRun 单命令状态机,每次调度时 initialize 重置状态,
   * 避免 andThen 调度切换时 defaultCommand 重新调度导致 feeder 被 stop() 覆盖,
   * 也修复了匿名 Runnable 字段跨调度残留(第二次按按键跳过升速)的问题。
   * VOLTAGE 模式升速判定用固定时间,VELOCITY 模式用实测转速。
   */
  private Command launchCommand(double feederValue, double shooterValue) {
    return startRun(
        // initialize: 每次调度时重置状态机,修复跨调度残留(第二次按键直接进 Phase 2 的问题)。
        () -> {
          m_hasSpunUp = false;
          m_initialized = false;
          m_startTime = 0.0;
        },
        // execute: 升速 → feed 状态机。
        () -> {
          m_inputs.isLaunching = true;
          if (!m_hasSpunUp) {
            m_inputs.feederSetpoint = 0.0;
            setShooter(shooterValue);
            if (Constants.kShooterControlMode == ShooterControlMode.VOLTAGE) {
              if (!m_initialized) {
                m_startTime = Timer.getFPGATimestamp();
                m_initialized = true;
              }
              if (Timer.getFPGATimestamp() - m_startTime >= Constants.kSpinUpSeconds) {
                m_hasSpunUp = true;
              }
            } else {
              if (atShooterSpeed(shooterValue)) {
                m_hasSpunUp = true;
              }
            }
          } else {
            setFeeder(feederValue);
            setShooter(shooterValue);
          }
        })
        .finallyDo(
            () -> {
              m_inputs.isLaunching = false;
              m_inputs.atTargetSpeed = false;
            });
  }

  /** 按住右 bumper 时高速射球(先升速再 feed)。 */
  public Command launchFastCommand() {
    return launchCommand(
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchFastFeederVoltage
            : Constants.kLaunchFastFeederVelocityRps,
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchFastShooterVoltage
            : Constants.kLaunchFastShooterVelocityRps)
        .withName("ShooterLaunchFast");
  }

  /** 按住 Y 时低速射球(先升速再 feed)。 */
  public Command launchSlowCommand() {
    return launchCommand(
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchSlowFeederVoltage
            : Constants.kLaunchSlowFeederVelocityRps,
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchSlowShooterVoltage
            : Constants.kLaunchSlowShooterVelocityRps)
        .withName("ShooterLaunchSlow");
  }

  /** 按住 B 时只转 feeder(shooter 停转),用于把球送到发射位置。 */
  public Command feedCommand() {
    return run(() -> {
          m_inputs.isFeeding = true;
          m_shooter.setControl(m_stopRequest);
          setFeeder(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? Constants.kFeederVoltage
                  : Constants.kFeederVelocityRps);
        })
        .finallyDo(() -> m_inputs.isFeeding = false)
        .withName("ShooterFeed");
  }
}