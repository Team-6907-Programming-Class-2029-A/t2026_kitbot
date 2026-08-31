// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  /** 让 feeder 和 shooter 停止输出。 */
  public void stop() {
    m_shooter.setControl(m_stopRequest);
    m_feeder.setControl(m_stopRequest);
  }

  /** 按当前模式给 feeder 设定输出(电压或目标转速)。 */
  private void setFeeder(double value) {
    if (Constants.kShooterControlMode == ShooterControlMode.VOLTAGE) {
      m_feeder.setControl(new VoltageOut(value));
    } else {
      m_feeder.setControl(m_feederVelocityRequest.withVelocity(value));
    }
  }

  /** 按当前模式给 shooter 设定输出(电压或目标转速)。 */
  private void setShooter(double value) {
    if (Constants.kShooterControlMode == ShooterControlMode.VOLTAGE) {
      m_shooter.setControl(new VoltageOut(value));
    } else {
      m_shooter.setControl(m_shooterVelocityRequest.withVelocity(value));
    }
  }

  /** 判断 shooter 是否已经升到目标转速(允许一定误差)。 */
  private boolean atShooterSpeed(double targetRps) {
    return Math.abs(m_shooter.getVelocity().getValueAsDouble())
        >= Math.abs(targetRps) - Constants.kLaunchShooterVelocityToleranceRps;
  }

  /** 按住左 bumper 时吸球。 */
  public Command intakeCommand() {
    return run(
        () -> {
          setFeeder(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? Constants.kIntakingFeederVoltage
                  : Constants.kIntakeFeederVelocityRps);
          setShooter(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? Constants.kIntakingShooterVoltage
                  : Constants.kIntakeShooterVelocityRps);
        });
  }

  /** 反向转动 feeder 和 shooter,把球从吸球口排出(速度/电压取吸球的反向)。 */
  public Command ejectCommand() {
    return run(
        () -> {
          setFeeder(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? -Constants.kIntakingFeederVoltage
                  : -Constants.kIntakeFeederVelocityRps);
          setShooter(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? -Constants.kIntakingShooterVoltage
                  : -Constants.kIntakeShooterVelocityRps);
        });
  }

  /**
   * 射球流程(和 AdvantageKit 的 launch 一样先升速再射):
   * 升速阶段 feeder 保持 0 输出,让 shooter 先转起来;
   * 升速完成后 feeder 才正向 feed,把球送进已加速的 shooter 射出。
   * VOLTAGE 模式升速判定用固定时间(AK 做法),VELOCITY 模式用实测转速。
   */
  private Command launchCommand(double feederValue, double shooterValue) {
    // 第一阶段:升速,feeder 保持 0。VOLTAGE 用固定时间,VELOCITY 等 shooter 达到目标转速。
    Command spinUp = run(() -> setShooter(shooterValue));
    if (Constants.kShooterControlMode == ShooterControlMode.VOLTAGE) {
      spinUp = spinUp.withTimeout(Constants.kSpinUpSeconds);
    } else {
      spinUp = spinUp.until(() -> atShooterSpeed(shooterValue));
    }

    // 第二阶段:feeder 开始 feed 射球。
    return spinUp.andThen(
        run(
            () -> {
              setFeeder(feederValue);
              setShooter(shooterValue);
            }));
  }

  /** 按住右 bumper 时高速射球(先升速再 feed)。 */
  public Command launchFastCommand() {
    return launchCommand(
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchFastFeederVoltage
            : Constants.kLaunchFastFeederVelocityRps,
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchFastShooterVoltage
            : Constants.kLaunchFastShooterVelocityRps);
  }

  /** 按住 Y 时低速射球(先升速再 feed)。 */
  public Command launchSlowCommand() {
    return launchCommand(
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchSlowFeederVoltage
            : Constants.kLaunchSlowFeederVelocityRps,
        Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
            ? Constants.kLaunchSlowShooterVoltage
            : Constants.kLaunchSlowShooterVelocityRps);
  }

  /** 按住 B 时只转 feeder(shooter 停转),用于把球送到发射位置。 */
  public Command feedCommand() {
    return run(
        () -> {
          m_shooter.setControl(m_stopRequest);
          setFeeder(
              Constants.kShooterControlMode == ShooterControlMode.VOLTAGE
                  ? Constants.kFeederVoltage
                  : Constants.kFeederVelocityRps);
        });
  }
}
