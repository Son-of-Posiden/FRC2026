// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package SysIdSequence;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SysIdSequence extends SequentialCommandGroup {
  /** Creates a new SysIdSequence. */
  public final SysIdRoutine sysIdRoutine;
  public SysIdSequence(
    String mechanismName, 
    Object voltageMethodConsumer, 
    RelativeEncoder encoder,
    DoubleSupplier motorOutput, 
    double voltageRampRatePerSecond, 
    double voltageStepRate, 
    double quasistaticTimeout,
    double dynamicTimeout,
    Subsystem subsystem) {
    // This needs to be tested
    sysIdRoutine =
            new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                  VoltsPerMeterPerSecond.of(voltageRampRatePerSecond),
                  Volts.of(voltageStepRate),
                  Time.ofRelativeUnits(quasistaticTimeout + dynamicTimeout, Seconds)
                ),
                new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (voltageMethodConsumer),
                    // Tell SysId how to record a frame of data for each motor on the mechanism being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor(mechanismName)
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    motorOutput.getAsDouble() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(encoder.getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                    // WPILog with this subsystem's name ("shooter")
                    subsystem));
    
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasistaticTimeout),
      new WaitCommand(3),
      sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse.withTimeout(quasistaticTimeout)),
      new WaitCommand(3),
      sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout),
      new WaitCommand(3),
      sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout)
    );
  }
}

