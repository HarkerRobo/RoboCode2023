package frc.robot.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import harkerrobolib.util.Constants;

public class MotorVelocitySystem {
  private LinearSystem<N1, N1, N1> plant;
  private KalmanFilter<N1, N1, N1> observer;
  private LinearQuadraticRegulator<N1, N1, N1> controller;
  private LinearSystemLoop<N1, N1, N1> loop;

  private static final Vector<N1> MODEL_STD_DEV = VecBuilder.fill(0.1); // mps
  private static final Vector<N1> ENCODER_STD_DEV = VecBuilder.fill(0.3); // mps

  private static final Vector<N1> RELMS = VecBuilder.fill(Constants.MAX_VOLTAGE);
  private double kS;

  public MotorVelocitySystem(double kS, double kV, double kA, double error) {
    this.kS = kS;
    plant = LinearSystemId.identifyVelocitySystem(kV, kA);
    observer =
        new KalmanFilter<>(
            Nat.N1(), Nat.N1(), plant, MODEL_STD_DEV, ENCODER_STD_DEV, Constants.ROBOT_LOOP);
    controller =
        new LinearQuadraticRegulator<>(plant, VecBuilder.fill(error), RELMS, Constants.ROBOT_LOOP);
    loop =
        new LinearSystemLoop<>(
            plant, controller, observer, Constants.MAX_VOLTAGE, Constants.ROBOT_LOOP);
  }

  public double getVoltage(double desiredSpeed, double currSpeed) {
    loop.setNextR(VecBuilder.fill(desiredSpeed));

    // Correct our Kalman filter's state vector estimate with encoder data.
    loop.correct(VecBuilder.fill(currSpeed));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    loop.predict(Constants.ROBOT_LOOP);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = loop.getU(0) + kS * Math.signum(desiredSpeed - currSpeed);
    return nextVoltage;
  }
}
