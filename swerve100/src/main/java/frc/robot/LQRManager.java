// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class LQRManager {

    private LinearSystem<N2, N1, N1> m_plant;
    private KalmanFilter<N2, N1, N1> m_observer;
    private LinearQuadraticRegulator<N2, N1, N1> m_controller;
    public LinearSystemLoop<N2, N1, N1> m_loop;
    public final TrapezoidProfile.Constraints m_constraints;
    
    public LQRManager(LinearSystem<N2, N1, N1> plant,  KalmanFilter<N2, N1, N1> observer, LinearQuadraticRegulator<N2, N1, N1> controller, TrapezoidProfile.Constraints constraints){
        m_plant = plant;
        m_observer = observer;
        m_controller = controller;
        m_constraints = constraints;
        m_loop = new LinearSystemLoop<>(m_plant, m_controller, m_observer, 12.0, 0.020);
    }

    
}
