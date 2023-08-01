package org.team100.frc2023;

import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.InversionFeedforward;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.NonlinearSystemLoop;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

// TODO: what is this?
public class LQRManager {
    private NonlinearPlant<N2, N1, N1> m_Plant;
    private ExtrapolatingEstimator<N2, N1, N1> m_Predictor;
    private PointEstimator<N2, N1, N1> m_PointEstimator;
    private LinearPooling<N2> m_Pooling;
    private FeedbackControl<N2, N1, N1> m_Controller;
    public NonlinearSystemLoop<N2, N1, N1> m_Loop;
    private InversionFeedforward<N2, N1, N1> m_Feedforward;

    public LQRManager(NonlinearPlant<N2, N1, N1> plant,
            ExtrapolatingEstimator<N2, N1, N1> predictor,
            PointEstimator<N2, N1, N1> pointEstimator,
            LinearPooling<N2> pooling,
            FeedbackControl<N2, N1, N1> controller,
            InversionFeedforward<N2, N1, N1> feedforward) {
        m_Plant = plant;
        m_Predictor = predictor;
        m_PointEstimator = pointEstimator;
        m_Pooling = pooling;
        m_Controller = controller;
        m_Feedforward = feedforward;
        m_Loop = new NonlinearSystemLoop<>(m_Plant, m_Predictor, m_PointEstimator, m_Pooling, m_Controller, m_Feedforward);
    }
}
