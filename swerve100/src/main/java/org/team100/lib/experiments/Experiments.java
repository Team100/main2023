package org.team100.lib.experiments;

import java.util.EnumSet;
import java.util.Map;
import java.util.Set;

import org.team100.lib.config.Identity;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;

public class Experiments {
    /** These experiments are enabled on every robot type. */
    private final Set<Experiment> globalExperiments = Set.of(
            Experiment.UseSetpointGenerator);
    /** These experiments are enabled on specific robot types. */
    private final Map<Identity, Set<Experiment>> experimentsByIdentity = Map.of(
            Identity.COMP_BOT, Set.of(Experiment.UseSetpointGenerator));

    /** Computed for the actual identity used. */
    private final Set<Experiment> m_experiments;

    public Experiments(Identity identity) {
        m_experiments = EnumSet.copyOf(globalExperiments);
        m_experiments.addAll(experimentsByIdentity.getOrDefault(identity, EnumSet.noneOf(Experiment.class)));
        enabledExperiments.set(m_experiments.stream().map(Experiment::name).toArray(String[]::new));
    }

    public boolean enabled(Experiment experiment) {
        return m_experiments.contains(experiment);
    }

    //////////////////////////////////////////////////

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("experiments");
    private final StringArrayPublisher enabledExperiments = table.getStringArrayTopic("enabled").publish();

}
