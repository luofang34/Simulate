use sim_core::ControlInputs;

use super::{EpisodeConfig, EpisodeEnv, EpisodeObservation, EpisodeStep};
use crate::ScenarioError;

/// Many [`EpisodeEnv`]s of one configuration stepped in lock step, as a
/// vectorised rollout worker drives them.
pub struct BatchScenarioRunner {
    envs: Vec<EpisodeEnv>,
}

impl BatchScenarioRunner {
    /// `count` environments of `config`, none reset yet.
    pub fn new(config: EpisodeConfig, count: usize) -> Result<Self, ScenarioError> {
        let envs = (0..count)
            .map(|_| EpisodeEnv::new(config))
            .collect::<Result<_, _>>()?;
        Ok(Self { envs })
    }

    /// The environments, in batch order.
    #[must_use]
    pub fn envs(&self) -> &[EpisodeEnv] {
        &self.envs
    }

    /// Resets environment `i` with seed `base_seed + i`, wrapping.
    pub fn reset_all(&mut self, base_seed: u64) -> Result<Vec<EpisodeObservation>, ScenarioError> {
        let mut seed = base_seed;
        let mut observations = Vec::with_capacity(self.envs.len());
        for env in &mut self.envs {
            observations.push(env.reset(seed)?);
            seed = seed.wrapping_add(1);
        }
        Ok(observations)
    }

    /// Steps environment `i` on `controls[i]`; there must be exactly one
    /// control set per environment.
    pub fn step_all(
        &mut self,
        controls: &[ControlInputs],
    ) -> Result<Vec<EpisodeStep>, ScenarioError> {
        if controls.len() != self.envs.len() {
            return Err(ScenarioError::Unsupported {
                detail: "a batch step needs one control set per environment",
            });
        }
        self.envs
            .iter_mut()
            .zip(controls)
            .map(|(env, controls)| env.step(*controls))
            .collect()
    }
}
