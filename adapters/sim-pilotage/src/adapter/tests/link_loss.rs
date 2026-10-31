use super::*;

#[test]
fn neutralize_latches_the_scope_until_it_is_cleared() {
    let mut adapter = kinematic();
    let motion = ScopeId::new(MOTION_SCOPE);
    adapter.apply_control(&command(body_rate(0.5, 0.0, 0.0, 1.0)));
    adapter.step(StepBudget { ticks: 120 });

    adapter
        .set_link_loss_policy(VEHICLE, &motion, Some(LinkLossPolicy::Neutralize))
        .unwrap();

    assert_eq!(
        adapter
            .apply_control(&command(body_rate(0.5, 0.0, 0.0, 1.0)))
            .disposition,
        Disposition::Rejected(RejectReason::LinkLossEngaged)
    );
    // The rate loop settles with a one-second time constant.
    adapter.step(StepBudget { ticks: 720 });
    let rates = adapter.vehicle_state().angular_rate_body;
    assert!(rates.length() < 0.01, "{rates}");

    adapter
        .set_link_loss_policy(VEHICLE, &motion, None)
        .unwrap();
    assert_eq!(
        adapter
            .apply_control(&command(body_rate(0.1, 0.0, 0.0, 0.5)))
            .disposition,
        Disposition::Accepted
    );
}

#[test]
fn hold_brief_keeps_the_last_command_then_neutralizes() {
    let mut adapter = kinematic();
    adapter.apply_control(&command(body_rate(0.3, 0.0, 0.0, 0.5)));
    adapter.step(StepBudget { ticks: 120 });
    adapter
        .set_link_loss_policy(
            VEHICLE,
            &ScopeId::new(MOTION_SCOPE),
            Some(LinkLossPolicy::HoldBrief { ticks: 60 }),
        )
        .unwrap();
    adapter.step(StepBudget { ticks: 60 });
    assert!(adapter.vehicle_state().angular_rate_body.x > 0.25);
    adapter.step(StepBudget { ticks: 720 });
    assert!(adapter.vehicle_state().angular_rate_body.x.abs() < 0.01);
}

/// A fenced scope must never stay drivable, even when the policy cannot be
/// enacted as asked.
#[test]
fn an_unsupported_policy_refuses_but_still_latches() {
    let mut adapter = kinematic();
    let motion = ScopeId::new(MOTION_SCOPE);
    assert!(matches!(
        adapter.set_link_loss_policy(VEHICLE, &motion, Some(LinkLossPolicy::Brake)),
        Err(LinkLossEnactError::ChannelRejected { .. })
    ));
    assert_eq!(
        adapter
            .apply_control(&command(body_rate(0.1, 0.0, 0.0, 0.5)))
            .disposition,
        Disposition::Rejected(RejectReason::LinkLossEngaged)
    );
}

/// Latches are per scope: losing the motion link must not fence a reset.
#[test]
fn link_loss_on_one_scope_leaves_the_other_alone() {
    let mut adapter = kinematic();
    adapter
        .set_link_loss_policy(
            VEHICLE,
            &ScopeId::new(MOTION_SCOPE),
            Some(LinkLossPolicy::Neutralize),
        )
        .unwrap();
    let reset = frame(SIM_LIFECYCLE_SCOPE, None, vec![ControlAction::SimReset]);
    assert_eq!(
        adapter.apply_control(&reset).disposition,
        Disposition::Accepted
    );

    let lifecycle = ScopeId::new(SIM_LIFECYCLE_SCOPE);
    adapter
        .set_link_loss_policy(VEHICLE, &lifecycle, Some(LinkLossPolicy::Neutralize))
        .unwrap();
    assert_eq!(
        adapter.apply_control(&reset).disposition,
        Disposition::Rejected(RejectReason::LinkLossEngaged)
    );
    assert!(matches!(
        adapter.set_link_loss_policy(VehicleId::new(9), &lifecycle, None),
        Err(LinkLossEnactError::UnknownVehicle { .. })
    ));
    assert!(
        adapter
            .set_link_loss_policy(VEHICLE, &ScopeId::new("vehicle.gimbal"), None)
            .is_err()
    );
}
