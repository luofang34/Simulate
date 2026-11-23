import SwiftUI
import FlightDeckKit

/// Offline Aeronautical Chart, 3D Airspace, and NASR Cycle Manager (`v99n62` + `Flightbag` provenance).
/// Enforces pre-download offline availability and explicit `EXPIRED · INVALID` suppression when stale.
struct PadDataDownloadsSheet: View {
    @Bindable var controller: FlightDeckController
    @Environment(\.dismiss) private var dismiss

    private var store: AeroDataPackStore {
        controller.dataPackStore
    }

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    overallCycleBanner
                    packGroupSection(
                        header: "FAA 28-DAY AIRAC NAVIGATION, PROCEDURES & 3D AIRSPACE",
                        authorities: [.faaNasrAirspace, .faaNasrWaypoints, .faaCifp, .faaDtpp]
                    )
                    packGroupSection(
                        header: "FAA 56-DAY VISUAL CHARTS (US VFR)",
                        authorities: [.faaVfrSectional, .faaVfrTerminal]
                    )
                    packGroupSection(
                        header: "FAA 56-DAY ENROUTE CHARTS (IFR LOW / HIGH)",
                        authorities: [.faaIfrLow, .faaIfrHigh]
                    )
                    expirationClockInspectorSection
                }
                .padding(16)
            }
            .background(PadTheme.background)
            .navigationTitle("Downloads & AIRAC Cycles")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button("Done") { dismiss() }
                        .tint(PadTheme.commanded)
                }
            }
        }
    }

    // MARK: - Overall Cycle Currency & Sync Header

    private var overallCycleBanner: some View {
        let refUtc = store.effectiveReferenceUtc
        let health = store.overallHealthSummary(at: refUtc)
        let isOverridden = (store.referenceUtcOverride != nil)
        let bannerTint = health.hasExpired ? PadTheme.warning : PadTheme.active

        return VStack(alignment: .leading, spacing: 10) {
            HStack(alignment: .center, spacing: 10) {
                Image(systemName: health.hasExpired ? "xmark.octagon.fill" : "checkmark.shield.fill")
                    .font(.title2)
                    .foregroundStyle(bannerTint)

                VStack(alignment: .leading, spacing: 2) {
                    HStack(spacing: 6) {
                        Text(health.label)
                            .font(.system(size: 13, weight: .heavy, design: .monospaced))
                            .foregroundStyle(bannerTint)
                        if isOverridden {
                            Text("CLOCK OVERRIDE")
                                .font(.system(size: 9, weight: .heavy, design: .monospaced))
                                .padding(.horizontal, 6)
                                .padding(.vertical, 2)
                                .background(PadTheme.caution.opacity(0.20), in: Capsule())
                                .foregroundStyle(PadTheme.caution)
                        }
                    }
                    Text("Reference UTC: \(AiracCycleGrid.formatUtcShort(refUtc)) · Grid Cycle \(AiracCycleGrid.airacCode(at: refUtc))")
                        .font(.system(size: 11, weight: .medium, design: .monospaced))
                        .foregroundStyle(PadTheme.label)
                }

                Spacer()

                Button {
                    Task {
                        await store.syncFromManifest()
                    }
                } label: {
                    Label("Check & Sync", systemImage: "arrow.triangle.2.circlepath.circle.fill")
                        .font(.caption.weight(.bold))
                        .foregroundStyle(PadTheme.background)
                }
                .buttonStyle(.borderedProminent)
                .tint(PadTheme.commanded)
                .controlSize(.small)
                .disabled(isSyncBusy)
            }

            Text("All aeronautical charts, 3D NASR airspace volumes, and navigation waypoints operate strictly from SHA-256-verified offline packs. Expired packs without a downloaded replacement cycle are marked INVALID and suppressed from rendering.")
                .font(.caption2)
                .foregroundStyle(PadTheme.label)

            syncStateNoteView
        }
        .padding(14)
        .background(
            bannerTint.opacity(0.10),
            in: RoundedRectangle(cornerRadius: 14, style: .continuous)
        )
        .overlay(
            RoundedRectangle(cornerRadius: 14, style: .continuous)
                .strokeBorder(bannerTint.opacity(0.45), lineWidth: 1)
        )
    }

    @ViewBuilder
    private var syncStateNoteView: some View {
        switch store.syncState {
        case .idle:
            EmptyView()
        case .checkingManifest:
            HStack(spacing: 8) {
                ProgressView().controlSize(.small)
                Text("Polling published Flightbag / AIRAC manifest…")
                    .font(.caption2.monospaced())
                    .foregroundStyle(PadTheme.commanded)
            }
        case .downloading(let authority, let fraction):
            VStack(alignment: .leading, spacing: 4) {
                Text("Downloading & SHA-256 verifying \(authority.title)…")
                    .font(.caption2.monospaced())
                    .foregroundStyle(PadTheme.commanded)
                ProgressView(value: fraction)
                    .tint(PadTheme.commanded)
            }
        case .synced(let summary, let checkedAt):
            Text("Last sync (\(AiracCycleGrid.formatUtcShort(checkedAt))): \(summary)")
                .font(.caption2.monospaced())
                .foregroundStyle(PadTheme.active)
        case .failed(let message):
            Text(message)
                .font(.caption2.monospaced())
                .foregroundStyle(PadTheme.caution)
        }
    }

    private var isSyncBusy: Bool {
        switch store.syncState {
        case .checkingManifest, .downloading:
            return true
        case .idle, .synced, .failed:
            return false
        }
    }

    // MARK: - Authority Pack Rows

    private func packGroupSection(header: String, authorities: [AeroDataAuthority]) -> some View {
        let refUtc = store.effectiveReferenceUtc
        return VStack(alignment: .leading, spacing: 8) {
            Text(header)
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.label)

            ForEach(authorities) { auth in
                packRow(for: auth, at: refUtc)
            }
        }
        .padding(12)
        .cockpitRaisedCard(cornerRadius: 14)
    }

    private func packRow(for authority: AeroDataAuthority, at refUtc: Date) -> some View {
        let validity = store.validity(for: authority, at: refUtc)
        let statusTint = validityColor(validity)

        return HStack(alignment: .center, spacing: 10) {
            Image(systemName: validitySymbol(validity))
                .font(.system(size: 16, weight: .semibold))
                .foregroundStyle(statusTint)
                .frame(width: 24)

            VStack(alignment: .leading, spacing: 3) {
                HStack(spacing: 6) {
                    Text(authority.title)
                        .font(.subheadline.weight(.bold))
                        .foregroundStyle(PadTheme.value)
                    Text(authority.cadenceBadge)
                        .font(.system(size: 9, weight: .bold, design: .monospaced))
                        .padding(.horizontal, 5)
                        .padding(.vertical, 2)
                        .background(PadTheme.control, in: Capsule())
                        .foregroundStyle(PadTheme.label)
                }

                Text(authority.provenanceSubtitle)
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundStyle(PadTheme.label)
                    .fixedSize(horizontal: false, vertical: true)

                Text(validity.detailNote)
                    .font(.system(size: 10.5))
                    .foregroundStyle(validity.isExpiredInvalid ? PadTheme.warning : PadTheme.label)
                    .fixedSize(horizontal: false, vertical: true)
            }

            Spacer(minLength: 8)

            VStack(alignment: .trailing, spacing: 6) {
                Text(validity.badgeText)
                    .font(.system(size: 9.5, weight: .heavy, design: .monospaced))
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background(statusTint.opacity(0.18), in: Capsule())
                    .overlay(Capsule().strokeBorder(statusTint.opacity(0.50), lineWidth: 1))
                    .foregroundStyle(statusTint)

                packActionButtons(for: authority, validity: validity)
            }
        }
        .padding(10)
        .background(PadTheme.background, in: RoundedRectangle(cornerRadius: 10, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 10, style: .continuous)
                .strokeBorder(
                    validity.isExpiredInvalid ? PadTheme.warning.opacity(0.65) : PadTheme.separator,
                    lineWidth: 1
                )
        )
    }

    @ViewBuilder
    private func packActionButtons(for authority: AeroDataAuthority, validity: AeroPackValidity) -> some View {
        HStack(spacing: 6) {
            if store.canRestoreBundledSeed(for: authority) {
                Button("Restore 2611") {
                    store.restoreBundledSeedIfPresent(for: authority)
                }
                .buttonStyle(.bordered)
                .tint(PadTheme.commanded)
                .controlSize(.mini)
                .font(.system(size: 10, weight: .bold))
            }

            switch validity {
            case .valid:
                Button("Offload") {
                    store.removeOrOffloadPack(for: authority)
                }
                .buttonStyle(.bordered)
                .tint(PadTheme.label)
                .controlSize(.mini)
                .font(.system(size: 10, weight: .semibold))
            case .expiredInvalid, .notYetEffective, .notDownloaded:
                Button {
                    Task {
                        await store.syncFromManifest()
                    }
                } label: {
                    Text("Download")
                        .foregroundStyle(validity.isExpiredInvalid ? PadTheme.value : PadTheme.background)
                }
                .buttonStyle(.borderedProminent)
                .tint(validity.isExpiredInvalid ? PadTheme.warning : PadTheme.commanded)
                .controlSize(.mini)
                .disabled(isSyncBusy)
                .font(.system(size: 10, weight: .bold))
            }
        }
    }

    // MARK: - Deterministic Expiration & Cycle Transition Verification Clock

    private var expirationClockInspectorSection: some View {
        let post28d = AiracCycleGrid.parseIso8601Utc("2026-12-03T12:00:00Z")
        let post56d = AiracCycleGrid.parseIso8601Utc("2027-01-02T12:00:00Z")

        return VStack(alignment: .leading, spacing: 8) {
            Text("CYCLE EXPIRATION & INVALIDATION VERIFICATION")
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(PadTheme.label)

            Text("Verify strict ICAO AIRAC (28d) and FAA Chart (56d) expiration invalidation when a successor cycle has not been downloaded:")
                .font(.caption2)
                .foregroundStyle(PadTheme.label)

            HStack(spacing: 8) {
                Button {
                    store.setReferenceUtcOverride(nil)
                } label: {
                    Text("Live UTC Clock")
                        .foregroundStyle(store.referenceUtcOverride == nil ? PadTheme.background : PadTheme.value)
                }
                .buttonStyle(.borderedProminent)
                .tint(store.referenceUtcOverride == nil ? PadTheme.active : PadTheme.control)
                .controlSize(.small)
                .font(.caption.weight(.bold))

                Button {
                    store.setReferenceUtcOverride(post28d)
                } label: {
                    Text("2026-12-03Z (28d AIRAC Expired)")
                        .foregroundStyle(store.referenceUtcOverride == post28d ? PadTheme.background : PadTheme.value)
                }
                .buttonStyle(.borderedProminent)
                .tint(store.referenceUtcOverride == post28d ? PadTheme.caution : PadTheme.control)
                .controlSize(.small)
                .font(.caption.weight(.bold))

                Button {
                    store.setReferenceUtcOverride(post56d)
                } label: {
                    Text("2027-01-02Z (All 2611 Expired)")
                        .foregroundStyle(PadTheme.value)
                }
                .buttonStyle(.borderedProminent)
                .tint(store.referenceUtcOverride == post56d ? PadTheme.warning : PadTheme.control)
                .controlSize(.small)
                .font(.caption.weight(.bold))
            }
        }
        .padding(12)
        .cockpitRaisedCard(cornerRadius: 14)
    }

    private func validityColor(_ validity: AeroPackValidity) -> Color {
        switch validity {
        case .valid: PadTheme.active
        case .expiredInvalid: PadTheme.warning
        case .notYetEffective: PadTheme.caution
        case .notDownloaded: PadTheme.caution
        }
    }

    private func validitySymbol(_ validity: AeroPackValidity) -> String {
        switch validity {
        case .valid: "checkmark.circle.fill"
        case .expiredInvalid: "xmark.octagon.fill"
        case .notYetEffective: "clock.badge.exclamationmark.fill"
        case .notDownloaded: "arrow.down.circle.fill"
        }
    }
}
