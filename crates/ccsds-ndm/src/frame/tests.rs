use super::*;

#[test]
fn listed_names_map_to_variants_and_back() {
    assert_eq!(RefFrame::from_name("EME2000"), RefFrame::Eme2000);
    assert_eq!(RefFrame::Itrf97.as_str(), "ITRF-97");
    assert_eq!(
        CenterName::from_name("MARS BARYCENTER"),
        CenterName::MarsBarycenter
    );
    assert_eq!(
        MeanElementTheory::from_name("SGP/SGP4").as_str(),
        "SGP/SGP4"
    );
}

#[test]
fn unlisted_names_keep_their_spelling() {
    for name in ["ITRF1997", "itrf2000", "J2000", "ICRF3"] {
        let frame = RefFrame::from_name(name);
        assert_eq!(frame, RefFrame::Other(name.to_owned()));
        assert_eq!(frame.to_string(), name);
    }
    assert_eq!(
        CenterName::from_name("STS 106"),
        CenterName::Other("STS 106".to_owned())
    );
}
