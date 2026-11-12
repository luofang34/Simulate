use super::LogWindow;

#[test]
fn a_window_keeps_positions_across_what_it_forgets() {
    let mut window: LogWindow<usize> = (0..10).collect();
    window.release_before(4);
    assert_eq!(window.len(), 10);
    assert_eq!(window.first_kept(), 4);
    assert_eq!(window.kept(), 6);
    assert_eq!(window.get(3), None);
    assert_eq!(window.get(4), Some(&4));
    window.push(10);
    assert_eq!(window.get(10), Some(&10));
    assert_eq!(window.last(), Some(&10));
    window.release_before(2);
    assert_eq!(
        window.first_kept(),
        4,
        "releasing backwards brings nothing back"
    );
    window.release_before(50);
    assert_eq!((window.len(), window.kept(), window.get(10)), (11, 0, None));
    window.push(11);
    assert_eq!(window.get(11), Some(&11));
}
