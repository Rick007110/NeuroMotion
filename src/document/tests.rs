use super::sample::placeholder_project;
use super::*;

fn first_clip(doc: &Document) -> (ClipId, ClipProps) {
    let clip = &doc.project.clips[0];
    (clip.id, clip.props.clone())
}

#[test]
fn undo_and_redo_restore_clip_props() {
    let mut doc = Document::new(placeholder_project());
    let (id, before) = first_clip(&doc);
    let mut after = before.clone();
    after.start += 12;
    after.name = "Moved".into();

    doc.apply("Edit clip", Edit::ClipProps { clip: id, before: before.clone(), after: after.clone() }, false);
    assert_eq!(doc.project.clip(id).unwrap().props, after);
    assert!(doc.is_dirty());

    assert_eq!(doc.undo().as_deref(), Some("Edit clip"));
    assert_eq!(doc.project.clip(id).unwrap().props, before);
    assert!(!doc.can_undo());

    assert_eq!(doc.redo().as_deref(), Some("Edit clip"));
    assert_eq!(doc.project.clip(id).unwrap().props, after);
    assert!(!doc.can_redo());
}

#[test]
fn new_edit_clears_redo() {
    let mut doc = Document::new(placeholder_project());
    let (id, before) = first_clip(&doc);
    let mut after = before.clone();
    after.weight = 0.5;
    doc.apply("A", Edit::ClipProps { clip: id, before: before.clone(), after: after.clone() }, false);
    doc.undo();
    assert!(doc.can_redo());
    doc.apply("B", Edit::ClipProps { clip: id, before, after }, false);
    assert!(!doc.can_redo());
}

#[test]
fn continuous_gesture_folds_into_one_entry() {
    let mut doc = Document::new(placeholder_project());
    let (id, original) = first_clip(&doc);
    let mut current = original.clone();
    for _ in 0..5 {
        let mut next = current.clone();
        next.start += 1;
        doc.apply("Move clip", Edit::ClipProps { clip: id, before: current, after: next.clone() }, true);
        current = next;
    }
    doc.end_gesture();
    assert_eq!(doc.undo_depth(), 1);

    // A second gesture on the same clip is its own entry.
    let mut next = current.clone();
    next.start += 1;
    doc.apply("Move clip", Edit::ClipProps { clip: id, before: current, after: next }, true);
    assert_eq!(doc.undo_depth(), 2);

    doc.undo();
    doc.undo();
    assert_eq!(doc.project.clip(id).unwrap().props, original);
}

#[test]
fn noop_edits_are_not_recorded() {
    let mut doc = Document::new(placeholder_project());
    let (id, props) = first_clip(&doc);
    doc.apply("Nothing", Edit::ClipProps { clip: id, before: props.clone(), after: props }, false);
    assert!(!doc.can_undo());
    assert!(!doc.is_dirty());
}

#[test]
fn key_ops_round_trip_through_undo() {
    let mut doc = Document::new(placeholder_project());
    let (id, _) = first_clip(&doc);
    let keys_before = doc.project.clip(id).unwrap().channels[0].keys.clone();

    assert!(doc.insert_key(id, 0, 5));
    assert!(doc.project.clip(id).unwrap().channels[0].key(5).is_some());

    let moved = Keyframe { frame: 7, value: 1.0, interp: Interp::Linear };
    assert!(doc.replace_key(id, 0, 5, moved.clone(), "Move key", false));
    assert_eq!(doc.project.clip(id).unwrap().channels[0].key(7), Some(&moved));

    // Can't land on an occupied frame.
    let blocked = Keyframe { frame: 0, ..moved.clone() };
    assert!(!doc.replace_key(id, 0, 7, blocked, "Move key", false));

    assert!(doc.remove_key(id, 0, 7));
    doc.undo();
    doc.undo();
    doc.undo();
    assert_eq!(doc.project.clip(id).unwrap().channels[0].keys, keys_before);
}

#[test]
fn removed_clip_returns_to_its_slot() {
    let mut doc = Document::new(placeholder_project());
    let ids: Vec<_> = doc.project.clips.iter().map(|c| c.id).collect();
    assert!(doc.remove_clip(ids[0]));
    assert!(doc.project.clip(ids[0]).is_none());
    doc.undo();
    let after: Vec<_> = doc.project.clips.iter().map(|c| c.id).collect();
    assert_eq!(after, ids);
}

#[test]
fn duplicated_clip_follows_original_and_undoes() {
    let mut doc = Document::new(placeholder_project());
    let (id, props) = first_clip(&doc);
    let count = doc.project.clips.len();
    let copy = doc.duplicate_clip(id).unwrap();
    assert_ne!(copy, id);
    assert_eq!(doc.project.clip(copy).unwrap().props.start, props.end());
    assert_eq!(doc.project.clips.len(), count + 1);
    doc.undo();
    assert!(doc.project.clip(copy).is_none());
    doc.redo();
    assert!(doc.project.clip(copy).is_some());
}

#[test]
fn sampling_respects_interpolation() {
    let mut ch = Channel { joint: "hips".into(), property: Property::RotX, keys: vec![] };
    assert_eq!(ch.sample(3.0), None);
    ch.set_key(Keyframe { frame: 0, value: 0.0, interp: Interp::Linear });
    ch.set_key(Keyframe { frame: 10, value: 10.0, interp: Interp::Step });
    assert_eq!(ch.sample(-5.0), Some(0.0));
    assert_eq!(ch.sample(5.0), Some(5.0));
    assert_eq!(ch.sample(20.0), Some(10.0));

    ch.keys[0].interp = Interp::Step;
    assert_eq!(ch.sample(9.0), Some(0.0));

    ch.keys[0].interp = Interp::Smooth;
    let mid = ch.sample(5.0).unwrap();
    assert!((mid - 5.0).abs() < 1e-4, "smooth curve should pass the midpoint, got {mid}");
    assert!(ch.sample(1.0).unwrap() < 1.0, "smooth curve eases out of the first key");
}

#[test]
fn layered_clip_blends_by_influence() {
    let project = placeholder_project();
    let shoulder = project.characters[0].skeleton.joint_index("r_shoulder").unwrap();
    let rz = Property::RotZ.index();
    let wave = project.clips.iter().find(|c| c.props.name == "Wave").unwrap();

    // Before the wave starts only the idle clip drives the arm.
    let before = project.evaluate(10.0)[0].joints[shoulder][rz];
    assert!(before > -20.0);
    // Fully blended in, the layer overrides it.
    let mid = (wave.props.start + wave.props.duration / 2) as f32;
    let full = project.evaluate(mid)[0].joints[shoulder][rz];
    assert!((full + 155.0).abs() < 1e-3);
    // Halfway through the blend-in we're somewhere in between.
    let ramp = (wave.props.start + wave.props.blend_in / 2) as f32;
    let partial = project.evaluate(ramp)[0].joints[shoulder][rz];
    assert!(partial < before && partial > full);
}

#[test]
fn muted_tracks_do_not_contribute() {
    let mut project = placeholder_project();
    let shoulder = project.characters[0].skeleton.joint_index("r_shoulder").unwrap();
    project.tracks[1].muted = true;
    let v = project.evaluate(120.0)[0].joints[shoulder][Property::RotZ.index()];
    assert!(v > -20.0);
}

#[test]
fn event_clips_do_not_change_the_pose() {
    let mut project = placeholder_project();
    let before = project.evaluate(120.0);
    let track = project.tracks[1].id;
    project.add_clip(
        ClipProps {
            name: "Push".into(),
            track,
            start: 100,
            duration: 40,
            blend_in: 0,
            blend_out: 0,
            weight: 1.0,
            enabled: true,
        },
        ClipKind::Event { event: "Force".into() },
        vec![],
    );
    assert_eq!(project.evaluate(120.0), before);
}
