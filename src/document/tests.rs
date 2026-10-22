use super::sample::{self, placeholder_project};
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

#[test]
fn euler_near_keeps_the_rotation_and_follows_the_previous_angles() {
    use glam::{EulerRot, Quat};
    let orders = [EulerRot::XYZ, EulerRot::XZY, EulerRot::YXZ, EulerRot::YZX, EulerRot::ZXY, EulerRot::ZYX];
    let rebuild = |order, e: [f32; 3]| Quat::from_euler(order, e[0].to_radians(), e[1].to_radians(), e[2].to_radians());
    for order in orders {
        for (i, near) in [[0.0, 0.0, 0.0], [170.0, 100.0, -170.0], [720.0, -30.0, 15.0]].into_iter().enumerate() {
            let q = Quat::from_euler(EulerRot::ZXY, 0.3 + i as f32, -1.2, 2.9).normalize();
            let e = euler_near(q, order, near);
            assert!(rebuild(order, e).angle_between(q) < 1e-3, "{order:?} doesn't rebuild the rotation");
            // Equivalent angle sets differ by whole turns or the 180° flip; the nearest is chosen.
            for (a, n) in e.iter().zip(near) {
                assert!((a - n).abs() <= 270.0, "{order:?}: {e:?} is far from {near:?}");
            }
        }
    }
    // Turning steadily about X past 180° keeps counting instead of wrapping.
    let mut previous = [0.0; 3];
    for step in 1..=40 {
        let angle = step as f32 * 10.0;
        previous = euler_near(Quat::from_rotation_x(angle.to_radians()), EulerRot::XYZ, previous);
        assert!((previous[0] - angle).abs() < 1e-2 && previous[1].abs() < 1e-2, "{previous:?} at {angle}°");
    }
}

#[test]
fn local_transforms_invert_world_transforms() {
    let project = placeholder_project();
    let skeleton = &project.characters[0].skeleton;
    let pose = &project.evaluate(100.0)[0];
    let local = local_transforms(skeleton, &world_transforms(skeleton, pose));
    for (i, joint) in skeleton.joints.iter().enumerate() {
        let expected = pose_local_rotation(joint.rest_rotation, pose.joints[i]);
        assert!(local[i].0.angle_between(expected) < 1e-4, "{}", joint.name);
        let offset = glam::Vec3::from(joint.offset) + glam::Vec3::new(pose.joints[i][3], pose.joints[i][4], pose.joints[i][5]);
        assert!(local[i].1.distance(offset) < 1e-5, "{}", joint.name);
    }
}

#[test]
fn batch_applies_in_order_and_undoes_in_reverse() {
    let mut doc = Document::new(placeholder_project());
    let original = doc.project.clone();
    let character = doc.project.characters[0].id;
    let track = Track { id: 900, name: "New".into(), character, muted: false };
    let (first, props) = first_clip(&doc);
    let clip = Clip {
        id: 901,
        props: ClipProps { track: 900, ..props.clone() },
        kind: ClipKind::Motion,
        channels: vec![],
        source: Default::default(),
    };
    // The clip needs its track, so order matters both ways.
    let batch = Edit::Batch(vec![
        Edit::InsertTrack { index: 2, track },
        Edit::InsertClip { index: 2, clip },
        Edit::ClipProps { clip: first, before: props.clone(), after: ClipProps { enabled: false, ..props } },
        Edit::CharacterLevel { character, before: SimLevel::Animated, after: SimLevel::Simulated },
    ]);
    doc.apply("Several things", batch, false);
    let changed = doc.project.clone();
    assert_eq!(doc.project.tracks[2].id, 900);
    assert_eq!(doc.project.clip(901).unwrap().props.track, 900);
    assert!(!doc.project.clip(first).unwrap().props.enabled);
    assert_eq!(doc.project.characters[0].default_level, SimLevel::Simulated);
    assert_eq!(doc.undo_depth(), 1);

    assert_eq!(doc.undo().as_deref(), Some("Several things"));
    assert_eq!(doc.project, original);
    doc.redo();
    assert_eq!(doc.project, changed);
}

#[test]
fn empty_batches_are_not_recorded() {
    let mut doc = Document::new(placeholder_project());
    doc.apply("Nothing", Edit::Batch(vec![]), false);
    let level = doc.project.characters[0].default_level;
    let character = doc.project.characters[0].id;
    doc.apply("Nothing", Edit::CharacterLevel { character, before: level, after: level }, false);
    assert!(!doc.can_undo());
}

#[test]
fn deleted_track_returns_with_its_clips_in_place() {
    let mut doc = Document::new(placeholder_project());
    let character = doc.project.characters[0].id;
    // Interleave clips of two tracks so the restore order matters.
    let base = doc.project.tracks[0].id;
    let (_, props) = first_clip(&doc);
    doc.insert_clip(ClipProps { name: "Late".into(), start: 300, ..props }, ClipKind::Motion, vec![]);
    let original = doc.project.clone();
    assert!(doc.project.clips_on_track(base).count() >= 2);

    assert!(doc.remove_track(base));
    assert!(doc.project.track(base).is_none());
    assert_eq!(doc.project.clips_on_track(base).count(), 0);
    assert_eq!(doc.project.tracks_of(character).count(), 1);
    doc.undo();
    assert_eq!(doc.project, original);
    doc.redo();
    assert!(doc.project.track(base).is_none());
    assert!(!doc.remove_track(base));
}

#[test]
fn new_tracks_go_on_top_of_their_own_character() {
    let mut project = placeholder_project();
    let skeleton = project.characters[0].skeleton.clone();
    let other = project.add_character("other", CharacterKind::Prop, skeleton);
    let other_track = project.add_track(other, "Other base");
    let mut doc = Document::new(project);
    let character = doc.project.characters[0].id;
    let added = doc.add_track(character, "Layer");
    let order: Vec<TrackId> = doc.project.tracks.iter().map(|t| t.id).collect();
    assert_eq!(order[2], added, "after the character's own tracks");
    assert_eq!(order[3], other_track);
    doc.undo();
    assert!(doc.project.track(added).is_none());
}

/// Everything but the id counter, which undo doesn't (and needn't) roll back.
fn content(p: &Project) -> (&ProjectSettings, &[Character], &[Track], &[Clip], &physics::PhysicsSettings) {
    (&p.settings, &p.characters, &p.tracks, &p.clips, &p.physics)
}

/// World transforms of a character over the project range, from its keys.
fn animated_frames(project: &Project, character: usize) -> Vec<Vec<(glam::Quat, glam::Vec3)>> {
    let range = project.settings.range;
    let skeleton = &project.characters[character].skeleton;
    (range.start..=range.end).map(|f| world_transforms(skeleton, &project.evaluate(f as f32)[character])).collect()
}

fn assert_frames_match(project: &Project, frames: &[Vec<(glam::Quat, glam::Vec3)>], tolerance: f32) {
    let start = project.settings.range.start;
    for (i, (now, expected)) in animated_frames(project, 0).iter().zip(frames).enumerate() {
        for (j, (a, b)) in now.iter().zip(expected).enumerate() {
            let name = &project.characters[0].skeleton.joints[j].name;
            assert!(a.1.distance(b.1) < tolerance, "{name} is {} m off at frame {}", a.1.distance(b.1), start + i as i32);
            assert!(a.0.angle_between(b.0) < 0.01, "{name} turned {} rad at frame {}", a.0.angle_between(b.0), start + i as i32);
        }
    }
}

#[test]
fn baked_keys_reproduce_the_motion_as_one_undo_step() {
    let mut doc = Document::new(placeholder_project());
    let original = doc.project.clone();
    let character = doc.project.characters[0].id;
    let frames = animated_frames(&doc.project, 0);
    let (track, clip) = doc.bake_motion(character, &frames).unwrap();

    let project = &doc.project;
    assert_eq!(project.tracks.last().unwrap().id, track);
    assert_eq!(project.track(track).unwrap().name, "Baked");
    let baked = project.clip(clip).unwrap();
    assert_eq!((baked.props.start, baked.props.end()), (0, 240));
    let joints = project.characters[0].skeleton.joints.len();
    let rotations = baked.channels.iter().filter(|c| c.property.is_rotation()).count();
    assert_eq!(rotations, joints * 3, "every joint gets rotation channels");
    assert!(baked.channels.iter().any(|c| c.joint == "hips" && c.property == Property::PosY));
    assert!(baked.channels.iter().all(|c| c.keys.len() == 241));
    // With the other tracks muted, the baked keys alone still give the same motion.
    let mut alone = project.clone();
    for t in alone.tracks.iter_mut().filter(|t| t.id != track) {
        t.muted = true;
    }
    assert_frames_match(&alone, &frames, 1e-4);

    assert_eq!(doc.undo_depth(), 1);
    doc.undo();
    assert_eq!(content(&doc.project), content(&original));
    // Baking twice names the second track apart.
    doc.redo();
    let (second, _) = doc.bake_motion(character, &frames).unwrap();
    assert_eq!(doc.project.track(second).unwrap().name, "Baked 2");
}

#[test]
fn baking_the_physics_demo_turns_it_into_keys() {
    use crate::physics::Simulation;
    let mut doc = Document::new(sample::physics_demo_project());
    let original = doc.project.clone();
    let mut sim = Simulation::default();
    sim.sync(1);
    while !sim.advance(&doc.project, std::time::Instant::now() + std::time::Duration::from_secs(60)) {}
    let range = doc.project.settings.range;
    let frames: Vec<_> = (range.start..=range.end).map(|f| sim.frame(f).unwrap()[0].as_ref().unwrap().joints.clone()).collect();

    let character = doc.project.characters[0].id;
    doc.bake_motion(character, &frames).unwrap();
    let project = &doc.project;
    assert!(!Simulation::is_needed(project), "the character now plays its keys");
    assert_eq!(project.characters[0].default_level, SimLevel::Animated);
    assert!(project.clips.iter().filter(|c| !c.kind.is_motion()).all(|c| !c.props.enabled));
    // The keys land where the simulation was (it stretches joints by under a millimetre).
    assert_frames_match(project, &frames, 2e-3);

    // Still a valid v2 project file.
    let text = crate::io::to_string(project).unwrap();
    assert_eq!(&crate::io::from_str(&text).unwrap(), project);

    doc.undo();
    assert_eq!(content(&doc.project), content(&original));
    assert!(Simulation::is_needed(&doc.project));
}

fn run_simulation(project: &Project) -> crate::physics::Simulation {
    let mut sim = crate::physics::Simulation::default();
    sim.sync(1);
    while !sim.advance(project, std::time::Instant::now() + std::time::Duration::from_secs(60)) {}
    sim
}

fn body_position(project: &Project, sim: &crate::physics::Simulation, frame: i32, body: &str) -> glam::Vec3 {
    let rig = project.characters[0].physics.as_ref().unwrap();
    sim.frame(frame).unwrap()[0].as_ref().unwrap().bodies[rig.body_index(body).unwrap()].1
}

#[test]
fn event_settings_read_back_what_they_write() {
    use events::*;
    let mut s = std::collections::BTreeMap::new();
    for option in SimLevel::ALL {
        set_level(&mut s, keys::PHYSICS_LEVEL, option);
        assert_eq!(level(&s, keys::PHYSICS_LEVEL), Some(option));
        assert_eq!(SimLevel::from_endorphin(option.endorphin_name()), Some(option));
    }
    set_number(&mut s, keys::STRENGTH, 0.25);
    assert_eq!(s[keys::STRENGTH], "0.25");
    set_vector(&mut s, keys::VELOCITY, glam::Vec3::new(0.4, 0.6, -3.5));
    assert_eq!(s[keys::VELOCITY], "0.4, 0.6, -3.5");
    assert_eq!(vector(&s, keys::VELOCITY), Some(glam::Vec3::new(0.4, 0.6, -3.5)));

    let targets_in = vec![
        BodyRef { character: "hero".into(), body: "Chest".into() },
        BodyRef { character: "villain".into(), body: "Right Hand".into() },
    ];
    set_targets(&mut s, &targets_in);
    assert_eq!(s[keys::TARGETS], "hero.MetaBodies.Chest, villain.MetaBodies.Right Hand");
    assert_eq!(targets(&s), targets_in);
    set_targets(&mut s, &[]);
    assert!(!s.contains_key(keys::TARGETS));

    for lock in LockType::ALL {
        assert_eq!(LockType::from_endorphin(lock.endorphin_name()), Some(lock));
    }
    for (speed, heading, elevation) in [(3.0, 180.0, 10.0), (1.5, -90.0, -20.0), (7.0, 45.0, 60.0)] {
        let (s2, h2, e2) = push_parts(push(speed, heading, elevation));
        assert!((s2 - speed).abs() < 1e-4 && (h2 - heading).abs() < 1e-3 && (e2 - elevation).abs() < 1e-3);
    }
    // Heading 0 pushes along +Z, 90 along +X.
    assert!(push(1.0, 0.0, 0.0).distance(glam::Vec3::Z) < 1e-6);
    assert!(push(1.0, 90.0, 0.0).distance(glam::Vec3::X) < 1e-6);
}

#[test]
fn new_clips_go_on_their_own_track_in_one_undo_step() {
    let mut doc = Document::new(placeholder_project());
    let original = doc.project.clone();
    let character = doc.project.characters[0].id;
    let clip = doc.add_new_clip(events::NewClip::Force, character, None, 30, None).unwrap();
    let track = doc.project.clip(clip).unwrap().props.track;
    assert_eq!(doc.project.track(track).unwrap().name, "Forces");
    assert_eq!(doc.undo_depth(), 1);
    // The next force joins that track instead of making another.
    let second = doc.add_new_clip(events::NewClip::Force, character, None, 60, None).unwrap();
    assert_eq!(doc.project.clip(second).unwrap().props.track, track);
    // An explicit track wins.
    let base = doc.project.tracks[0].id;
    let third = doc.add_new_clip(events::NewClip::Motion, character, Some(base), 10, None).unwrap();
    assert_eq!(doc.project.clip(third).unwrap().props.track, base);
    assert!(doc.project.clip(third).unwrap().kind.is_motion());
    for _ in 0..3 {
        doc.undo();
    }
    assert_eq!(content(&doc.project), content(&original));
}

#[test]
fn source_and_channel_edits_undo() {
    let mut doc = Document::new(sample::physics_demo_project());
    let push = doc.project.clips.iter().find(|c| c.props.name == "Shove").unwrap().clone();
    let mut after = push.source.clone();
    events::set_number(&mut after, events::keys::STRENGTH, 2.0);
    // A drag folds into one entry.
    doc.apply("Edit event settings", Edit::ClipSource { clip: push.id, before: push.source.clone(), after: after.clone() }, true);
    let mut later = after.clone();
    events::set_number(&mut later, events::keys::STRENGTH, 3.0);
    doc.apply("Edit event settings", Edit::ClipSource { clip: push.id, before: after, after: later.clone() }, true);
    doc.end_gesture();
    assert_eq!(doc.undo_depth(), 1);
    assert_eq!(doc.project.clip(push.id).unwrap().source, later);
    doc.undo();
    assert_eq!(doc.project.clip(push.id).unwrap().source, push.source);

    let channels = vec![Channel { joint: "hips".into(), property: Property::RotX, keys: vec![] }];
    doc.apply("Capture pose", Edit::ClipChannels { clip: push.id, before: vec![], after: channels.clone() }, false);
    assert_eq!(doc.project.clip(push.id).unwrap().channels, channels);
    doc.undo();
    assert!(doc.project.clip(push.id).unwrap().channels.is_empty());
}

/// Rebuilding the physics demo's events in the editor gives exactly the same simulation,
/// so the settings use the keys the simulation reads.
#[test]
fn editor_events_simulate_like_imported_ones() {
    let demo = sample::physics_demo_project();
    let mut doc = Document::new(placeholder_project());
    doc.project.settings.range = demo.settings.range;
    doc.project.clips.retain(|c| c.props.name != "Wave");
    let character = doc.project.characters[0].id;
    assert!(!crate::physics::Simulation::is_needed(&doc.project));

    let simulate = doc.add_new_clip(events::NewClip::Simulation, character, None, 45, None).unwrap();
    let push = doc.add_new_clip(events::NewClip::Force, character, None, 45, None).unwrap();
    // Match the demo's shove: same velocity change, into the chest.
    let before = doc.project.clip(push).unwrap().source.clone();
    let mut after = before.clone();
    events::set_vector(&mut after, events::keys::VELOCITY, glam::Vec3::new(0.4, 0.6, -3.5));
    events::set_targets(&mut after, &[events::BodyRef { character: "character01".into(), body: "chest".into() }]);
    doc.apply("Edit event settings", Edit::ClipSource { clip: push, before, after }, false);
    assert_eq!(doc.project.clip(simulate).unwrap().source.get("physicsLevel").map(String::as_str), Some("FullPhysics"));

    let (ours, theirs) = (run_simulation(&doc.project), run_simulation(&demo));
    for frame in [44, 46, 80, 150] {
        for body in ["hips", "chest", "head", "r_wrist"] {
            let (a, b) = (body_position(&doc.project, &ours, frame, body), body_position(&demo, &theirs, frame, body));
            assert!(a.distance(b) < 1e-5, "{body} differs at frame {frame}: {a} vs {b}");
        }
    }
    assert!(body_position(&doc.project, &ours, 150, "chest").y < 0.6, "the push knocks the character down");
}

#[test]
fn transition_brings_the_character_back_to_its_keys() {
    let mut doc = Document::new(sample::physics_demo_project());
    let character = doc.project.characters[0].id;
    let transition = doc.add_new_clip(events::NewClip::Transition, character, None, 90, None).unwrap();
    let end = doc.project.clip(transition).unwrap().props.end();
    let sim = run_simulation(&doc.project);
    let state = |f: i32| sim.frame(f).unwrap()[0].as_ref().unwrap().clone();
    assert_eq!(state(80).level, SimLevel::Simulated);
    assert_eq!(state(end + 5).level, SimLevel::Animated);
    let skeleton = &doc.project.characters[0].skeleton;
    let keys = world_transforms(skeleton, &doc.project.evaluate((end + 5) as f32)[0]);
    for (a, b) in state(end + 5).joints.iter().zip(&keys) {
        assert!(a.1.distance(b.1) < 1e-3, "back on the keys after the transition");
    }
}

#[test]
fn active_pose_holds_the_captured_pose() {
    let project = placeholder_project();
    let character = &project.characters[0];
    let pose = world_transforms(&character.skeleton, &project.evaluate(120.0)[0]);
    let channels = events::pose_channels(character, &pose);
    assert_eq!(channels.len(), character.skeleton.joints.len() * 3);
    assert!(channels.iter().all(|c| c.property.is_rotation() && c.keys.len() == 1));
    // The channels rebuild the captured rotations.
    let mut held = Pose::rest(character.skeleton.joints.len());
    for c in &channels {
        held.joints[character.skeleton.joint_index(&c.joint).unwrap()][c.property.index()] = c.keys[0].value;
    }
    let rebuilt = world_transforms(&character.skeleton, &held);
    for (a, b) in rebuilt.iter().zip(&pose) {
        assert!(a.0.angle_between(b.0) < 1e-3);
    }

    // In the demo, holding a standing pose keeps the chest higher than going limp.
    let mut doc = Document::new(sample::physics_demo_project());
    let limp = run_simulation(&doc.project);
    let id = doc.project.characters[0].id;
    let standing = world_transforms(&doc.project.characters[0].skeleton, &doc.project.evaluate(0.0)[0]);
    let clip = doc.add_new_clip(events::NewClip::ActivePose, id, None, 45, Some(&standing)).unwrap();
    let mut props = doc.project.clip(clip).unwrap().props.clone();
    let before = props.clone();
    props.duration = 105;
    doc.apply("Edit clip", Edit::ClipProps { clip, before, after: props }, false);
    let held = run_simulation(&doc.project);
    let chest = |sim, f| body_position(&doc.project, sim, f, "chest");
    assert_ne!(chest(&held, 100), chest(&limp, 100), "the active pose changes the simulation");
}

#[test]
fn constraint_locks_a_body_in_place() {
    let mut doc = Document::new(sample::physics_demo_project());
    let id = doc.project.characters[0].id;
    let lock = doc.add_new_clip(events::NewClip::Constraint, id, None, 50, None).unwrap();
    let source = &doc.project.clip(lock).unwrap().source;
    assert_eq!(source.get("lockType").map(String::as_str), Some("Lock"));
    assert_eq!(source.get("targets").map(String::as_str), Some("character01.MetaBodies.hips"));
    let mut props = doc.project.clip(lock).unwrap().props.clone();
    let before = props.clone();
    props.duration = 100;
    doc.apply("Edit clip", Edit::ClipProps { clip: lock, before, after: props }, false);

    let free = run_simulation(&sample::physics_demo_project());
    let locked = run_simulation(&doc.project);
    let hips = |sim, f| body_position(&doc.project, sim, f, "hips");
    let drift = hips(&locked, 140).distance(hips(&locked, 51));
    let fall = hips(&free, 140).distance(hips(&free, 51));
    assert!(drift < 0.05 && fall > 0.2, "locked hips moved {drift} m, free hips {fall} m");
}
