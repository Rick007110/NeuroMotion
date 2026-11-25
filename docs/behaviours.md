# NeuroMotion behaviours

Behaviours are controllers that run while a character is simulated. Every physics tick
they read its bodies (centre of mass, feet, velocity) and set its muscle targets and
strengths. endorphin has a library of them (stagger, catch fall, writhe, balance, …).
Its definitions are vendor-protected and were not read; everything here is an original
design that only shares the names, so that imported scenes run something sensible.

Code: `src/physics/behaviour.rs`. Tests: `physics::scenes::behaviour_tests` and
`physics::behaviour::tests`.

## Running them

A behaviour runs for the span of an event clip of kind `Behaviour`, matched by its
`behaviourName` setting, which imported endorphin events carry (the clip's own name
when it's missing). Matching ignores case, spaces and punctuation, so `CatchFall`
and `catch fall` both work. `BodyFoetal` also answers to `foetal`, `fetal` and
`bodyfetal`, `BodyWrithe` to Writhe, and `Explosion` to `WritheInMidAir` and
`Writhe in mid-air` (its 2.5 name and the name endorphin shows; plain Writhe stays
Writhe). endorphin's scenes name behaviours after the
set they come from and add a version, as in `Bob2: Balance 2.5`: the part up to the
last `:` and a trailing version number are dropped first. (Until this was added, no
imported scene's behaviour ran at all.) Where several overlap, the one that starts
last wins.
Names with no controller yet are ignored: the
character keeps doing what the timeline otherwise asks.

Look and Reach and Explosion read the settings on their event (below); the others
don't read any yet. The importer keeps a behaviour's parameters on its clip under
endorphin's own names (`Speed`, `Strength`, `LeftHand_Target`, ...; see
[endorphin-formats.md](endorphin-formats.md#behaviour-parameters)), and every behaviour
reads them through the same functions (`behaviour::setting` and `setting_flag`):
Explosion when its span is planned (`Behaviour::with_settings`), Look and Reach once the
world is built, since its targets name bodies in it.

The behaviour starts from the targets and strengths the timeline gives (active pose,
active animation, or the pose held since simulation began) and changes what it needs
to. Controllers keep no state between ticks; everything is read from the bodies, so
simulations stay deterministic. Besides the bodies they are told how many seconds the
behaviour has been running (`Drive::elapsed`), which Writhe uses for its slow
wandering and Look and Reach for its delays; that comes from the frame number, so it
is deterministic too. Look and Reach also reads its event's settings and targets
(`Drive::settings`); the others don't read theirs yet.

## Finding the limbs

Behaviours need to know which bodies are the pelvis, legs and arms. Names differ
between rigs, so `Layout` works from the rig's shape in its rest pose instead:

- the **pelvis** is the root body;
- the **legs** are the two branches from the pelvis that reach lowest; going down each
  one: thigh, shin, foot;
- each foot's **sole** is the lowest points of the collision shapes of the foot and the
  bodies beyond it (toes), those within 2 cm of the lowest, kept in the foot's frame:
  a box's four bottom corners, the bottom of a sphere, the two lower ends of a capsule.
  The ankle's height above the sole is kept too;
- the way the character **faces** is the way its toes point, from the ankles to the
  middle of the soles, so a rig turned any way at rest works (feet centred under the
  ankles don't say, and then it is taken to face +Z with its left at +X);
- the **trunk** is the branch that reaches highest;
- the **arms** are the two branches off the trunk that reach furthest sideways. Upper
  arm and forearm are the longest consecutive pair along each branch, which skips
  collarbones, hands and fingers. A body that can only twist about its own bone
  (swing limits under 0.05 rad, twist axis along the bone), like endorphin's forearm
  twist, counts as part of the segment before it. Without that, endorphin's forearms,
  split in two, made collarbone and upper arm the longest pair, so CatchFall, BodyFoetal
  and Writhe aimed the collarbones of every imported character (stiff joints that
  barely move) instead of the arms (test `finds_the_arms_past_clavicles_and_forearm_twists`).

A rig without two legs doesn't balance, and one without arms doesn't catch its fall.

Curling and writhing also need each joint's *bend axis*, so `Layout` records, for the
rest pose (whatever it is: a standing placeholder or a T-pose):

- the **spine**: the trunk branch's chain upwards, so chest, neck and head included;
- the character's **forward** direction, the horizontal direction from each ankle to
  its foot's centre, averaged (+Z if the feet give none);
- per body, its rest rotation relative to its parent, and the directions of the bend
  axis (up × forward), up and forward in its own frame.

Turning a body about its bend axis by +θ moves up towards forward: it bends a trunk
forwards and bends a hanging shin's knee, while −θ about it flexes a hanging thigh.
A chain is aimed by walking down it: each body's wanted rotation is its rest rotation
relative to its parent, turned about the (wanted) parent's axes. The engine clamps
muscle targets to the joint limits, so asking for more than a rig can bend is safe.
Arms don't use the bend axis (a T-pose arm lies along it): they are aimed by segment
direction, like catch fall, using the chest's current up, forward and outward
directions.

## Balance

Goal: keep the centre of mass over the feet, upright, after a push; step when the feet
can't hold it.

The controller works on the **capture point**: where the centre of mass would come to
rest given its speed, that is position + velocity × √(height / g). Each tick it first
looks at the feet: a foot whose sole is within 2 cm of the lowest sole is **planted**,
and the planted soles make the **support** (their convex hull, seen from above).

### Standing: the ankle strategy

While the capture point is inside the support, or less than 2 cm outside it, Balance
does exactly what it did before it could step:

1. Every muscle is tensed to the rig's `Normal` preset.
2. The wanted shift of the hips is (middle of the feet − centre of mass) − 2 × (velocity
   × √(height / g)), capped at 15 cm. It pulls against where the body *is heading*
   twice as hard as against where it is, which is what stops a push without
   overshooting.
3. **Ankle strategy.** Each leg's shin is aimed by rotating it about the ankle so the
   hip moves by that shift. The ankle muscles then push the body back over its feet.
4. **Pelvis upright.** The hip muscles hold the pelvis level, with its current heading.
   Fore and aft they aim at the thigh as it is now. Aiming at the swung thigh made the
   hip muscles swing the legs too, and their reaction tipped the pelvis forwards.
   Sideways, the legs and pelvis form a parallelogram that would fold with nothing
   resisting it except the narrow feet, so there the hips aim at the swung thigh.
5. Knees, trunk and arms keep the timeline's pose.

### Stepping

When the capture point is further out, one leg steps while the other stands:

1. **Which leg.** If one foot is already in the air, that one. Otherwise, for each leg,
   where its step would go (below) and how far outside the feet the capture point
   would then be. A leg whose step leaves it more than 3 cm out loses; between legs
   that both cover it, the one further from the centre of mass (carrying less
   weight) steps.
2. **Where to.** On one foot, the body tips about the edge of that foot nearest the
   capture point, and the capture point runs away from that edge exponentially, with
   time constant √(height / g). So the step aims where it will be when the foot gets
   there: the edge + (capture point − edge) × e^(t / √(height / g)), where t is 0.1 s
   plus the distance to go at 2 m/s, at most 0.25 s, and 5 cm past that. Looking
   further ahead made long steps the body then swayed back from. The foot never
   crosses the other: it lands at least 0.6 hips' width outside it, and within 0.6
   legs' length of the hip.
3. **The stance leg** keeps the ankle strategy, towards the middle of its own foot, and
   its hip holds the pelvis upright and turned the way that foot points. On one leg
   nothing else holds the pelvis's heading: left as it was, the pelvis spun about the
   stance leg as the other swung, and the swing foot aimed off to the side.
4. **The swing leg** works out hip and knee from the triangle of thigh, shin and
   hip-to-ankle, the knee bending forwards, and keeps the foot level. While the foot
   is still on the ground it is lifted straight up (15% of the leg's length), at the
   rig's `Strong` preset, rather than dragged: the body often still leans on it, and
   a gentle lift only sinks the hips onto it. In the air it is carried at 25% of the
   leg's length towards the target (15% for a step straight forwards) and comes
   down along a slope, 0.9 leg lengths of height per metre it still has to go, so it
   reaches the ground as it reaches the target (*Swing height*, below). Its muscles are
   damped critically for the leg they swing (2√(stiffness × inertia)): the `Normal`
   preset, tuned to hold poses against the ground, is about 3 times overdamped at the
   hip and 12 times at the knee for a leg in the air, which then swung too slowly to
   step.
5. **Landing.** The swinging foot is put down where it is (and pulled in under the hip
   if the leg can't reach the ground from there) as soon as, down there, it would hold
   the capture point, or when its step can't (more than 5 cm out): then the other leg
   gets to step. Chasing a target that keeps moving left feet hovering for half a
   second while the body fell.
6. **Safe on one foot**, the other is put down beside it, hips' width apart.

Nothing is remembered between ticks: the foot in the air is the one stepping, and
where it goes is worked out afresh from the bodies, so simulations stay deterministic
and can restart anywhere (test `stepping_is_deterministic`). It works on any biped
`Layout` finds: the soles, ankle heights, facing and hip widths all come from the rig
(test `balance_steps_with_toes_facing_any_way`, the stand-in with toes, turned).

### Swing height

Balance first carried its swing foot at 10% of the leg's length and dropped it over
the last 6 cm. The foot rarely got that high: about 2 to 5 cm under the sole, as the
hips sank on the stance leg. Within 2 cm of the ground it counts as planted (a foot
in the air is the one stepping, and nothing else says so), so a foot sagging back
down was taken for a planted one and went back to lifting straight up, at the
`Strong` preset, before it could carry on. Steps came late and short, and stalled
most often where a foot had to come out from under the body's weight. Carried at 25%
and brought down along a slope instead, it stays clear until it is nearly there.
Over five pushes each way (1.9 to 2.7 m/s), the ticks Balance spent lifting a foot
back off the ground fell from 314 to 229 to the left, 351 to 164 to the right, 447 to
69 backwards and 143 to 108 forwards.

Straight forwards the foot is carried at 15%. Carried at 25% there too, forward pushes
were held less often: about 15 of the 21 in the sweep below, averaged over 11 nudged
runs, against 19. Why is not pinned down: the trunk doesn't tip back more after the
higher step (measured), and the forward step itself lands as well; what differs is
how the body sways back over the two feet afterwards, which is chaotic.

### Friction

Balance was tuned with NeuroMotion's old friction, a cap of so many newtons per
contact whatever the load. Since the friction fix (lesson 21 in `physics-design.md`)
imported scenes use Coulomb friction, μ = endorphin's friction value / 100, and since
this change NeuroMotion's own projects do too (the placeholder's rubber soles on
tarmac grip at μ = 3, its skin at μ = 1). Planted feet hardly slip with either: over
24 pushes, 1.5 to 3.0 m/s each way, a planted foot slid 1-15 cm in all over the 3 s
after a push with Coulomb friction and 1-9 cm with force caps. What changed was which
pushes it survived, as a chaotic recovery near its limit turns on small differences
(lesson 21's numbers; the rows below). Coulomb friction alone, with the old swing,
lost a few forwards and backwards; the higher swing gained far more than that.

### Results

Pushes on the placeholder's chest from 1.0 to 3.0 m/s, every 0.1 m/s, each way; `#`
means the head stayed above 1.45 m for the next 3 s (`push_sweep`, an ignored test):

| | forwards | backwards | to its left | to its right | in all |
|---|---|---|---|---|---|
| Before stepping | `#######..............` | `####.................` | `##########...........` | `########.............` | 29 |
| Ankle strategy alone, after the friction fix below | `#######..............` | `####.................` | `#########............` | `#######..............` | 27 |
| Stepping, force-capped friction | `########.#.##########` | `########.###.........` | `##########........#..` | `#########............` | 50 |
| The same with Coulomb friction | `########.###...####..` | `#########............` | `##########........#..` | `#########............` | 44 |
| Higher swing, Coulomb friction (now) | `#######.#############` | `#####.##############.` | `##########..#.#######` | `#########.#....#.....` | 68 |

Each row is one run on one machine, and near the limit it is sharper than the truth:
whether a push is survived turns on small differences in where a foot lands, and
another platform's rounding is enough to change it (CI fell at 1.9 m/s to the left on
Linux and at 1.8 to the right on Windows). Averaged over 11 runs with every speed
nudged by up to ±5e-4, the pushes survived of the 21 each way:

| | forwards | backwards | to its left | to its right |
|---|---|---|---|---|
| Stepping, force-capped friction | 17.3 | 10.6 | 11.7 | 8.7 |
| The same with Coulomb friction | 15.5 | 9.8 | 11.4 | 8.8 |
| Higher swing, Coulomb friction (now) | 19.7 | 19.3 | 17.7 | 11.4 |
| Higher swing, force-capped friction | 16.1 | 19.2 | 19.1 | 14.4 |

The last row is what a project saved before the switch gets, as it keeps force-capped
friction (`physics-design.md`): better than before except forwards (16.1 against
17.3). In speeds, Balance now stands after pushes up to 3.0 m/s forwards and to the
left (with gaps between 2.0 and 2.3 to the left in some runs), 2.9 backwards, and up
to about 2.0 to the right. Sideways is still the weakest way, see *Not yet*.

So the test, `balance_steps_when_the_ankles_cannot_hold`, counts, in each set of pushes
past what the ankles hold, how many Balance steps and stays up after: at least 4 of
the 8 backwards from 1.4 to 2.1 m/s, 2 of the 22 sideways (left 1.9-2.8, right
1.7-2.8) and 5 of the 12 forwards from 1.7 to 2.8. Over 101 runs with the speeds
nudged by up to ±1e-3 (`nudge_survey`) the counts are now 6-8, 8-12 and 9-12 (5-8,
3-7 and 6-11 before the change, with force-capped friction; 3-7, 2-6 and 6-11 with
Coulomb friction and the old swing, which fails the backwards count).
`balance_keeps_the_placeholder_upright_after_a_push`, what the ankle strategy held
before stepping, is unchanged.

### Not yet

- **Sideways.** Pushed sideways, the body falls towards the foot on that side, which
  carries the weight: lifting it drops the hips onto it, and the other foot can't
  cross over to catch the body (the legs collide; a crossover target was tried, and
  the foot jammed against the other leg). The higher swing helps (17.7 and 11.4 of 21
  pushes held to the left and right, from 11.7 and 8.7), but to the right it still
  falls from about 2.0 m/s. It needs a weight shift onto the far foot before the near
  one lifts (tried with the ankle strategy alone: far too slow against a fall already
  under way), or a crossover step that goes around the other leg.
- **Other rigs.** Everything above is measured on the placeholder. On endorphin's own
  character (Stagger Walk) stepping doesn't work at all; see *Stagger Walk* under
  Stagger.
- **Which pushes it survives is erratic** past what the ankles hold: 1.8 m/s
  backwards falls where 1.7 and 1.9 don't (in about two runs in three when nudged).
  A recovery near the limit depends on small differences in where a foot lands; the
  settings were chosen on the whole sweep, not on any one push. The tests count sets
  of pushes rather than check single speeds for that reason (table above).
- After a long step the feet stay staggered, a narrow stance across the line between
  them; the body sometimes drifts out sideways from it later. Bringing the back foot
  up beside the front one once settled was tried and made no difference.
- The stance knee stays straight, so a swinging foot reaches the ground only near the
  hip until the body has tipped. Bending it (by a fixed angle, or with the same
  triangle as the swing leg) made more pushes fail, not fewer.
- Steps up or down: the ground is taken to be the lower foot's sole.
- The trunk and arms don't help (no arm swing, no hip strategy).

## Stagger

Goal: endorphin's stagger, a character driven off its feet by a push and stumbling to
keep up with it, rather than a clean recovery.

Stagger is Balance whose support is allowed to move: the same controller with a
different gait.

- It steps as soon as the capture point leaves the planted feet (no 2 cm margin), so
  it gives way instead of standing its ground with the ankles.
- It aims its steps where the capture point will be only half as far ahead as
  Balance does (0.125 s instead of 0.25, 5 cm past it as Balance does), so they fall
  short of where the body is going and it keeps stepping.
- It shuffles: its feet are carried at 10% of the leg's length and dropped over the
  last 6 cm, as Balance's were before its swing was raised. With Balance's higher
  swing and the short steps it recovered too well, and with Balance's look-ahead it
  went no further than Balance after hard pushes once friction became Coulomb (below).
- The trunk, head and arms are at 35% of the rig's `Normal` preset, so the upper body
  sways and lags behind the legs.

Results (`push_sweep` with `NM_SWEEP=Stagger`, same pushes as above):

| | forwards | backwards | to its left | to its right | in all |
|---|---|---|---|---|---|
| Before, force-capped friction | `#######.#....#..#....` | `#####.####.#.........` | `##########...........` | `#########............` | 39 |
| Before, Coulomb friction | `##############.##..#.` | `########.###.........` | `#########.#.#........` | `#######..............` | 46 |
| Now (Coulomb) | `#######......#..#....` | `#########.##.........` | `#########.......#....` | `#########............` | 39 |

39 of 84 stay up, far fewer than with Balance (68), as meant. Pushed hard forwards
(2.7 to 3.0 m/s) it keeps stepping, 3 to 13 steps a push, and goes down, its feet
going 4.2 to 9.5 m over the four pushes, where Balance stops in a few steps and
nearly always stands, its feet going 1.7 to 2.5 m (101 runs with the speeds nudged by
up to ±1e-3, `nudge_survey`). The old Stagger on Coulomb friction stood up after most
of them: its feet went only 0.7 to 2.8 times as far as Balance's, and after one push
it took only 2 steps in some runs. The test, `stagger_keeps_stepping_after_a_hard_push`,
checks those four pushes for at least 2 steps after each, 12 in all, and feet going
more than 1.5 times as far as Balance's; over the 101 runs the fewest steps after one
push was 3, the total 25 to 41 and the ratio 1.9 to 4.5 (3, 17-31 and 2.3-4.2 before,
on force-capped friction). A single push can't be checked on its own: how many steps
it takes turns on small differences in where a foot lands.

### Stagger Walk

Stagger Walk, the sample scene that uses it, pushes endorphin's own character back
at about 0.5 m/s at its first frame. endorphin's takes about five steps (two
backwards by frame 36, then strides), lowers its arms and never falls. NeuroMotion's
doesn't step at all: its feet shuffle within 10 cm of where they stood while the
pelvis tips back (12° by frame 16, 19° by frame 32), the knees fold to 140° and the
body sinks and falls flat by frame 110, with Stagger, with Balance or with no
behaviour. The placeholder's tests didn't show it.

The cause is the engine's spring cap (lesson 4 in `physics-design.md`), not the
controllers or friction: it caps a muscle at 30 Hz for the two bodies it joins, and
endorphin's character has a light pelvis (4.5 kg) between its 9.3 kg thighs and its
trunk, and feet made of three bodies (heel 2.4 kg, foot, toes). In this scene the cap
cut the hips from the `Normal` preset's 3000 N·m/rad to 480-1070, the knees from 5000
to about 1100-1400 and, whenever the heel wasn't the part touching the ground, the
ankles from 1000 to 110-200, under the ~370 an ankle needs to hold a standing body
(lesson 9 counts only the touching body as held). With the cap lifted (a test, not a
fix), the same controller steps back twice, stops the body by frame 60 and stands
until the walk takes over at frame 99, pelvis within 6° of upright.

A fix was tried and not kept: estimating the cap from each side of a joint as a
whole (the child's branch, and the rest), a side resting on the ground counting as
immovable, as lesson 9 does for single bodies. Stagger Walk then steps back twice
(the feet end 0.3 and 0.8 m back), stands with the pelvis within 5° of upright, and
endorphin's sweep stays 16 of 16 clean. But the placeholder's stance muscles get
stronger too, and Balance then holds only 3 of its 8 hard backward pushes (the test
wants 4; 51 of 84 in `push_sweep`): the controllers were tuned on the capped
muscles. That change needs Balance retuned with it, which is the next step.

Imported scenes now count a muscle's whole limb in the cap (`SpringCap::Limbs`, lesson
27 in `physics-design.md`), which is close to that whole-side estimate, without
touching NeuroMotion's own projects. Measured on the combined build
(`dump_scene_motion`): Stagger Walk's character now steps back. Its right foot goes
1.0 m back over frames 5-60 in two low strides (5-9 cm off the ground), its left 0.37
m in shorter ones; it stops by frame 60 and stands, head 1.54-1.74 m up and pelvis
0.86-0.97 m, until the scene's animation takes over at frame 145. endorphin's takes
about five steps; ours takes two or three, low and long, and hasn't been compared
frame by frame.

**Not yet.** Settings on the behaviour event (ArmsWindmill, MaxSteps, ArmsSpeed,
ArmsStrength in Stagger Walk) aren't read, the arms don't windmill, and nothing but
Stagger Walk's first second has been compared with endorphin. It inherits Balance's
limits (above), sideways most of all.

## Catch fall

Goal: get the hands on the ground before the head or chest when shoved over.

Each tick:

1. The fall direction is the centre of mass's horizontal velocity (or, when nearly
   still, the direction it leans from the feet).
2. Both arms are aimed, straight, at the ground 1.5 chest heights out that way (about
   where the head would land), shoulder-width apart. That point is out of reach at
   first, so the arms point forwards and down. As the body pivots over its feet, the
   hands meet the ground braced well ahead of the head.
3. The arms use the rig's `Strong` preset; the trunk `Normal`; the legs a fifth of
   `Normal`, so the knees give and the body comes down onto the hands.

Results on the placeholder (test `catch_fall_lands_on_the_hands_first`): shoved
forwards at 3.5 or 5 m/s, diagonally, sideways or backwards, the hands touch first.
The head and chest follow half a second or more later (with Coulomb friction, 0.5 to
0.9 s; before, with force caps, 0.3 to 0.5 s, or the arms held them off the ground
altogether shoved diagonally). Shoved forwards limp, it lands on its face.

## Body foetal

Goal: curl up and protect the head.

1. The trunk bends forwards 0.9 rad in total, shared evenly by its joints (spine,
   chest, neck, head), so the head bows too.
2. The hips flex −1.6 rad and the knees bend 2.0 rad.
3. The arms go over the head: upper arms point forwards, up and out from the chest,
   forearms fold back towards the head.
4. The trunk and hips use the rig's `Strong` preset (`Normal` if it has none), everything
   else `Normal`.

Results on the placeholder (test `body_foetal_curls_up`): it drops limp, then at 1 s
BodyFoetal runs. Two seconds later, compared with the same scene limp: the shins' centres
are 0.77 m from the chest (limp 1.10 m), the head 1.00 m from them (limp 1.38 m) and the
hands 0.14 m from the head (limp 0.83 m). No NaN, joints stretched at most 1 mm,
fastest body 5.5 m/s (the fall). That is as curled as the placeholder gets: its hips
stop at 1.4 rad of flexion and its spine joints at 0.5, so the 1.6 rad hip target settles
at the limit. Rigs with a wider range curl further. The placeholder falls on its back,
where curling lifts the legs and head against gravity; the tests don't cover the
character on its side or front.

## Writhe

Goal: a weak, endless struggle, as of a character in pain or trying to get up.

The pose is the foetal one at half strength (hips −0.8, knees 1.0, trunk 0.45 rad, arms
halfway between hanging and over the head) with slow sines added to every target: bend,
side bend and twist of each trunk joint (±0.25, 0.3 and 0.2 rad), flexion and sway of the
hips (±0.5, 0.35), the knees (±0.5) and the arms' directions. Each has its own rate,
0.5 to 1.5 rad/s, and phase, driven by the time since the event began. There is no
randomness. The trunk's muscles are 0.6 × `Normal`, all the limbs' 0.3 ×, so the trunk
leads and the limbs flail along.

Results (test `writhe_keeps_moving_and_is_deterministic`): after 3 s, 13 of the
placeholder's 17 bodies still move at over 0.1 m/s (mean 0.24 m/s; 17 and 0.29 with
force-capped friction, as a body lying on the ground grips it in proportion to its
weight now), no NaN, joints stretched at most 1.8 mm, fastest body 5.6 m/s. Two runs
give bit-identical bodies.

## Look and Reach

endorphin's most used behaviour (11 of its scenes, as `Bob2: Look and Reach` and
the demo "Hands reach and look at"): each hand reaches for a target and the head
looks at one, while the rest of the body holds its pose. NeuroMotion runs it for
events named `Look and Reach` or `Hands reach and look at`.

### What endorphin does

Measured from a recording of endorphin 2.5.2 running its demo scene "Hands reach and
look at2.5", with the frame counter, and a camera fitted to the view (3 px rms on 9
points of the first frame), which lets NeuroMotion's run be drawn into the same view.
Two characters stand in a T-pose, each held by a lock on its upper spine, so the trunk
and legs don't move; both reach and look at a 0.5 m box that NeuroMotion knows to be
animated: frames 0-100 it rises 0.94 m, 1.35 m in front of the left character
(character01), from shoulder height; frames 100-200 it slides 2.4 m sideways over
both, 2.5-2.7 m up; frames 200-300 it comes down 1.35 m in front of the right
character (character02), 0.5 m to its left. Settings: hand speed 0.7, strength 0.7,
`MaxExtension` 0.9, look-at speed 1; palms facing (`applyHandOrientation`) on for
character02, off for character01.

- **The arms point at the target, from the shoulders, and the hands come together on
  it.** character01's arms swing in from the T-pose: by frame 6 the far arm has
  dropped and swings in, by 12 its forearm folds across the chest, by frame 18-24
  both arms are straight ahead at the box with the hands together, and from 24 to
  40 the hands move less than 15 px (about 10 cm). So a reach of about 90° takes
  0.3-0.4 s.
- **They don't stretch out fully**: the elbows stay a little bent and out
  (`MaxExtension` 0.9). A box out of reach is pointed at, not chased: nothing leans or
  steps (here the trunk is locked, so that can't be seen).
- **An arm that has to cross the chest is slower.** For character02 (box 62° to its
  right) the near arm is already roughly on the line, but the far one stays out until
  frame 24-30, crosses the chest by 40 and points at the box by about 60.
- **The head turns to the target quickly**: character02's head faces the box by frame
  12-18 at look-at speed 1.
- **They follow it**: with the box overhead (frames 130-180) the hands go up, elbows
  bent, by the face; as it comes down beside character02 (240-300) its arms point
  down at it.

### Design

`LookAndReach` in `behaviour.rs`, stateless like the others. The importer now keeps
a behaviour's settings ([endorphin-formats.md](endorphin-formats.md#behaviour-parameters));
the simulation finds each target, a body (`character.MetaBodies.Body`, its centre) or
a collision surface (its path; the body, or the shape's centre when the body has only
one), in the world once all characters are built, so a character can reach for
another, a prop or the scenery. A setting a scene leaves out takes the value most of
endorphin's scenes give it (speed and strength 0.8, `MaxExtension` 1, look-at speed
0.5, `LegTorsoStiffness` 0.8).

Each tick:

1. **Hold the rest.** The legs and trunk are set to the rig's `Normal` preset ×
   `LegTorsoStiffness` and keep the timeline's pose (active pose, animation or the
   pose held since simulation began). Arms and neck keep the timeline's muscles until
   a reach or the look takes them over.
2. **Where each hand goes**: the target point plus `CatchOffset` (world axes), moved on
   to where a moving target will be when the hand gets there if `UsePrediction` (its
   velocity × distance / 2 m/s, at most 0.5 s ahead). Two hands going for one point
   are aimed 8 cm apart across the reach, so they come together without fighting.
   The point is brought within `MaxExtension` of the arm's length (shoulder to finger
   tips) from the shoulder.
3. **How fast**: the arm aims at a point at most 0.8 m × `Speed` ahead of the finger
   tips, on the way to the goal. The muscles pull the hand towards a point never far
   ahead, which sets its pace; fitted so that the 2.5 scene's speed 0.7 reaches in
   endorphin's 20-24 frames.
4. **Two bones**: the elbow goes out from the chest (and a little down) from the line
   to that point, the forearm reaching to the finger tips. The upper arm is turned
   whole, twist included, so its front (the way the character faces at rest, which is
   the side an elbow folds towards, in a T-pose as with hanging arms) faces the way the
   forearm goes, and the elbow bends about its own hinge. The collarbone (any bodies
   between the trunk and the upper arm) takes 30% of the turn from where the arm
   points at rest to the aim: the shoulder joint alone can't bring a hand across the
   chest. Wrist and fingers are held straight along the forearm by `Orientation` when
   `applyHandOrientation` is set. Arm muscles use the rig's `Strong` preset ×
   `Strength`.
5. **Look**: the head is turned to face the target, the top of the head towards
   `HeadUpVector` (up) when `UseUpVector`, at most 80° from straight on the chest; the
   neck and head joints share the turn evenly, at the `Strong` preset. The head is
   never aimed more than 1.5 rad × `LookAtSpeed` past where it is.
6. `Delay` and `Blend` (and `LookAtDelay`, `LookAtBlend`) ease each hand and the head
   in from the timeline's pose, from the time since the event began.

`Layout` gets the bones for this from the rig's shape: the arm branch's finger tips,
and the upper arm and forearm every behaviour uses. On endorphin's character, whose
forearm is two bodies (forearm and forearm twist), those used to be the collarbone and
the upper arm, and reaching bent at the wrong joint; Look and Reach first had its own
pick (the pair with the longest reach, weighed by how evenly the elbow splits it).
Since the arm finder looks past forearm twists ([Finding the limbs](#finding-the-limbs)),
both picks found the same bones on every rig tested (the stand-ins and
all sixteen sample scenes), and Look and Reach uses the shared one.

### Results

On endorphin's scene (NeuroMotion's run drawn into endorphin's view), arms (shoulder to finger tips) and head
pointing off the box, in degrees:

| frame | character01 arms, endorphin | ours | character02 far arm, endorphin | ours | character02 head, ours |
|---|---|---|---|---|---|
| 0 | T-pose | 104, 92 | T-pose | 153 | 61 |
| 12 | swinging in | 52, 42 | still out | 97 | 24 |
| 24 | on the box | 12, 11 | starting | 33 | 13 |
| 40 | on the box | 9, 10 | across the chest | 17 | 9 |
| 60 | on the box | 9, 9 | on the box | 14 | 8 |

The finger tips drawn into endorphin's view land on average 24 px from where
endorphin's hands are at 14 points read off its frames (12 to 60), at most 45 px; an
arm is about 105 px long there. Before Look and Reach (the characters standing in
their T-pose) it was 81 px on average, at most 206. Ours sit about 10° below their aim
(the finger tips 10 cm under it): endorphin's shoulder doesn't twist far enough for
the elbow to fold upwards, and an elbow pointing straight down had them 15° low.
character02's far arm sets off at once where endorphin's waits until frame 24-30,
and both get there by 40-60.

On the stand-in (test `look_and_reach_points_the_arms_and_head_at_the_target`): the
placeholder with fingers, chest locked, a ball 1.2 m in front of it, up to its left,
out of reach, the 2.5 scene's settings. Both arms point within 20° of it at frame 22
starting with hanging arms and 23 from a T-pose (endorphin: 20-24); the head within
15° at frame 4 (its neck is light); at 2 s the arms are 3° off and the head 1°.
Following a ball moving 1.5 m sideways over 3 s, the arms stay within 8° of it and
the head within 6°; a ball within reach gets both hands' finger tips to 4-5 cm of its
centre. Two runs are identical to the bit, and with the ball moved by up to 4e-4 m
(`nudges`) the arms arrive at the same frame and end within 0.001° of where they did.
No joint stretches over 0.5 mm.

On Tackle, the sample scene, Look and Reach runs for frames 40-48 on the runner
(character01), its left hand reaching for the tackler's head: the runner is only
simulated from frame 42, and the tackle comes in at once. By frame 48 its left hand is
0.68 m from the tackler's head (0.78 m without the behaviour) and its head 90° off
the tackler where it had turned away to 109°: the tackler is 83° off at the start, past
the 80° the head may turn, and there are 6 frames. Drawn into endorphin's view
(camera fitted to 6 points, 7 px rms), the two runs agree on where the bodies are
through the tackle, the runner's right arm flung out included, but endorphin's
recording is too tangled in those frames to read its left hand or head from.

Of the 16 sample scenes (`sample_scenes_simulate_stably`, whole range), the six that
run Look and Reach change and still pass; the rest are unchanged to the last digit:

| Scene | Before | After |
|---|---|---|
| Catch Rope | lowest 0.017 m, stretch 11.2 mm, fastest 21.4 m/s, lock drift 12 mm | 0.013 m, 15.1 mm, 19.3 m/s, 1 mm |
| Catch Run | lock drift 14 mm | 29 mm |
| Heli Leap | lowest -1.906 m, lock drift 24 mm | -1.614 m, 39 mm |
| Rotating Floor | lowest -4.734 m, stretch 5.6 mm, lock drift 2 mm | -4.671 m, 8.1 mm, 6 mm |
| Tackle | lowest 0.015 m, fastest 9.2 m/s | 0.019 m, 9.3 m/s |
| Trapeze | lowest 0.017 m, stretch 10.9 mm, fastest 29.3 m/s | 0.010 m, 9.9 mm, 15.9 m/s |

Heli Leap first drifted 20 cm: its catcher's hand reaches only after a 1.5 s delay,
while a join already holds it to the other character's, and Look and Reach held that
arm stiffly in its old pose meanwhile. Now arms and neck keep the timeline's muscles
until a reach or the look takes them over; only the legs and trunk are set to
`LegTorsoStiffness`.

### Not yet

- `DistanceScale` (endorphin's "slowdown"), `PredictionRate`, `FingerAngle`,
  `Exaggeration`, `MissilePathOffset` and the acceleration thresholds are imported but
  not read. Prediction uses the target's velocity alone.
- The hands come in about 10° low (above).
- What `CatchOffset` is relative to is a guess (world axes, added to the target);
  Catch Run's (-3.4, 0, -4.2) suggests it may be something else.
- A collision surface on a body with several shapes is aimed at by the body's centre,
  as NeuroMotion doesn't keep shape names: Bridge Escape looks at a piece of its
  scenery that way (`background.root.background.Box14`).
- Only the demo scene has been compared with endorphin by numbers; the trunk was
  locked there, so how endorphin's trunk and legs follow a reach (`LegTorsoStiffness`)
  hasn't been seen.

## Explosion (Writhe in mid-air)

Goal: endorphin's "Writhe in mid-air", a character thrown through the air (blown up,
hit by a car, kicked) flailing until it lands. endorphin 2.0 calls the behaviour
`Explosion`, 2.5 `WritheInMidAir`; both show as "Writhe in mid-air". Seven scenes use
it: Car Hit and its two tutorial copies (frames 48-60, as the pedestrian goes over the
car), Car Crash, Multi Martial (three times), Bridge Escape and the behaviour scene
itself.

### What endorphin does

Measured from a recording of endorphin's behaviour scene, "Writhe in mid-air 2.5"
(two characters let go 4 m up, its feet at 4.0 m; Amplitude and Speed 0.6, Strength
0.5, BackArch 1, HeadTurn 0.6), frame by frame with its frame counter. The left
character starts in a T-pose:

- **Arms.** Both arms swing up and down together, from out to the sides: up to about
  +35° (+38° for one arm) between frames 20 and 30, down to -40° to -58° at 42-46,
  coming back up (-25°) at 52. One cycle is about 45-50 frames (0.8 s). The two arms
  move nearly together, one a few frames behind the other. Elbows bend as the arms
  come up, a forearm folding in over the head around frames 30-38.
- **Legs.** Small: the knees bend about 30° and the legs part a little between
  frames 14 and 28, then straighten.
- **Trunk.** No visible curl. Seen from the front, an arch backwards can't be measured;
  in Car Hit the pedestrian stretches out face down over the bonnet, back arched and
  legs trailing (frames 54-58).
- **Rotation.** None: it falls upright, as it started, until it lands.
- **Landing.** Feet first at frame 56-58, then it goes over backwards onto its back,
  flat by about frame 70, and lies still: frames 118-142 are identical.

The settings scenes use (from their events): Speed 0.3-0.8, Strength 0.5-1,
ShoulderAmplitude 0.3-0.8, ElbowAmplitude 0.3-0.6, LegAmplitude 0.3-1, BackArch 0 or
1, HeadTurn 0.6, PhaseOffset 0; WritheInMidAir has one Amplitude instead of the three.

### Design

An original design: every target a sine of the time since the event began
(`Drive::elapsed`), at a rate of 12 rad/s × Speed (7.2 rad/s, 52 frames a cycle, at
Speed 0.6), plus PhaseOffset cycles. Nothing is remembered between ticks.

1. **Arms** are aimed in the chest's frame (as Catch fall's are, so a T-pose rig and
   one resting with its arms down work alike): out to the sides, swung up and down
   together by ±1.1 × ShoulderAmplitude rad. Both arms move together, so their swing
   doesn't turn the body. The forearms bend by up to 1.5 × ElbowAmplitude rad, most
   just after the arms are highest, bringing the hands in over the head.
2. **Legs** pedal in turn: hips ±0.6 × LegAmplitude rad, knees bending up to
   1.0 × LegAmplitude rad as their hip comes forward, each spread 0.15 rad to its side.
3. **Trunk** arches backwards by 0.4 × BackArch rad, shared by its joints (neck and head
   included); the head turns from side to side by ±0.5 × HeadTurn rad at half the rate.
4. **Muscles** are the rig's `Normal` preset × Strength. The arms are damped critically
   for the limb they swing (like Balance's swinging leg): with the preset's damping,
   tuned to hold poses against the ground, they barely moved (±20°). The legs keep the
   preset's damping: damped critically, they kicked Car Hit's pedestrian off the
   bonnet, its centre of mass rising to 2.2 m instead of endorphin's 1.8.
5. **Only in the air.** While any of its bodies pushes on something else (the ground,
   scenery, another character, from the contacts of the last substep: `World::pressing`)
   the flailing stops and the pose holds. On the bonnet, flailing pushed Car Hit's
   pedestrian along the car, landing it 0.5 m ahead of endorphin's and 10 frames early.
6. **Landing.** The lower its lowest body is over the last 0.35 m above the ground
   plane (from 0.5 to 0.15 m), the weaker its legs; lying down near the ground (the
   trunk tilted 46° to 74° from upright), all its muscles fade out, so it lies still.
   Holding the pose, even at a third of the strength, it sat up again or rolled over
   for a second or more; with only the trunk's tilt deciding, it stood on its feet in
   some runs. The legs keep their strength for the first 0.25 s, so a character the
   behaviour starts on while it stands (Multi Martial kicks two) keeps its feet: going
   limp there, one had its hand crushed by its attacker's keyed arm (21 cm of
   stretch).

### Results

The stand-in `mid_air` drops a character from 4 m running Explosion with the 2.5
scene's settings, imported the way endorphin writes them (`Bob2: WritheInMidAir 2.5`).
On a rig like endorphin's (clavicles, forearm twists, resting in a T-pose; test
`explosion_flails_in_mid_air_like_endorphins`):

| | endorphin (video) | NeuroMotion |
|---|---|---|
| Arms highest | +35-38° at frames 20-30 | +44° at frame 24 |
| Arms lowest | -40° to -58° at 42-46 | -56° at frame 50 |
| Turned in the air | none visible | pelvis 9° (the arch) |
| Lands | feet first, frame 56-58 | feet first (toes), frame 54 |
| Then | on its back by ~70, still | trunk 89° from upright, bodies moving 0.01 m/s at frames 190-210 |

Limp, the same character drops with its arms still out (within 0.1°) and lands
sprawled. On the placeholder, whose arms hang at rest, the arms come up to +24° and
swing down to -34° (`explosion_works_on_the_placeholder_and_is_deterministic`), and two
runs are bit-identical. The numbers in the air don't move when the drop height is
nudged by up to ±5 cm; the landing settles the same way in all 11 nudged runs (trunk
89-90°, at most 0.02 m/s).

In endorphin's own scene, with its characters, NeuroMotion's left character's arms
reach +44° at frame 16 and -40° at 40; it lands feet first at 55, is on its back
(trunk about 90° from upright) by frame 80-90 and still from about 92. Before, both
characters fell with their arms out and landed limp.

**Car Hit** (key numbers from the frame-by-frame comparison with endorphin's run):

| | endorphin | before | Explosion | merged with lessons 25-27 |
|---|---|---|---|---|
| Centre of mass highest | 1.81 m at frame 68 | 1.49 m at 64 | 1.75 m at 68 | 1.62 m at 65 |
| Centre of mass x at frames 68 / 78 / 88 | 0.02 / -0.11 / -0.11 m | -0.14 / -0.15 / -0.17 m | 0.03 / 0.05 / 0.06 m | 0.06 / 0.08 / 0.05 m |
| Upside down | frames 76-88 | about 74 | about 75 | head below the centre of mass 63-83 |
| First touches the ground | feet first, about frame 96, at -0.3 to -0.44 m | frame 90 (toes), at -0.17 m | feet coming down at 96 (toes' centres 7 cm up), then the hands, at frame 100, at -0.31 m | toes at frame 89, centre of mass at -0.27 m (z) |

So the flight is higher and longer, as endorphin's is, and lands in the same place.
Merged with the limb cap and the other engine changes (the last column, and the
arms' damping counting the whole arm, lesson 28) it flies 0.2 m lower than
endorphin's and comes down about 7 frames early. It isn't gravity (free fall is
right, docs/physics-design.md lesson N): it is the swing sizes read the way lesson 27
reads them. Read the old way round it flies 1.80 m high at frame 69 and drifts back as
endorphin's does, but lands on a hand at 99; why it flies lower with the reading that
fits endorphin's stored poses is still open.
NeuroMotion already turned the pedestrian over head first before (the trunk turned
360°, with the car and "Guide character to roll"); it still turns over a few frames
early, and drifts slightly forward (+0.06 m) where endorphin's drifts back (-0.11 m).
What remains may lie elsewhere: BodyFoetal (frames 60-105) and the landing animation's
priorities, which NeuroMotion doesn't use yet.

### Not yet

- **HeadTurn** and **BackArch** couldn't be checked: the recording shows the characters
  from the front. The side-to-side head turn and the 0.4 rad arch are guesses at what
  the names mean.
- endorphin's arms rise a little later (highest at 20-30, ours at 16-24) and fall
  faster than a sine; matching the shape would need more than one recording.
- `Strength` scales the `Normal` preset, as everywhere else; how endorphin scales it
  isn't known.
- Landing goes limp; whether endorphin's characters keep any muscle on the ground
  isn't known (they lie still either way).

## Solver changes behind them

Balance exposed a limit in the spring cap (lesson 4 in `physics-design.md`): an
ankle's muscle was capped as if it moved the 1.1 kg foot, giving about 270 N·m/rad,
less than the ~370 N·m/rad each ankle needs just to hold a standing body up.
Bodies resting on the ground or scenery now count as immovable when the cap
estimates the mass a muscle moves (lesson 9).

Stepping exposed another (lesson 16): friction gripped contacts that were only inside
the 2 cm speculative margin, not touching, at its full fixed force cap. A swinging
foot within 2 cm of the floor was held in place, and its hip and knee muscles couldn't
move it. Friction now only acts on contacts that touch or push.

NeuroMotion's own projects now use Coulomb friction, as imported scenes do (lesson
21). Balance and Stagger were made to work with it before the switch (*Swing height*,
*Friction*, and Stagger above).

## Next

The rest of endorphin's library still needs designs: protective reactions (protect the
head, brace for impact), grabbing and holding on, hit and shot
reactions, and others found in imported scenes. [behaviour-inventory.md](behaviour-inventory.md)
lists what the sample scenes use and ranks the missing ones; Look and Reach, first
there, and Explosion are done. Behaviour event settings are imported now; only Look
and Reach and Explosion read them so far, and the others should before they are tuned
against endorphin. See [ROADMAP.md](../ROADMAP.md), milestone 2.
