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

Explosion reads the settings on its event (below); the others don't read any yet.
The importer keeps a behaviour's parameters on its clip under endorphin's own names
(`Speed`, `Strength`, ...; see [endorphin-formats.md](endorphin-formats.md)).

The behaviour starts from the targets and strengths the timeline gives (active pose,
active animation, or the pose held since simulation began) and changes what it needs
to. Controllers keep no state between ticks; everything is read from the bodies, so
simulations stay deterministic. The one thing they are told besides the bodies is how
many seconds the behaviour has been running (`Drive::elapsed`), which Writhe uses for
its slow wandering; that comes from the frame number, so it is deterministic too.

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
   a gentle lift only sinks the hips onto it. In the air it is carried at 10% of the
   leg's length towards the target and lowered as it gets there. Its muscles are
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

### Results

Pushes on the placeholder's chest from 1.0 to 3.0 m/s, every 0.1 m/s, each way; `#`
means the head stayed above 1.45 m for the next 3 s (`push_sweep`, an ignored test):

| | forwards | backwards | to its left | to its right |
|---|---|---|---|---|
| Before stepping | `#######..............` | `####.................` | `##########...........` | `########.............` |
| Ankle strategy alone, after the friction fix below | `#######..............` | `####.................` | `#########............` | `#######..............` |
| Balance (stepping) | `########.#.##########` | `########.###.........` | `##########........#..` | `#########............` |

In all: 29 of 84 pushes before, 50 now, and every push survived before is still
survived in this run. In speeds: forwards up to 1.6 m/s before, now up to 3.0 (1.8
and 2.0 aside); backwards up to 1.3, now up to 2.1 (1.8 aside); to the left 1.9 both
before and now; to the right 1.7 before, 1.8 now. The steps leave the feet 0.25 to
0.6 m from where they stood, in 2 to 7 steps, with joints stretching under 3 mm.

Each row is one run on one machine, and near the limit it is sharper than the truth:
whether a push is survived turns on small differences in where a foot lands, and
another platform's rounding is enough to change it (CI fell at 1.9 m/s to the left on
Linux and at 1.8 to the right on Windows). With each push's speed nudged by up to
±1e-3 (101 runs, `nudge_survey`), the share of runs in which Balance stepped and stayed
up (head above 1.45 m, hips above 0.8 m):

| | |
|---|---|
| backwards | 1.4-1.7 m/s: 91-100%; 1.8: 36%; 1.9 and 2.0: 91-95%; 2.1: 53% |
| forwards | 1.7: 77%; 1.8: 6%; 1.9: 64%; 2.0: never; 2.1: 58%; 2.2 and 2.3: 90-95%; 2.4: 53%; 2.5-2.8: 94-100% |
| to its left | 1.9: 71%; 2.0-2.4: never; 2.5: 47%; 2.6: 3%; 2.7: 63%; 2.8: always |
| to its right | 1.7: always; 1.8: 42%; 1.9-2.8: never |

So the test, `balance_steps_when_the_ankles_cannot_hold`, counts, in each set of pushes
past what the ankles hold, how many Balance steps and stays up after: at least 4 of
the 8 backwards from 1.4 to 2.1 m/s, 2 of the 22 sideways (left 1.9-2.8, right
1.7-2.8) and 5 of the 12 forwards from 1.7 to 2.8. Over the nudged runs the counts
were 5-8, 3-7 and 6-11. `balance_keeps_the_placeholder_upright_after_a_push`, what
the ankle strategy held before stepping, is unchanged.

### Not yet

- **Sideways.** Pushed sideways, the body falls towards the foot on that side, which
  carries the weight: lifting it drops the hips onto it, and the other foot can't
  cross over to catch the body (the legs collide; a crossover target was tried, and
  the foot jammed against the other leg). So sideways Balance is barely better than
  before. It needs a weight shift onto the far foot before the near one lifts (tried
  with the ankle strategy alone: far too slow against a fall already under way), or a
  crossover step that goes around the other leg.
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
- Its steps go just where the capture point will be, not past it, so one step rarely
  stops the body and it keeps stepping.
- The trunk, head and arms are at 35% of the rig's `Normal` preset, so the upper body
  sways and lags behind the legs.

Results (`push_sweep` with `NM_SWEEP=Stagger`, same pushes as above):

| | forwards | backwards | to its left | to its right |
|---|---|---|---|---|
| Stagger | `#######.#....#..#....` | `#####.####.#.........` | `##########...........` | `#########............` |

39 of 84 stay up, fewer than with Balance, as meant. Pushed hard forwards (2.7 to
3.0 m/s) it keeps stepping, 3 to 10 steps a push, and goes down, its feet going 4.5
to 8.3 m over the four pushes, where Balance stops in a few steps and nearly always
stands, its feet going 1.9 to 2.3 m (101 runs with the speeds nudged by up to ±1e-3,
`nudge_survey`). The test, `stagger_keeps_stepping_after_a_hard_push`, checks those
four pushes for at least 2 steps after each, 12 in all, and feet going more than 1.5
times as far as Balance's. A single push can't be checked on its own: at 2.5 m/s it
took 6 steps over 1.1 m here, 2 on every CI platform, and 2 to 6 over 9 nudged runs.

**Not yet.** Settings on the behaviour event aren't read, and nothing has been
compared with endorphin. It inherits Balance's limits (above), sideways most of all.
Stagger Walk,
the sample scene that uses it, now runs this controller instead of going limp, but it
couldn't be checked here (no endorphin install).

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
The head and chest follow half a second or more later, or the arms hold them off the
ground altogether. Shoved forwards limp, it lands on its face.

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

Results (test `writhe_keeps_moving_and_is_deterministic`): after 3 s, 14 of the
placeholder's 17 bodies still move at over 0.1 m/s (mean 0.25 m/s), no NaN, joints
stretched at most 1.4 mm, fastest body 5.8 m/s. Two runs give bit-identical bodies.

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

| | endorphin | before | Explosion |
|---|---|---|---|
| Centre of mass highest | 1.81 m at frame 68 | 1.49 m at 64 | 1.75 m at 68 |
| Centre of mass x at frames 68 / 78 / 88 | 0.02 / -0.11 / -0.11 m | -0.14 / -0.15 / -0.17 m | 0.03 / 0.05 / 0.06 m |
| Upside down | frames 76-88 | about 74 | about 75 |
| First touches the ground | feet first, about frame 96, at -0.3 to -0.44 m | frame 90 (toes), at -0.17 m | feet coming down at 96 (toes' centres 7 cm up), then the hands, at frame 100, at -0.31 m |

So the flight is higher and longer, as endorphin's is, and lands in the same place.
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

## Next

The rest of endorphin's library still needs designs: protective reactions (protect the
head, brace for impact), reaching and grabbing, looking at a target, hit and shot
reactions, and others found in imported scenes. [behaviour-inventory.md](behaviour-inventory.md)
lists what the sample scenes use and ranks the missing ones (Look and Reach, 11 scenes,
comes first). Behaviour event settings are imported now; only Explosion reads them
so far, and the other behaviours' settings (strengths, speeds, targets) come before
tuning them against endorphin. See [ROADMAP.md](../ROADMAP.md), milestone 2.
