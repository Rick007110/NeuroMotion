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
`bodyfetal`, and `BodyWrithe` to Writhe. endorphin's scenes name behaviours after the
set they come from and add a version, as in `Bob2: Balance 2.5`: the part up to the
last `:` and a trailing version number are dropped first. (Until this was added, no
imported scene's behaviour ran at all.) Where several overlap, the one that starts
last wins.
Names with no controller yet are ignored: the
character keeps doing what the timeline otherwise asks.

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
  collarbones, hands and fingers.

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
head, brace for impact), reaching and grabbing, looking at a target, hit and shot
reactions, and others found in imported scenes. [behaviour-inventory.md](behaviour-inventory.md)
lists what the sample scenes use and ranks the missing ones (Look and Reach, 11 scenes,
comes first). Behaviour event settings (strengths, speeds, targets) aren't read yet;
that comes before tuning any of them against endorphin. See [ROADMAP.md](../ROADMAP.md), milestone 2.
