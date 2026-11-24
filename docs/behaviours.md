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
`bodyfetal`. Where several overlap, the one that starts last wins.
Names with no controller yet (`Stagger`, …) are ignored: the
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

Goal: keep the centre of mass over the feet, upright, after a push.

The controller works on the **capture point**: where the centre of mass would come to
rest given its speed, that is position + velocity × √(height / g). Each tick:

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

Results on the placeholder (test `balance_keeps_the_placeholder_upright_after_a_push`):
after a 1.5 m/s push on the chest, forwards or to either side, the head stays at
1.70 m for the following 3 s. Without Balance the character falls.

**Not yet.** Pushes that move the capture point off the feet need a step. Backwards,
that's already the case for a 1.5 m/s chest push on the placeholder, whose heel is
only 6 cm behind the ankle. A hip strategy (bending at the hips to move the centre of
mass) was tried and made things worse as implemented. Stepping is the next thing to
add.

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

## Solver change behind them

Balance exposed a limit in the spring cap (lesson 4 in `physics-design.md`): an
ankle's muscle was capped as if it moved the 1.1 kg foot, giving about 270 N·m/rad,
less than the ~370 N·m/rad each ankle needs just to hold a standing body up.
Bodies resting on the ground or scenery now count as immovable when the cap
estimates the mass a muscle moves (lesson 9).

## Ideas for the others

- **Stagger**: balance whose support is allowed to move: when the capture point leaves
  the feet, step the swing foot to it.
