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
and `catch fall` both work. Where several overlap, the one that starts last wins.
Names with no controller yet (`Stagger`, `Writhe`, `BodyFoetal`, …) are ignored: the
character keeps doing what the timeline otherwise asks.

The behaviour starts from the targets and strengths the timeline gives (active pose,
active animation, or the pose held since simulation began) and changes what it needs
to. Controllers keep no state between ticks; everything is read from the bodies, so
simulations stay deterministic.

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

## Solver change behind them

Balance exposed a limit in the spring cap (lesson 4 in `physics-design.md`): an
ankle's muscle was capped as if it moved the 1.1 kg foot, giving about 270 N·m/rad,
less than the ~370 N·m/rad each ankle needs just to hold a standing body up.
Bodies resting on the ground or scenery now count as immovable when the cap
estimates the mass a muscle moves (lesson 9).

## Ideas for the others

- **Stagger**: balance whose support is allowed to move: when the capture point leaves
  the feet, step the swing foot to it.
- **Writhe**: low-strength, slowly wandering targets around a curled pose, stronger
  on the trunk than the limbs.
- **Body foetal**: blend the targets towards a curled pose with high trunk and hip
  strength, arms over the head.
