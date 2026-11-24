## 🗺️ Roadmap

### ✅ Done
- Opens **endorphin 2.5** scenes, characters and poses (`.ens`, `.nmc`, `.nma`)
- Own physics engine at **120 Hz**: all **16 endorphin sample scenes** simulate stably over their **full length**
- **Car Hit matches endorphin**: compared frame by frame with endorphin's own run
- Friction and constraints that behave like endorphin's, smooth transitions back to animation
- Behaviours: **Balance** (with stepping), **Stagger**, **CatchFall**, **BodyFoetal**, **Writhe**, now running in imported scenes too
- Sever events, constraints, active poses, forces
- Export to **FBX** and **BVH**, bake simulation to keys, optionally at 2x or 4x the frame rate
- Smooth playback on 60 Hz+ displays: frames are interpolated between physics frames
- **v0.1.0 beta** with downloads for Windows, macOS and Linux, and our new logo

### 🔨 In progress
- Comparing **more scenes with endorphin, by numbers**, so every change is checked against endorphin's real results
- Active poses applied the way endorphin does (Back-Flip-To-Combo)
- New behaviours: **Look and Reach** and **Explosion**, designed from endorphin's own runs
- Balance and Stagger tuned for the new friction, and stepping sideways
- **Physics settings panel** (gravity, materials, friction) and a **scene tree**

### 📋 Planned
- More behaviours: **JumpDive**, **Legs Kick**, **Arms Windmill** and the rest of endorphin's library
- Import your own **BVH/FBX** animation
- A proper **3D viewport** with shaded bodies and several views
- A **toolbar** to select, move and rotate, and to add shapes, props, constraints and forces
- Behaviour settings in the inspector
- Signed installers for Windows and macOS

The latest state is always in the [GitHub issues](https://github.com/Rick007110/NeuroMotion/issues) and the [roadmap](https://github.com/Rick007110/NeuroMotion/blob/main/ROADMAP.md).
