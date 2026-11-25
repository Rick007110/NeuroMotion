## 🗺️ Roadmap

### ✅ Done
- Opens **endorphin 2.5** scenes, characters and poses (`.ens`, `.nmc`, `.nma`)
- Own physics engine at **120 Hz**: **all 17 endorphin sample scenes** simulate stably over their **full length**, faster than real time, Bridge Escape's tumbling truck included
- **Car Hit matches endorphin**: compared frame by frame with endorphin's own run
- Friction and constraints that behave like endorphin's, smooth transitions back to animation
- Balance and Stagger on endorphin's friction, now used by NeuroMotion's own projects too
- Behaviours: **Balance** (with stepping), **Stagger**, **CatchFall**, **BodyFoetal**, **Writhe**, now running in imported scenes too
- **Look and Reach** (hands reach for a target, the head looks at it) and **Explosion** (writhe in mid-air), compared with endorphin's own runs
- Sever events, constraints, active poses, forces
- Export to **FBX** and **BVH**, bake simulation to keys, optionally at 2x or 4x the frame rate
- Smooth playback on 60 Hz+ displays: frames are interpolated between physics frames
- **Same results on Windows, macOS and Linux**, bit for bit
- **Exact cylinders**: flat caps and a sharp rim, so posts, forearms and logs rest on them as they should
- **Scored against endorphin**: 68 measurements of endorphin's own runs in 8 scenes, checked by a test on every change
- Edit the scene's **physics settings** (gravity, rate, materials) in the app, and browse the scene's bodies in a **scene tree**
- **v0.1.0 beta** with downloads for Windows, macOS and Linux, and our new logo

### 🔨 In progress
- Comparing **more scenes with endorphin, by numbers**: exports of endorphin's results instead of recordings, and more scenes
- Active poses applied the way endorphin does (Back-Flip-To-Combo)
- Stepping on endorphin's own characters (Stagger Walk), and stepping sideways
- **Virtual muscles**: active poses pull the hands and heels toward the pose

### 📋 Planned
- More behaviours: **JumpDive**, **Legs Kick**, **Arms Windmill** and the rest of endorphin's library
- Import your own **BVH/FBX** animation
- A proper **3D viewport** with shaded bodies and several views
- A **toolbar** to select, move and rotate, and to add shapes, props, constraints and forces
- Behaviour settings in the inspector
- Signed installers for Windows and macOS

The latest state is always in the [GitHub issues](https://github.com/Rick007110/NeuroMotion/issues) and the [roadmap](https://github.com/Rick007110/NeuroMotion/blob/main/ROADMAP.md).
