🐙 SpiRobs: Bioinspired Octoarm Robotic Grasping System

A MuJoCo-based simulation of a soft, cable-driven, logarithmic-spiral robotic arm inspired by octopus biomechanics.

🚀 Overview

SpiRobs is a bioinspired soft-robotics simulation framework implementing a logarithmic-spiral cable-driven octoarm capable of reaching, wrapping, and grasping objects.

Built using MuJoCo, Python, and a custom 3-tendon actuation system, SpiRobs demonstrates how biological principles—such as distributed compliance and spiral contraction—can be translated into robotics.

This project explores:

Soft continuum manipulation

Multi-cable actuation

Bioinspired locomotion patterns

Dynamic grasping without fingers

Stable object capture using wrapping mechanics

🧬 Bioinspiration

The system is inspired by:

Octopus arms — highly flexible, muscular-hydrostat structures

Logarithmic spirals in nature — efficient wrapping & continuous surface contact

Cable-driven soft robotics — tendons act as antagonistic force generators

The model uses:

1 primary curling tendon (F1)

2 antagonistic tendons (F2, F3)

Together, these generate:

Spiral packing

Extension & reaching

Wrapping around targets

Stable grasping

⚙️ Features
✅ Realistic Soft-Body Segmentation

21 jointed mesh-based segments (STL-based)

Natural stiffness–damping gradients

High-resolution tendon routing through 63+ anatomical “sites”

🎛️ Biologically-Inspired Control States

Implemented in main.py:

PACKING – Curl into a logarithmic spiral

REACHING – Extend towards the object

WRAPPING – Climb and wrap around the object

HOLDING – Maintain a strong grasp

DONE – Stable final configuration

🎯 Automatic Object Injection

A graspable box is automatically inserted into the MuJoCo XML at runtime.

🖥️ Live Visualization

Real-time tendon control values

Tip (end-effector) position tracking

Control state overlay directly inside the MuJoCo viewer


🎮 How the Control Works

The grasping controller uses time-based tendon interpolation:

f1_target = get_target_ctrl(t_progress, CTRL_STRAIGHT, CTRL_TIGHT_CURL)


F1 → primary curling

F2 / F3 → antagonistic shaping

State switching is triggered by elapsed time within each phase

The octoarm automatically:

✔ curls

✔ reaches forward

✔ wraps around the object

✔ secures a grasp

✔ maintains stability

🧪 Example Console Output
>> 🤖 Transitioning to state: PACKING
>> 🤖 Transitioning to state: REACHING
>> 🤖 Transitioning to state: WRAPPING
>> 🤖 Transitioning to state: HOLDING
>> 🤖 Transitioning to state: DONE

📸 Snapshots (Recommended to Add)

You can include screenshots and GIFs:

/media/demo_grasp.gif
/media/spirals.png

🧠 Key Algorithms
✔ Logarithmic Spiral Grasping

The spiral form follows:

r = a * e^(bθ)


Implemented through dynamic tendon shortening.

✔ Multi-Cable Coordination

Tendons behave like biological muscles:

Curling

Extension

Shape steering

✔ Distributed Soft Segmentation

Each of the 21 soft segments includes:

Mesh geometry (STL)

Ball-joint kinematics

In/out tendon routing sites

📚 Applications

Soft robotics research

Underwater manipulation

Continuum-arm control

Multi-tendon actuation learning

Hazardous-environment robotics

Bioinspired engineering coursework

🛠️ Future Enhancements

Reinforcement learning–based grasping

Physics-informed control

Adaptive spiral generation

Tactile sensing & slip detection

Real soft-robot hardware implementation

ROS2 integration
