import time
import math
import numpy as np
import mujoco
import mujoco.viewer
import os

# --- CONFIG ---
# Time parameters for the grasping sequence (based on typical phases)
T_PACKING = 2.0      # Time to curl into the spiral (F1 high, F2/F3 low)
T_REACHING = 3.0     # Time to extend/reach out (F2/F3 increase antagonistic pull)
T_WRAP_GRASP = 3.0   # Time to wrap/climb and secure the grasp (F1 decreases, F2/F3 adjust)
T_HOLD = 2.0

# Actuation length control values from tentacle.xml ctrlrange="0.120 0.340"
# Smaller control value means more cable pulled (tighter curl/higher force)
CTRL_TIGHT_CURL = 0.120  # Minimum ctrlrange (maximum pull/tightest curl)
CTRL_STRAIGHT = 0.340    # Maximum ctrlrange (relaxed/straight)

# Initial "Packing" force for the antagonist cables (F2, F3)
CTRL_PACK_ANTAGONIST = 0.300 

# Object Definition
OBJECT_POS = [0.0, 0.25, 0.0]  # Target location for the grasping box (X, Y, Z)
OBJECT_SIZE = [0.05, 0.05, 0.05]
OBJECT_NAME = "grasp_box_geom"
BOX_BODY_NAME = "grasp_box_body"

# --- HELPER FUNCTIONS ---

STATE = "PACKING"
STATE_START_TIME = 0.0

def set_state(name, start_time):
    """Transition to a new state and record start time."""
    global STATE, STATE_START_TIME
    STATE = name
    STATE_START_TIME = start_time
    print(f">> 🤖 Transitioning to state: {STATE}")

def get_target_ctrl(t_progress, start_ctrl, end_ctrl):
    """Linear interpolation of control signal based on time progress."""
    return start_ctrl + (end_ctrl - start_ctrl) * t_progress

def log_spiral_control(model, data, time_sec):
    """
    Implements the core 3-cable SpiRobs grasping strategy using cable control.
    F1 (tendon_1) is used as the primary curling cable. F2/F3 are antagonists.
    """
    global STATE, STATE_START_TIME

    time_in_state = time_sec - STATE_START_TIME

    # --- 1. PACKING (Curl into a spiral from the tip) ---
    if STATE == "PACKING":
        t_progress = min(1.0, time_in_state / T_PACKING)
        
        # F1 (Curling cable) pulls: from relaxed (straight) to tight curl
        f1_target = get_target_ctrl(t_progress, CTRL_STRAIGHT, CTRL_TIGHT_CURL)
        
        # F2/F3 (Antagonist cables) maintain light tension to manage curl direction
        f2_target = CTRL_PACK_ANTAGONIST
        f3_target = CTRL_PACK_ANTAGONIST

        # Check for transition
        if t_progress >= 1.0:
            set_state("REACHING", time_sec)
        
    # --- 2. REACHING (Uncurl from base to reach target) ---
    elif STATE == "REACHING":
        t_progress = min(1.0, time_in_state / T_REACHING)
        
        # F1 maintains the tight curl tension
        f1_target = CTRL_TIGHT_CURL
        
        # F2/F3 (Antagonists) pull stronger to enable reaching/straightening from the base
        f2_target = get_target_ctrl(t_progress, CTRL_PACK_ANTAGONIST, CTRL_TIGHT_CURL)
        f3_target = get_target_ctrl(t_progress, CTRL_PACK_ANTAGONIST, CTRL_TIGHT_CURL)
        
        # Check for transition (In a real system, this would be triggered by contact)
        if t_progress >= 1.0:
            set_state("WRAPPING", time_sec)

    # --- 3. WRAPPING & GRASPING (Climb and secure grasp) ---
    elif STATE == "WRAPPING":
        t_progress = min(1.0, time_in_state / T_WRAP_GRASP)
        
        # F1 (Curling cable) relaxes to allow wrapping/climbing over the object.
        f1_target = get_target_ctrl(t_progress, CTRL_TIGHT_CURL, CTRL_STRAIGHT)
        
        # F2/F3 (Antagonists) maintain strong pull to secure the wrap.
        f2_target = CTRL_TIGHT_CURL 
        f3_target = CTRL_TIGHT_CURL
        
        # Check for transition
        if t_progress >= 1.0:
            set_state("HOLDING", time_sec)
            
    # --- 4. HOLDING (Maintain the strong, wrapped grasp) ---
    elif STATE == "HOLDING":
        # Final control state to maximize grasp stability
        f1_target = CTRL_STRAIGHT # Relaxed cable
        f2_target = CTRL_TIGHT_CURL
        f3_target = CTRL_TIGHT_CURL
        
        if time_in_state >= T_HOLD:
            set_state("DONE", time_sec)
            
    # --- 5. DONE ---
    elif STATE == "DONE":
        # No change, maintain final grasp
        f1_target = data.ctrl[0]
        f2_target = data.ctrl[1]
        f3_target = data.ctrl[2]
        
    else:
        # Default/uninitialized state
        f1_target = CTRL_STRAIGHT
        f2_target = CTRL_STRAIGHT
        f3_target = CTRL_STRAIGHT


    # Apply the calculated control signals to the actuators
    data.ctrl[0] = f1_target # actuator_1 (tendon_1)
    data.ctrl[1] = f2_target # actuator_2 (tendon_2)
    data.ctrl[2] = f3_target # actuator_3 (tendon_3)
    
    # Update viewer text
    # Monitor the position of the tip_center site for movement
    tip_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, b'tip_center')
    tip_pos_z = data.site_xpos[tip_site_id, 2] if tip_site_id >= 0 else 0.0
    
    mujoco.viewer.user_data.text = (
        f"State: {STATE} | Time: {time_sec:.2f}s | "
        f"Tip Z: {tip_pos_z:.3f}m\n"
        f"F1: {data.ctrl[0]:.3f} (Curling), F2/F3: {data.ctrl[1]:.3f} (Antagonists)"
    )
    

def append_box_to_xml(xml_string):
    """
    Inserts a simple graspable box into the MuJoCo XML.
    """
    box_xml = f"""
    <body name="{BOX_BODY_NAME}" pos="{OBJECT_POS[0]} {OBJECT_POS[1]} {OBJECT_POS[2]}">
        <geom name="{OBJECT_NAME}" type="box" size="{OBJECT_SIZE[0]} {OBJECT_SIZE[1]} {OBJECT_SIZE[2]}" 
              rgba="0 0.5 1 1" mass="0.5" friction="1 0.05 0.05"/>
    </body>
    """
    # Find the closing tag of the worldbody and insert the new body before it
    closing_tag = '</worldbody>'
    if closing_tag in xml_string:
        return xml_string.replace(closing_tag, box_xml + '\n' + closing_tag)
    return xml_string

# --- MAIN EXECUTION ---

def main():
    global STATE_START_TIME

    # 1. Load the model from the provided XML file
    try:
        xml_path = 'tentacle.xml'
        with open(xml_path, 'r') as f:
            model_xml = f.read()
    except FileNotFoundError:
        print(f"Error: XML file '{xml_path}' not found.")
        return

    # 2. Add the graspable object to the model XML
    modified_xml = append_box_to_xml(model_xml)

    # 3. Create MuJoCo Model and Data
    try:
        model = mujoco.MjModel.from_xml_string(modified_xml)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error compiling MuJoCo model: {e}")
        # print("The raw XML that failed compilation is:")
        # print(modified_xml)
        return
    
    # Initial control
    data.ctrl = np.array([CTRL_STRAIGHT, CTRL_STRAIGHT, CTRL_STRAIGHT])

    # 4. Setup initial state and run
    STATE_START_TIME = data.time
    print(f"Loaded SpiRobs model with {model.nu} actuators and {model.njnt} joints.")

    # 5. Launch the viewer and run the control loop
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.user_data.text = "" 
        
        while viewer.is_running():
            step_start = time.time()
            
            # --- Call the logarithmic spiral control function ---
            log_spiral_control(model, data, data.time)
            
            # --- Standard MuJoCo Step ---
            mujoco.mj_step(model, data)

            # Update viewer
            viewer.sync()

            # Throttle the simulation
            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

if __name__ == "__main__":
    main()