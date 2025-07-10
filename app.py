# -*- coding: utf-8 -*-
# Original main code from: https://github.com/gokul6350/ARMv6/blob/main/main_src/inverse_k.py
# Adapted and built into an interactive Gradio App for educational purposes.
# --- MODIFIED TO INCLUDE A CODE-COPYING DROPDOWN ---

import gradio as gr
import matplotlib.pyplot as plt
import numpy as np
import math

# --- Core Inverse Kinematics Logic (Refactored and Improved) ---
def inverse_kinematics_2dof(x, y, l1, l2):
    """
    Calculates the joint angles (q1, q2) for a 2-DOF robotic arm.

    Args:
        x (float): The x-coordinate of the end effector.
        y (float): The y-coordinate of the end effector.
        l1 (float): The length of the first arm segment.
        l2 (float): The length of the second arm segment.

    Returns:
        tuple: (success, q1, q2) where q1 and q2 are in radians.
               'success' is False if the point is unreachable.
    """
    # Check if the target is reachable
    dist_sq = x**2 + y**2
    if dist_sq > (l1 + l2)**2 or dist_sq < (l1 - l2)**2:
        return (False, 0, 0)

    # Calculate q2 using the Law of Cosines
    # The value must be clamped to [-1.0, 1.0] to avoid math errors from float precision
    cos_q2_arg = (dist_sq - l1**2 - l2**2) / (2 * l1 * l2)
    cos_q2_arg = np.clip(cos_q2_arg, -1.0, 1.0)
    q2 = math.acos(cos_q2_arg)

    # Calculate q1 using atan2 for full quadrant support
    angle_to_target = math.atan2(y, x)
    angle_l1_to_ee = math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
    q1 = angle_to_target - angle_l1_to_ee

    return (True, q1, q2)

# --- Main Simulation and Plotting Function for Gradio ---
def run_simulation(x, y, l1, l2):
    """
    This function is called by Gradio. It runs the inverse kinematics,
    generates a plot using Matplotlib, and returns the results.
    """
    success, q1_rad, q2_rad = inverse_kinematics_2dof(x, y, l1, l2)

    status_message = "Success!"
    shoulder_angle_text = ""
    elbow_angle_text = ""

    # Create the plot using Matplotlib. This is the standard way to create
    # complex visualizations for Gradio's gr.Plot component.
    fig, ax = plt.subplots(figsize=(6, 6))

    if not success:
        status_message = f"Target ({x:.1f}, {y:.1f}) is unreachable."
        ax.plot(x, y, 'rx', markersize=10, label='Unreachable Target')
        ax.legend()
    else:
        q1_deg = math.degrees(q1_rad)
        q2_deg = math.degrees(q2_rad)

        joint2_x = l1 * np.cos(q1_rad)
        joint2_y = l1 * np.sin(q1_rad)
        end_effector_x = joint2_x + l2 * np.cos(q1_rad + q2_rad)
        end_effector_y = joint2_y + l2 * np.sin(q1_rad + q2_rad)

        ax.plot([0, joint2_x], [0, joint2_y], 'r-', linewidth=4, label=f'Arm 1 (L1={l1})')
        ax.plot([joint2_x, end_effector_x], [joint2_y, end_effector_y], 'b-', linewidth=4, label=f'Arm 2 (L2={l2})')
        ax.plot(0, 0, 'ko', markersize=10, label='Joint 1 (Shoulder)')
        ax.plot(joint2_x, joint2_y, 'go', markersize=10, label='Joint 2 (Elbow)')
        ax.plot(end_effector_x, end_effector_y, 'mo', markersize=10, label='End Effector')
        ax.legend()

        shoulder_angle_text = f"{q1_deg:.2f}°"
        elbow_angle_text = f"{180.0 - q2_deg:.2f}°" # Common servo angle representation

    max_reach = l1 + l2 + 2
    ax.set_xlim([-max_reach, max_reach])
    ax.set_ylim([-max_reach, max_reach])
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('2-DOF Robotic Arm Simulation')
    ax.grid(True)
    plt.tight_layout() # Helps fit everything nicely

    # Return the Matplotlib figure to be displayed by gr.Plot, along with other values
    return fig, status_message, shoulder_angle_text, elbow_angle_text

# --- Python Code Snippet for the UI ---
# This string contains the self-contained Python function for easy copying.
# It's defined here for cleanliness and then used in the Gradio UI below.
CODE_SNIPPET = '''
import math
import numpy as np

def inver_k(l1, l2, x, y):
    """
    Calculates the joint angles (q1, q2) for a 2-DOF robotic arm.
    This is the core inverse kinematics logic.

    Args:
        l1 (float): The length of the first arm segment.
        l2 (float): The length of the second arm segment.
        x (float): The x-coordinate of the end effector.
        y (float): The y-coordinate of the end effector.

    Returns:
        tuple: (success, q1, q2) where q1 and q2 are in radians.
               'success' is False if the point is unreachable.
               Returns (False, 0, 0) on failure.
    """
    # Check if the target is reachable.
    # The distance from origin to target squared
    dist_sq = x**2 + y**2

    # Check if the target is outside the outer circle or inside the inner circle.
    if dist_sq > (l1 + l2)**2 or dist_sq < (l1 - l2)**2:
        return (False, 0, 0)

    # Calculate q2 (elbow angle) using the Law of Cosines.
    # The value must be clamped to [-1.0, 1.0] to avoid math domain errors
    # from floating-point inaccuracies.
    cos_q2_arg = (dist_sq - l1**2 - l2**2) / (2 * l1 * l2)
    cos_q2_arg = np.clip(cos_q2_arg, -1.0, 1.0)
    q2 = math.acos(cos_q2_arg)

    # Calculate q1 (shoulder angle).
    # q1 is the angle to the target minus the angle within the arm triangle.
    angle_to_target = math.atan2(y, x)
    angle_l1_to_ee = math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
    q1 = angle_to_target - angle_l1_to_ee

    return (True, q1, q2)

# --- Example Usage ---
# arm1_len = 12.5
# arm2_len = 12.5
# target_x = -10
# target_y = 15
#
# success, q1_rad, q2_rad = inver_k(arm1_len, arm2_len, target_x, target_y)
#
# if success:
#     print(f"Success! Angles in Radians: q1={q1_rad:.4f}, q2={q2_rad:.4f}")
#     print(f"Success! Angles in Degrees: q1={math.degrees(q1_rad):.2f}, q2={math.degrees(q2_rad):.2f}")
# else:
#     print("The target point is unreachable.")
'''

# --- Build the Gradio Interface ---
with gr.Blocks(theme=gr.themes.Soft(), title="2-DOF Arm Simulator") as app:
    gr.Markdown("# 2-DOF Robotic Arm: Inverse Kinematics Simulator")
    gr.Markdown(
        "This app provides an easy way to understand how a 2-DOF (two-degree-of-freedom) robotic arm works. "
        "Use the sliders on the left to set the target coordinates (X, Y) and arm lengths (L1, L2). "
        "The simulation on the right will update in real-time to show the required joint angles."
    )
    gr.Markdown("---")

    with gr.Row():
        with gr.Column(scale=1, min_width=300):
            gr.Markdown("### Controls")
            x_slider = gr.Slider(minimum=-25, maximum=0, value=-10, step=0.5, label="Target X Coordinate")
            y_slider = gr.Slider(minimum=-25, maximum=25, value=15, step=0.5, label="Target Y Coordinate")
            l1_slider = gr.Slider(minimum=1, maximum=15, value=12.5, step=0.5, label="Arm 1 Length (L1)")
            l2_slider = gr.Slider(minimum=1, maximum=15, value=12.5, step=0.5, label="Arm 2 Length (L2)")

            gr.Markdown("### Calculated Angles")
            status_output = gr.Textbox(label="Status", interactive=False)
            shoulder_output = gr.Textbox(label="Shoulder Angle (q1)", interactive=False)
            elbow_output = gr.Textbox(label="Elbow Angle (servo-style)", interactive=False)

            gr.Markdown("--- \n *Original code from [ARMv6 by gokul6350](https://github.com/gokul6350/ARMv6/blob/main/main_src/inverse_k.py)*")

        with gr.Column(scale=2, min_width=500):
            gr.Markdown("### Simulation")
            # The gr.Plot component is a "picture frame" that displays plots
            # generated by libraries like Matplotlib.
            plot_output = gr.Plot()

    with gr.Accordion("Understanding the Math: Inverse Kinematics Formulas", open=False):
        # Using a raw string r"""...""" ensures LaTeX renders correctly without Python misinterpreting backslashes.
        # Formatting is adjusted to match the source image.
        gr.Markdown(
        r"""
        To find the joint angles ($$q_1, q_2$$) for a target position $$(x, y)$$, we use the laws of geometry.

        **1. Calculate the Elbow Angle ($$q_2$$)**

        We use the **Law of Cosines** on the triangle formed by the two arm links and the line from the origin to the end-effector. The distance-squared from the origin to the target is $$d^2 = x^2 + y^2$$.

        $$ \cos(q_2) = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2} $$

        From this, we can find $$q_2$$:

        $$ q_2 = \arccos\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}\right) $$

        *Note: The angle displayed in the app for the elbow is $$180^\circ - q_2$$, which is common for servo control.*

        **2. Calculate the Shoulder Angle ($$q_1$$)**

        The shoulder angle $$q_1$$ is composed of two parts: the angle to the target point ($$\alpha$$) and the angle within the arm triangle ($$\beta$$).

        $$ q_1 = \alpha - \beta $$

        Where:
        *   $$\alpha$$ is the angle to the target point: `atan2(y, x)`
        *   $$\beta$$ is the angle between the first link and the line to the end-effector: `atan2(L2 * sin(q2), L1 + L2 * cos(q2))`

        `atan2(y, x)` is used instead of `atan(y/x)` because it correctly computes the angle in all four quadrants.
        """
        )

    # *** NEWLY ADDED DROPDOWN FOR COPYING PYTHON CODE ***
    with gr.Accordion("Copy the Core Python Function", open=False):
        gr.Code(
            value=CODE_SNIPPET,
            language="python",
            label="Python Code for inver_k(l1, l2, x, y)"
        )

    inputs = [x_slider, y_slider, l1_slider, l2_slider]
    outputs = [plot_output, status_output, shoulder_output, elbow_output]

    # Use .change for live updates as sliders move
    for component in inputs:
        component.change(fn=run_simulation, inputs=inputs, outputs=outputs, api_name=False)

    # Use .load to run the simulation once when the app starts
    app.load(fn=run_simulation, inputs=inputs, outputs=outputs, api_name=False)


if __name__ == "__main__":
    app.launch()
