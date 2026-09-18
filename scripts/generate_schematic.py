import os

import matplotlib.patches as patches
import matplotlib.pyplot as plt


def draw_system_schematic(save_path: str):
    fig = plt.figure(figsize=(12, 6.5), dpi=300)
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6.5)
    ax.axis('off')

    # Color palette
    c_circuit = '#2c3e50'
    c_fill_elec = '#ebf5fb'
    c_fill_mech = '#fef9e7'
    c_accent = '#2980b9'
    c_mech = '#d35400'

    # Background cards
    # Electrical Domain box
    card_elec = patches.FancyBboxPatch((0.4, 3.2), 5.4, 3.0, boxstyle="round,pad=0.15",
                                      facecolor=c_fill_elec, edgecolor=c_accent, lw=1.5, ls='--')
    ax.add_patch(card_elec)
    ax.text(0.7, 5.9, "ELECTRICAL DOMAIN (Armature Circuit)", fontsize=11, fontweight='bold', color=c_accent)

    # Mechanical Domain box
    card_mech = patches.FancyBboxPatch((6.2, 3.2), 5.4, 3.0, boxstyle="round,pad=0.15",
                                      facecolor=c_fill_mech, edgecolor=c_mech, lw=1.5, ls='--')
    ax.add_patch(card_mech)
    ax.text(6.5, 5.9, "MECHANICAL DOMAIN (Rotor & Load)", fontsize=11, fontweight='bold', color=c_mech)

    # Circuit elements
    # Input voltage
    ax.plot([1.0, 1.0], [3.7, 5.3], color=c_circuit, lw=2)
    ax.plot([1.0, 1.8], [5.3, 5.3], color=c_circuit, lw=2)
    ax.plot([1.0, 4.8], [3.7, 3.7], color=c_circuit, lw=2)
    ax.scatter([1.0, 1.0], [5.3, 3.7], color=c_accent, s=40, zorder=4)
    ax.text(0.5, 4.5, "$+$\n$V(t)$\n$-$", fontsize=11, ha='center', va='center', fontweight='bold')

    # Current arrow
    ax.annotate("", xy=(1.6, 5.3), xytext=(1.2, 5.3),
                arrowprops=dict(arrowstyle="->", color='#c0392b', lw=2))
    ax.text(1.4, 5.5, "$i(t)$", fontsize=11, color='#c0392b', ha='center')

    # Resistor R
    res = patches.Rectangle((1.8, 5.1), 0.8, 0.4, facecolor='white', edgecolor=c_circuit, lw=2)
    ax.add_patch(res)
    ax.text(2.2, 5.65, "Resistance $R$", fontsize=10, ha='center')

    ax.plot([2.6, 3.0], [5.3, 5.3], color=c_circuit, lw=2)

    # Inductor L
    for i in range(3):
        arc = patches.Arc((3.2 + i*0.3, 5.3), 0.3, 0.35, angle=0, theta1=0, theta2=180, color=c_circuit, lw=2)
        ax.add_patch(arc)
    ax.plot([3.0, 3.05], [5.3, 5.3], color=c_circuit, lw=2)
    ax.plot([3.95, 4.8], [5.3, 5.3], color=c_circuit, lw=2)
    ax.text(3.5, 5.65, "Inductance $L$", fontsize=10, ha='center')

    # Motor armature circle
    arm = patches.Circle((4.8, 4.5), 0.55, facecolor='white', edgecolor=c_circuit, lw=2)
    ax.add_patch(arm)
    ax.plot([4.8, 4.8], [5.3, 5.05], color=c_circuit, lw=2)
    ax.plot([4.8, 4.8], [3.95, 3.7], color=c_circuit, lw=2)
    ax.text(4.8, 4.5, "M", fontsize=14, fontweight='bold', ha='center', va='center')
    ax.text(4.1, 4.5, "$e_b(t)$", fontsize=10, ha='center', va='center', color=c_accent)

    # Coupling interface (Electromechanical conversion)
    ax.annotate("", xy=(6.5, 4.5), xytext=(5.35, 4.5),
                arrowprops=dict(arrowstyle="<->", color='#8e44ad', lw=2.5))
    ax.text(5.9, 4.75, "Torque: $\\tau_m = K \\cdot i$\nBack-EMF: $e_b = K \\cdot \\omega$",
            fontsize=9.5, ha='center', va='bottom', color='#8e44ad', fontweight='bold')

    # Mechanical elements
    # Shaft
    shaft = patches.Rectangle((6.5, 4.35), 2.2, 0.3, facecolor='#bdc3c7', edgecolor=c_circuit, lw=1.5)
    ax.add_patch(shaft)

    # Rotor Inertia J (flywheel / cylinder)
    flywheel = patches.Ellipse((7.4, 4.5), 0.4, 1.5, facecolor='#95a5a6', edgecolor=c_circuit, lw=2)
    ax.add_patch(flywheel)
    ax.text(7.4, 5.4, "Inertia $J$", fontsize=10, ha='center', color=c_mech, fontweight='bold')

    # Viscous Damper b
    damper_base = patches.Rectangle((8.6, 3.6), 0.6, 0.4, facecolor='white', edgecolor=c_circuit, lw=1.5)
    ax.add_patch(damper_base)
    ax.plot([8.9, 8.9], [4.35, 4.0], color=c_circuit, lw=2)
    ax.plot([8.6, 9.2], [3.6, 3.6], color=c_circuit, lw=2)
    # Ground hatching
    for k in range(5):
        ax.plot([8.6 + k*0.12, 8.5 + k*0.12], [3.6, 3.4], color='gray', lw=1.2)
    ax.text(9.4, 3.8, "Friction $b$", fontsize=10, color=c_mech, va='center')

    # Output motion arrow
    arc_motion = patches.Arc((10.0, 4.5), 0.8, 0.8, angle=0, theta1=-70, theta2=160, color=c_mech, lw=2.5)
    ax.add_patch(arc_motion)
    ax.annotate("", xy=(10.25, 4.8), xytext=(10.2, 4.85),
                arrowprops=dict(arrowstyle="->", color=c_mech, lw=2.5))
    ax.text(10.7, 4.6, "Speed $\\omega(t)$\nPosition $\\theta(t)$", fontsize=10.5,
            fontweight='bold', color=c_mech, va='center')

    # Bottom Half: Feedback Control Loop Architecture
    card_ctrl = patches.FancyBboxPatch((0.4, 0.3), 11.2, 2.5, boxstyle="round,pad=0.15",
                                       facecolor='#fcfcfc', edgecolor='#7f8c8d', lw=1.2)
    ax.add_patch(card_ctrl)
    ax.text(0.7, 2.5, "CLOSED-LOOP FEEDBACK CONTROL ARCHITECTURE", fontsize=11, fontweight='bold', color='#2c3e50')

    # Reference input
    ax.annotate("", xy=(1.8, 1.5), xytext=(0.7, 1.5),
                arrowprops=dict(arrowstyle="->", color='#2c3e50', lw=2))
    ax.text(1.1, 1.7, "Ref $r(t)$", fontsize=10, ha='center')

    # Summing junction
    sum_junc = patches.Circle((2.1, 1.5), 0.25, facecolor='white', edgecolor='#2c3e50', lw=2)
    ax.add_patch(sum_junc)
    ax.text(2.1, 1.5, "$+$", fontsize=11, ha='center', va='center')
    ax.text(1.95, 1.15, "$-$", fontsize=11, ha='center', va='center')

    # Error signal
    ax.annotate("", xy=(3.0, 1.5), xytext=(2.35, 1.5),
                arrowprops=dict(arrowstyle="->", color='#2c3e50', lw=2))
    ax.text(2.65, 1.7, "Error $e(t)$", fontsize=9.5, ha='center')

    # Controller block
    ctrl_box = patches.FancyBboxPatch((3.0, 1.0), 2.2, 1.0, boxstyle="round,pad=0.1",
                                      facecolor='#e8f8f5', edgecolor='#16a085', lw=2)
    ax.add_patch(ctrl_box)
    ax.text(4.1, 1.5, "PID Controller\n$C(s) = K_p + \\frac{K_i}{s} + \\frac{K_d s}{1+s/N}$",
            fontsize=9.5, ha='center', va='center', fontweight='bold', color='#16a085')

    # Control effort signal
    ax.annotate("", xy=(6.2, 1.5), xytext=(5.2, 1.5),
                arrowprops=dict(arrowstyle="->", color='#2c3e50', lw=2))
    ax.text(5.7, 1.7, "Effort $V_a(t)$", fontsize=9.5, ha='center')

    # Plant block
    plant_box = patches.FancyBboxPatch((6.2, 1.0), 3.0, 1.0, boxstyle="round,pad=0.1",
                                       facecolor='#fef5e7', edgecolor='#d35400', lw=2)
    ax.add_patch(plant_box)
    ax.text(7.7, 1.5, "DC Motor Plant $P(s)$\n$\\frac{K}{(Js+b)(Ls+R) + K^2}$",
            fontsize=9.5, ha='center', va='center', fontweight='bold', color='#d35400')

    # Output signal
    ax.plot([9.2, 11.0], [1.5, 1.5], color='#2c3e50', lw=2)
    ax.annotate("", xy=(11.1, 1.5), xytext=(10.5, 1.5),
                arrowprops=dict(arrowstyle="->", color='#2c3e50', lw=2))
    ax.scatter([10.2], [1.5], color='#2c3e50', s=35, zorder=5)
    ax.text(10.7, 1.75, "Output $y(t)$", fontsize=10, ha='center', fontweight='bold')

    # Feedback path
    ax.plot([10.2, 10.2], [1.5, 0.6], color='#2c3e50', lw=2)
    ax.plot([10.2, 2.1], [0.6, 0.6], color='#2c3e50', lw=2)
    ax.annotate("", xy=(2.1, 1.25), xytext=(2.1, 0.6),
                arrowprops=dict(arrowstyle="->", color='#2c3e50', lw=2))
    ax.text(6.0, 0.75, "Unity Negative Feedback ($H(s) = 1$)", fontsize=9, ha='center', color='#7f8c8d')

    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()
    print(f"Schematic successfully generated at: {save_path}")

if __name__ == '__main__':
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    assets_dir = os.path.join(repo_root, "assets")
    os.makedirs(assets_dir, exist_ok=True)
    draw_system_schematic(os.path.join(assets_dir, 'dc_motor_schematic.png'))
    draw_system_schematic(os.path.join(assets_dir, 'dc_motor_schematic.svg'))
