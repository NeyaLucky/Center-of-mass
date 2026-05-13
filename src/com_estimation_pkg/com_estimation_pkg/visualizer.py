import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from typing import Tuple, Optional, List
from dataclasses import dataclass, field


@dataclass
class CoMEstimate:
    position: Tuple[float, float, float]  
    mass: float
    uncertainty: Tuple[float, float, float]
    confidence: float


@dataclass
class ObjectInfo:
    """Object properties derived from URDF / pybullet."""
    name: str = "Unknown"
    size: Tuple[float, float, float] = (0.05, 0.05, 0.05)
    color: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)
    actual_com: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    attachment_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mass: float = 0.0
    urdf_file: str = ""
    mesh_filename: str = ""


class CoMVisualizer:
    def __init__(self, figsize: Tuple[int, int] = (14, 10)):
        self.figsize = figsize
        self.fig = None
        self.ax = None

    def visualize(
        self,
        com_estimate: CoMEstimate,
        object_info: ObjectInfo,
        eef_position: Tuple[float, float, float] = (0, 0, 0),
        title: str = "Center of Mass Estimation",
        save_path: str = None,
        show: bool = True,
        ax=None,
    ):
        if ax is not None:
            self.ax = ax
            self.fig = ax.figure
        else:
            self.fig = plt.figure(figsize=self.figsize)
            self.ax = self.fig.add_subplot(111, projection='3d')

        obj_origin = tuple(
            eef_position[i] + object_info.attachment_offset[i] for i in range(3)
        )

        obj_color = object_info.color[:3]
        obj_alpha = min(object_info.color[3], 0.35) if len(object_info.color) > 3 else 0.3
        edge_color = tuple(max(c - 0.25, 0.0) for c in obj_color)
        self._draw_cube(
            center=obj_origin,
            size=object_info.size,
            facecolor=obj_color,
            edgecolor=edge_color,
            alpha=obj_alpha,
            label=object_info.name,
        )

        actual_com_global = tuple(
            obj_origin[i] + object_info.actual_com[i] for i in range(3)
        )
        self.ax.scatter(
            *[[v] for v in actual_com_global],
            color='dodgerblue', s=220, marker='D',
            edgecolors='navy', linewidths=2, zorder=12,
            label='Real CoM(URDF)',
        )

        est_com_global = tuple(
            eef_position[i] + com_estimate.position[i] for i in range(3)
        )
        self.ax.scatter(
            *[[v] for v in est_com_global],
            color='red', s=220, marker='o',
            edgecolors='darkred', linewidths=2, zorder=12,
            label='Estimated CoM',
        )

        self._draw_uncertainty(est_com_global, com_estimate.uncertainty)

        self.ax.scatter(
            *[[v] for v in eef_position],
            color='limegreen', s=200, marker='^',
            edgecolors='darkgreen', linewidths=2, zorder=12,
            label='End-effector',
        )

        self.ax.plot(
            [eef_position[0], est_com_global[0]],
            [eef_position[1], est_com_global[1]],
            [eef_position[2], est_com_global[2]],
            'r--', linewidth=1.5, alpha=0.6,
        )
        self.ax.plot(
            [eef_position[0], actual_com_global[0]],
            [eef_position[1], actual_com_global[1]],
            [eef_position[2], actual_com_global[2]],
            color='dodgerblue', linestyle='--', linewidth=1.5, alpha=0.6,
        )

        axis_len = max(object_info.size) * 1.5
        self._draw_axes(eef_position, axis_len)

        all_points = np.array([eef_position, est_com_global, actual_com_global, obj_origin])
        half_size = np.array(object_info.size) / 2
        pts_min = all_points.min(axis=0) - half_size - 0.02
        pts_max = all_points.max(axis=0) + half_size + 0.02
        center = (pts_min + pts_max) / 2
        max_range = (pts_max - pts_min).max() / 2
        max_range = max(max_range, max(object_info.size))  
        self.ax.set_xlim([center[0] - max_range, center[0] + max_range])
        self.ax.set_ylim([center[1] - max_range, center[1] + max_range])
        self.ax.set_zlim([center[2] - max_range, center[2] + max_range])

        self.ax.set_xlabel('X [м]', fontsize=12)
        self.ax.set_ylabel('Y [м]', fontsize=12)
        self.ax.set_zlabel('Z [м]', fontsize=12)
        self.ax.legend(loc='upper right', fontsize=9)

        com_err = np.linalg.norm(
            np.array(est_com_global) - np.array(actual_com_global)
        )
        info_lines = [
            f"Object: {object_info.name}  ({object_info.urdf_file})",
            f"Size: {object_info.size[0]*1e3:.1f}×{object_info.size[1]*1e3:.1f}×{object_info.size[2]*1e3:.1f} mm",
            f"Mass (estimated): {com_estimate.mass:.4f} kg",
            f"Mass (URDF):    {object_info.mass:.4f} kg" if object_info.mass else "",
            f"CoM estimated: ({com_estimate.position[0]:.4f}, {com_estimate.position[1]:.4f}, {com_estimate.position[2]:.4f}) m",
            f"CoM real:  ({object_info.actual_com[0]:.4f}, {object_info.actual_com[1]:.4f}, {object_info.actual_com[2]:.4f}) m",
            f"Error CoM: {com_err*1e3:.2f} mm",
            f"Uncertainty: ±({com_estimate.uncertainty[0]:.4f}, {com_estimate.uncertainty[1]:.4f}, {com_estimate.uncertainty[2]:.4f}) m",
            f"Confidence: {com_estimate.confidence*100:.1f}%",
        ]
        info_text = "\n".join(line for line in info_lines if line)
        self.ax.text2D(
            0.02, 0.98, info_text, transform=self.ax.transAxes,
            fontsize=9, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6),
        )

        self.ax.set_title(title, fontsize=14, fontweight='bold')
        plt.tight_layout()

        if save_path:
            self.fig.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Visualization saved: {save_path}")
        if show:
            plt.show()

    def visualize_comparison(
        self,
        results: dict,                 
        object_info: ObjectInfo,
        eef_position: Tuple[float, float, float] = (0, 0, 0),
        save_path: str = None,
        show: bool = False,
    ):
        """
        Grid figure: one 3D subplot per method, all on the same scale.
        results: ordered dict {method_label: CoMEstimate}
        """
        n = len(results)
        ncols = min(n, 3)
        nrows = (n + ncols - 1) // ncols

        fig = plt.figure(figsize=(ncols * 6, nrows * 5 + 1))
        fig.suptitle(
            f"CoM Estimation — Method Comparison\n"
            f"Object: {object_info.name}  |  True mass: {object_info.mass:.3f} kg",
            fontsize=13, fontweight='bold', y=1.01,
        )
        obj_origin = tuple(
            eef_position[i] + object_info.attachment_offset[i] for i in range(3)
        )
        actual_com_global = tuple(
            obj_origin[i] + object_info.actual_com[i] for i in range(3)
        )
        all_pts = np.array([eef_position, obj_origin, actual_com_global]
                           + [tuple(eef_position[i] + r.position[i] for i in range(3))
                              for r in results.values()])
        half = np.array(object_info.size) / 2
        pts_min = all_pts.min(axis=0) - half - 0.02
        pts_max = all_pts.max(axis=0) + half + 0.02
        center = (pts_min + pts_max) / 2
        max_range = max((pts_max - pts_min).max() / 2, max(object_info.size))

        axes = []
        for idx, (label, estimate) in enumerate(results.items()):
            ax = fig.add_subplot(nrows, ncols, idx + 1, projection='3d')
            axes.append(ax)

            self.visualize(
                com_estimate=estimate,
                object_info=object_info,
                eef_position=eef_position,
                title="",
                save_path=None,
                show=False,
                ax=ax,
            )

            mass_err = ""
            if object_info.mass:
                pct = abs(estimate.mass - object_info.mass) / object_info.mass * 100
                mass_err = f"  err {pct:.1f}%"
            com_err_mm = np.linalg.norm(
                np.array([eef_position[i] + estimate.position[i] for i in range(3)])
                - np.array(actual_com_global)
            ) * 1e3
            ax.set_title(
                f"{label}\nm={estimate.mass:.3f} kg{mass_err}\n"
                f"CoM err {com_err_mm:.1f} mm  conf {estimate.confidence:.2f}",
                fontsize=9,
            )

            ax.set_xlim([center[0] - max_range, center[0] + max_range])
            ax.set_ylim([center[1] - max_range, center[1] + max_range])
            ax.set_zlim([center[2] - max_range, center[2] + max_range])
            ax.legend(fontsize=7, loc='upper right')

        for idx in range(n, nrows * ncols):
            fig.add_subplot(nrows, ncols, idx + 1).set_visible(False)

        plt.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Comparison saved: {save_path}")
        if show:
            plt.show()
        plt.close(fig)

    def _draw_cube(
        self,
        center: Tuple[float, float, float],
        size: Tuple[float, float, float],
        facecolor=(0.2, 0.2, 0.8),
        edgecolor=(0.1, 0.1, 0.4),
        alpha: float = 0.3,
        label: str = None,
    ):
        dx, dy, dz = size
        x, y, z = center

        v = np.array([
            [x - dx/2, y - dy/2, z - dz/2],
            [x + dx/2, y - dy/2, z - dz/2],
            [x + dx/2, y + dy/2, z - dz/2],
            [x - dx/2, y + dy/2, z - dz/2],
            [x - dx/2, y - dy/2, z + dz/2],
            [x + dx/2, y - dy/2, z + dz/2],
            [x + dx/2, y + dy/2, z + dz/2],
            [x - dx/2, y + dy/2, z + dz/2],
        ])

        faces = [
            [v[0], v[1], v[2], v[3]],
            [v[4], v[5], v[6], v[7]],
            [v[0], v[1], v[5], v[4]],
            [v[2], v[3], v[7], v[6]],
            [v[0], v[3], v[7], v[4]],
            [v[1], v[2], v[6], v[5]],
        ]

        collection = Poly3DCollection(
            faces, alpha=alpha, facecolor=facecolor,
            edgecolor=edgecolor, linewidths=1,
        )
        self.ax.add_collection3d(collection)

        if label:
            self.ax.scatter([], [], [], color=facecolor, label=label, s=60, marker='s')

    def _draw_uncertainty(
        self,
        center: Tuple[float, float, float],
        uncertainty: Tuple[float, float, float],
        scale: float = 3.0,
        n: int = 25,
    ):
        radii = tuple(max(u * scale, 0.003) for u in uncertainty)
        u = np.linspace(0, 2 * np.pi, n)
        v = np.linspace(0, np.pi, n)
        ex = radii[0] * np.outer(np.cos(u), np.sin(v)) + center[0]
        ey = radii[1] * np.outer(np.sin(u), np.sin(v)) + center[1]
        ez = radii[2] * np.outer(np.ones_like(u), np.cos(v)) + center[2]
        self.ax.plot_surface(ex, ey, ez, color='red', alpha=0.12, linewidth=0)
        self.ax.plot_wireframe(ex, ey, ez, color='red', alpha=0.20, linewidth=0.4)

    def _draw_axes(self, origin: Tuple[float, float, float], length: float):
        x, y, z = origin
        self.ax.quiver(x, y, z, length, 0, 0, color='red',
                       arrow_length_ratio=0.1, linewidth=2)
        self.ax.text(x + length * 1.15, y, z, 'X', color='red', fontsize=11)
        self.ax.quiver(x, y, z, 0, length, 0, color='green',
                       arrow_length_ratio=0.1, linewidth=2)
        self.ax.text(x, y + length * 1.15, z, 'Y', color='green', fontsize=11)
        self.ax.quiver(x, y, z, 0, 0, length, color='blue',
                       arrow_length_ratio=0.1, linewidth=2)
        self.ax.text(x, y, z + length * 1.15, 'Z', color='blue', fontsize=11)

    def show(self):
        plt.show()

    def save(self, filename: str, dpi: int = 150):
        if self.fig:
            self.fig.savefig(filename, dpi=dpi, bbox_inches='tight')
            print(f"Saved: {filename}")

    def close(self):
        if self.fig:
            plt.close(self.fig)


def visualize_torque_influence(
    delta_torques_matrix: np.ndarray,
    joint_names: List[str] = None,
    position_names: List[str] = None,
    title: str = "Torque Influence Heatmap",
):
    if joint_names is None:
        joint_names = [f'joint{i+1}' for i in range(delta_torques_matrix.shape[1])]
    if position_names is None:
        position_names = [f'pos{i+1}' for i in range(delta_torques_matrix.shape[0])]

    fig, ax = plt.subplots(figsize=(12, 6))
    im = ax.imshow(delta_torques_matrix, cmap='RdBu_r', aspect='auto')

    ax.set_xticks(range(len(joint_names)))
    ax.set_xticklabels(joint_names)
    ax.set_yticks(range(len(position_names)))
    ax.set_yticklabels(position_names)

    for i in range(len(position_names)):
        for j in range(len(joint_names)):
            value = delta_torques_matrix[i, j]
            color = 'white' if abs(value) > 1.0 else 'black'
            ax.text(j, i, f'{value:.2f}', ha='center', va='center',
                    color=color, fontsize=9)

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Δτ [Nm]', fontsize=12)
    ax.set_xlabel('Joints', fontsize=12)
    ax.set_ylabel('Positions', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    return fig


def demo_visualization():
    com_estimate = CoMEstimate(
        position=(0.01, -0.005, 0.025),
        mass=0.5,
        uncertainty=(0.008, 0.008, 0.01),
        confidence=0.85,
    )
    obj = ObjectInfo(
        name='Demo Cube',
        size=(0.05, 0.05, 0.05),
        color=(0.2, 0.2, 0.8, 1.0),
        actual_com=(0.0, 0.0, 0.0),
        mass=0.5,
        urdf_file='demo.urdf',
    )
    visualizer = CoMVisualizer()
    visualizer.visualize(
        com_estimate,
        object_info=obj,
        eef_position=(0, 0, 0),
        title='Object Center of Mass (Demo)',
    )
    visualizer.show()


if __name__ == '__main__':
    demo_visualization()
