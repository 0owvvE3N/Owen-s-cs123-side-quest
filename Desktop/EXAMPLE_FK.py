import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import List, Tuple

class Vector:
    """2D vector with homogeneous representation."""
    def __init__(self, x: float, y: float) -> None:
        self.x: float = x
        self.y: float = y
        self.h: np.ndarray = np.array([[x], [y], [1]])  # homogeneous column vector

    def __repr__(self) -> str:
        return f"Vector(x={self.x:.2f}, y={self.y:.2f})"

def from_array(arr: np.ndarray) -> "Vector":
    if arr.shape[0] == 2:
        return Vector(arr[0][0],arr[1][0])
    elif arr.shape[0] == 3:
        return Vector(arr[0][0], arr[1][0])
    else:
        return ValueError("Invalid array shape for Vector")
        

def transMat(dx: float, dy: float) -> np.ndarray:
    """2D homogeneous translation matrix."""
    return np.array([
        [1, 0, dx],
        [0, 1, dy],
        [0, 0, 1]
    ])


def rotMat(theta: float) -> np.ndarray:
    """2D homogeneous rotation matrix (about origin)."""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])



# Storage for animation frames
frames_x: List[List[float]] = []
frames_y: List[List[float]] = []

def kine(x0: float, y0: float, l1: float, l2: float,
         theta1: float, theta2: float,
         frames_x: List[List[float]], frames_y: List[List[float]]) -> None:
    """
    Compute step-by-step forward kinematics, recording each frame.
    """
    # Start at world base
    base = Vector(0, l2)
    frames_x.append([0,base.x])
    frames_y.append([0,base.y])
    # 2
    R2 = rotMat(theta2) @ base.h
    print(R2)
    print(R2.shape[0])
    R2_vec = from_array(R2)
    frames_x.append([0,R2_vec.x])
    frames_y.append([0,R2_vec.y])

    # 3
    L1 = Vector(0,l1)
    T2 = transMat(0,l1) @ R2
    T2_vec = from_array(T2)
    frames_x.append([0,L1.x, T2_vec.x])
    frames_y.append([0,L1.y,T2_vec.y])

    # 4
    L1_rot1 = rotMat(theta1) @ L1.h
    L1_rot1_vec = from_array(L1_rot1)
    
    T2_rot1 = rotMat(theta1) @ T2
    T2_rot1_vec = from_array(T2_rot1)
    frames_x.append([0,L1_rot1_vec.x,T2_rot1_vec.x])
    frames_y.append([0,L1_rot1_vec.y,T2_rot1_vec.y])

    # 5 Shift by x0 y0 wrld coord
    adjL1 = transMat(x0,y0) @ L1_rot1
    adjL1_vec = from_array(adjL1)
    adjT2 = transMat(x0,y0) @ T2_rot1 
    adjT2_vec = from_array(adjT2)

    frames_x.append([x0, adjL1_vec.x, adjT2_vec.x])
    frames_y.append([y0, adjL1_vec.y, adjT2_vec.y])



# Example usage
kine(x0=-1, y0=1, l1=2.0, l2=1.5, theta1= (-3 * np.pi)/4, theta2=-np.pi/2,
     frames_x=frames_x, frames_y=frames_y)


# --- Plotting / animation ---
fig, ax = plt.subplots()
(line,) = ax.plot([], [], 'o-', lw=3, markersize=8, color='tab:blue')
ax.set_xlim(-3, 3)
ax.set_ylim(-4, 2)
ax.set_aspect("equal", adjustable="box")

def init():
    line.set_data([], [])
    return (line,)

def update(frame: int):
    line.set_data(frames_x[frame], frames_y[frame])
    return (line,)

ani = animation.FuncAnimation(
    fig, update, frames=len(frames_x), init_func=init,
    blit=True, interval=1000, repeat=True
)

plt.show()
