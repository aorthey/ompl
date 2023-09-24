import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

coords = np.loadtxt('multi_robot_disk_path.dat')
#coords = np.loadtxt('../../build/multi_robot_disk_path_prioritization.dat')

robot_radius = 0.15
padding = robot_radius + 0.05
path_line_width = 5
number_iterations = 100
alpha_start_goal_disk = 0.2

fontsize = 20

N = int(coords.shape[1] / 2)
num_pts = int(coords.shape[0])

fig, ax = plt.subplots()
ax.yaxis.get_label().set_fontsize(fontsize)
ax.xaxis.get_label().set_fontsize(fontsize)
ax.set_ylabel('Y', rotation=0)
ax.set_xlabel('X', rotation=0)
ax.tick_params(axis='both', which='major', labelsize=fontsize)

ax.set_xlim(-1-padding, 1+padding)
ax.set_ylim(-1-padding, 1+padding)
fig.tight_layout()
ax.set_aspect('equal', 'box')

circle_radius = 1
circle = plt.Circle((0, 0), circle_radius, color='gray', alpha=0.2)
circle_edge = plt.Circle((0, 0), circle_radius, fill=False, edgecolor='black', linewidth=5, alpha=1)
ax.add_patch(circle)
ax.add_patch(circle_edge)

colors = plt.cm.rainbow(np.linspace(0, 1, N))
disks = [plt.Circle((0, 0), robot_radius, fc=colors[i], alpha=1, zorder=10) for i in range(N)]
for disk in disks:
  ax.add_patch(disk)

disks_transparent_start = [plt.Circle((0, 0), robot_radius, fc=colors[i], alpha=alpha_start_goal_disk) for i in range(N)]
for disk in disks_transparent_start:
  ax.add_patch(disk)
disks_transparent_goal = [plt.Circle((0, 0), robot_radius, fc=colors[i], alpha=alpha_start_goal_disk) for i in range(N)]
for disk in disks_transparent_goal:
  ax.add_patch(disk)

for i in range(N):
  x = coords[:, 2*i]
  y = coords[:, 2*i+1]
  ax.plot(x, y, color=colors[i], linewidth=path_line_width)
  disks_transparent_goal[i].center = (x[-1], y[-1])
  disks_transparent_start[i].center = (x[0], y[0])


interpolated_paths = [np.empty((0, 2)) for _ in range(N)]

# Perform cubic spline interpolation for each path
for i in range(N):
  x = coords[:, 2*i]
  y = coords[:, 2*i+1]
  t = np.arange(num_pts)
  t_new = np.linspace(0, num_pts, number_iterations)  # Adjust the number of interpolated points
  x_new = np.interp(t_new, t, x)
  y_new = np.interp(t_new, t, y)
  interpolated_paths[i] = np.column_stack((x_new, y_new))

total_frames = len(interpolated_paths[0])

def update(frame):
  global frame_number
  frame_number = frame
  for i in range(N):
    x, y = interpolated_paths[i][frame]
    disks[i].center = (x, y)
    print("Frame %d/%d" % (frame, total_frames))
  return disks

anim = FuncAnimation(fig, update, frames=total_frames, blit=True, repeat=False)

#Repeat animation on key press event (space)
def on_key(event):
  global frame_number, anim
  if event.key == ' ':
    if frame_number >= total_frames - 1:
      anim = FuncAnimation(fig, update, frames=total_frames, blit=True, repeat=False)
      frame_number = 0
    if anim.event_source.interval > 0:
      anim.pause()
      anim.event_source.interval = 0
    else:
      anim.resume()

# Connect the key event handler function
fig.canvas.mpl_connect('key_press_event', on_key)
plt.show()

# f = r"/home/aorthey/Videos/animation.mp4"
# writervideo = animation.FFMpegWriter(fps=30)
# anim.save(f, writer=writervideo)
