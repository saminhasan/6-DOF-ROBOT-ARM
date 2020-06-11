import numpy as np



class Armlinkage:
	def __init__(self, length, previous_link = None):
		self.length = length
		if previous_link:
			self.previous_link = previous_link
			self.initial_position =   np.array([self.length, 0.0, 0.0]) #+ previous_link.initial_position # x,y,z of tip 
			self.initial_orientation =  np.array([0.0, 0.0, 0.0])  #+ previous_link.initial_orientation # roll,pitch,yaw

		else:
			self.previous_link = None
			self.initial_position = np.array([self.length, 0.0, 0.0]) # x,y,z base 
			self.initial_orientation = np.array([0.0, 0.0, 0.0]) # roll,pitch,yaw
		self.current_position = self.initial_position 
		self.current_orientation = self.initial_orientation
		self.update_pose()
		
	def rotation_matrix(self, roll, pitch, yaw):
		#roll,pitch,yaw = np.radians(roll),np.radians(pitch), np.radians(yaw)
		rot_mat = np.array([
		[(np.cos(yaw) * np.cos(pitch)), (np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll)), (np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll))],
		[(np.sin(yaw) * np.cos(pitch)), (np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll)), (np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll))],
		[(-np.sin(pitch))			  ,	 (np.cos(pitch) * np.sin(roll))										 , (np.cos(pitch) * np.cos(roll))]
		])
		return rot_mat	
		
		
	def update_pose(self,orientation = np.array([0.0, 0.0, 0.0])):
		if self.previous_link:
			self.current_orientation = orientation + self.previous_link.current_orientation
		else:
			self.current_orientation = orientation 
			
		link_n = self.initial_position
		R, P, Y = self.current_orientation 
		yaw_matrix = self.rotation_matrix(0.0, 0.0, Y)
		link_n = np.dot(yaw_matrix,link_n)
		pitch_matrix = self.rotation_matrix(0.0, P, 0.0)
		link_n = np.dot(pitch_matrix,link_n)
		roll_matrix = self.rotation_matrix(R, 0.0 , 0.0)
		link_n = np.dot(roll_matrix,link_n)

		if self.previous_link:
			self.current_position = link_n + self.previous_link.current_position
		else:
			self.current_position = link_n 

class Arm:

				
	def __init__(self,arm_segment_length):
		self.arm_links = np.empty(shape=(len(arm_segment_length),),dtype=object)
		self.positions = np.zeros(shape=(len(self.arm_links),3))
		self.orientations = np.zeros(shape=(len(self.arm_links),3))
		#self.arm_links = []
		self.arm_links[0] =  Armlinkage(arm_segment_length[0])

		#self.arm_links.append(Armlinkage(arm_segment_length[0]))
		for i in range(1,len(arm_segment_length)):
		
			self.arm_links[i] = Armlinkage(arm_segment_length[i],self.arm_links[i-1])
			self.positions[i] = self.arm_links[i].current_position
			self.orientations[i] = self.arm_links[i].current_orientation
			#self.arm_links.append(Armlinkage(arm_segment_length[i],arm_segment_length[i-1]))

		#self.forward_kinematics(self.orientations)

	def forward_kinematics(self,orientation):
		for i in range(len(orientation)):
			self.arm_links[i].update_pose(orientation[i])
			self.positions[i] = self.arm_links[i].current_position
			self.orientations[i] = self.arm_links[i].current_orientation

			
def rotation_matrix(roll, pitch, yaw):
	#roll,pitch,yaw = np.radians(roll),np.radians(pitch), np.radians(yaw)
	rot_mat = np.array([
	[(np.cos(yaw) * np.cos(pitch)), (np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll)), (np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll))],
	[(np.sin(yaw) * np.cos(pitch)), (np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll)), (np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll))],
	[(-np.sin(pitch))			  ,	 (np.cos(pitch) * np.sin(roll))										 , (np.cos(pitch) * np.cos(roll))]
	])
	return rot_mat	
			
	
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
#plt.style.use('dark_background')
plt.style.use('fast')
#fig = plt.figure("Arm")
#fig = plt.subplot(1, 2, 1,projection='3d')
fig = plt.figure(figsize=plt.figaspect(0.5))
#ax = Axes3D(fig)
ax = fig.add_subplot(1, 2, 1, projection='3d')
lengths = np.array([3.0,2.0,1.5])
Robot = Arm(lengths)
orientation1 = np.array([0.0,0.0,0.0])
orientation2 = np.array([0.0,0.0,0.0])
orientation3 = np.array([0.0,0.0,0.0])
O = np.vstack((orientation1,orientation2,orientation3))
Robot.forward_kinematics(O)
plot_lim = 10.0
ax.grid(True)
ax.set_xlim(-plot_lim, plot_lim)
ax.set_ylim(-plot_lim, plot_lim)
ax.set_zlim(-plot_lim, plot_lim)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.scatter(0,0,0)
ax.plot((0,Robot.positions[0][0]),(0,Robot.positions[0][1]),(0,Robot.positions[0][2]))
ax.scatter((0,Robot.positions[0][0]),(0,Robot.positions[0][1]),(0,Robot.positions[0][2]))

ax.plot((Robot.positions[0][0],Robot.positions[1][0]),(Robot.positions[0][1],Robot.positions[1][1]),(Robot.positions[0][2],Robot.positions[1][2]))
ax.scatter((Robot.positions[0][0],Robot.positions[1][0]),(Robot.positions[0][1],Robot.positions[1][1]),(Robot.positions[0][2],Robot.positions[1][2]))


ax.plot((Robot.positions[1][0],Robot.positions[2][0]),(Robot.positions[1][1],Robot.positions[2][1]),(Robot.positions[2][2],Robot.positions[2][2]))
ax.scatter((Robot.positions[1][0],Robot.positions[2][0]),(Robot.positions[1][1],Robot.positions[2][1]),(Robot.positions[2][2],Robot.positions[2][2]))

#ax.scatter(Robot.positions[:,0],Robot.positions[:,1],Robot.positions[:,2])
#ax.plot(Robot.positions[:,0],Robot.positions[:,1],Robot.positions[:,2])

V1 = np.array([[1,0,0],[0,1,0],[0,0,1]])
origin = np.array([[0],[0],[0]]) # origin point
rot_mat1 = rotation_matrix(-Robot.orientations[0][0],-Robot.orientations[0][1],-Robot.orientations[0][2])
V1 = np.dot(rot_mat1,V1)
ax.quiver(*origin, *V1[0], color=['r'])
ax.quiver(*origin, *V1[1], color=['g'])
ax.quiver(*origin, *V1[2], color=['b'])
V2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
origin2 = np.array([[Robot.positions[0][0]],[Robot.positions[0][1]],[Robot.positions[0][2]]]) # origin point
rot_mat2 = rotation_matrix(-Robot.orientations[1][0],-Robot.orientations[1][1],-Robot.orientations[1][2])
V2 = np.dot(rot_mat2,V2)
ax.quiver(*origin2, *V2[0], color=['r'])
ax.quiver(*origin2, *V2[1], color=['g'])
ax.quiver(*origin2, *V2[2], color=['b'])
V3 = np.array([[1,0,0],[0,1,0],[0,0,1]])
origin3 = np.array([[Robot.positions[1][0]],[Robot.positions[1][1]],[Robot.positions[1][2]]]) # origin point
rot_mat3 = rotation_matrix(-Robot.orientations[2][0],-Robot.orientations[2][1],-Robot.orientations[2][2])
V3 = np.dot(rot_mat3,V3)
ax.quiver(*origin3, *V3[0], color=['r'])
ax.quiver(*origin3, *V3[1], color=['g'])
ax.quiver(*origin3, *V3[2], color=['b'])
def animate(i):
	global Robot
	ax.clear()
	plot_lim = 10.0
	ax.grid(True)
	ax.set_xlim(-plot_lim, plot_lim)
	ax.set_ylim(-plot_lim, plot_lim)
	ax.set_zlim(-plot_lim, plot_lim)
	ax.set_xlabel("X")
	ax.set_ylabel("Y")
	ax.set_zlabel("Z")
	ax.scatter(0,0,0)
	ax.plot((0,Robot.positions[0][0]),(0,Robot.positions[0][1]),(0,Robot.positions[0][2]))
	ax.scatter(Robot.positions[:,0],Robot.positions[:,1],Robot.positions[:,2])
	ax.plot(Robot.positions[:,0],Robot.positions[:,1],Robot.positions[:,2])
	V1 = np.array([[1,0,0],[0,1,0],[0,0,1]])
	origin = np.array([[0],[0],[0]]) # origin point
	rot_mat1 = rotation_matrix(-Robot.orientations[0][0],-Robot.orientations[0][1],-Robot.orientations[0][2])
	V1 = np.dot(rot_mat1,V1)
	ax.quiver(*origin, *V1[0], color=['r'])
	ax.quiver(*origin, *V1[1], color=['g'])
	ax.quiver(*origin, *V1[2], color=['b'])
	V2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
	origin2 = np.array([[Robot.positions[0][0]],[Robot.positions[0][1]],[Robot.positions[0][2]]]) # origin point
	rot_mat2 = rotation_matrix(-Robot.orientations[1][0],-Robot.orientations[1][1],-Robot.orientations[1][2])
	V2 = np.dot(rot_mat2,V2)
	ax.quiver(*origin2, *V2[0], color=['r'])
	ax.quiver(*origin2, *V2[1], color=['g'])
	ax.quiver(*origin2, *V2[2], color=['b'])
	V3 = np.array([[1,0,0],[0,1,0],[0,0,1]])
	origin3 = np.array([[Robot.positions[1][0]],[Robot.positions[1][1]],[Robot.positions[1][2]]]) # origin point
	rot_mat3 = rotation_matrix(-Robot.orientations[2][0],-Robot.orientations[2][1],-Robot.orientations[2][2])
	V3 = np.dot(rot_mat3,V3)
	ax.quiver(*origin3, *V3[0], color=['r'])
	ax.quiver(*origin3, *V3[1], color=['g'])
	ax.quiver(*origin3, *V3[2], color=['b'])







ani = animation.FuncAnimation(fig, animate, interval=1)


#fig = plt.figure("slider")
#fig = plt.subplot(1, 2, 2)
#ax = fig.add_subplot(1, 2, 2)#, projection='3d')
#plt.subplots_adjust(left=0.5)
R1 = plt.axes([0.55, 0.1, 0.4, 0.05])
P1 = plt.axes([0.55, 0.2, 0.4, 0.05])
Y1 = plt.axes([0.55, 0.3, 0.4, 0.05])
R2 = plt.axes([0.55, 0.4, 0.4, 0.05])
P2 = plt.axes([0.55, 0.5, 0.4, 0.05])
Y2 = plt.axes([0.55, 0.6, 0.4, 0.05])
R3 = plt.axes([0.55, 0.7, 0.4, 0.05])
P3 = plt.axes([0.55, 0.8, 0.4, 0.05])
Y3 = plt.axes([0.55, 0.9, 0.4, 0.05])
roll1_silder = Slider(R1, 'roll_1', 0.0,2.0*np.pi, valinit=0,valstep = 0.1)
pitch1_silder = Slider(P1, 'pitch_1', 0.0, 2.0*np.pi, valinit=0,valstep = 0.1)
yaw1_silder = Slider(Y1, 'yaw_1', 0.0, 2.0*np.pi, valinit=0,valstep = 0.1)
roll2_silder = Slider(R2, 'roll_2', 0.0,2.0*np.pi, valinit=0,valstep = 0.1)
pitch2_silder = Slider(P2, 'pitch_2',0.0, 2.0*np.pi, valinit=0,valstep = 0.1)
yaw2_silder = Slider(Y2, 'yaw_2', 0.0, 2.0*np.pi, valinit=0,valstep = 0.1)
roll3_silder = Slider(R3, 'roll_3', 0.0, 2.0*np.pi, valinit=0,valstep = 0.1)
pitch3_silder = Slider(P3, 'pitch_3', 0.0, 2.0*np.pi, valinit=0,valstep = 0.1)
yaw3_silder = Slider(Y3, 'yaw_3', 0.0, 2.0*np.pi, valinit=0,valstep = 0.1)

def update_slider(i):
	global Robot
	roll1 = roll1_silder.val
	pitch1 = pitch1_silder.val
	yaw1 = yaw1_silder.val
	roll2 = roll2_silder.val
	pitch2 = pitch2_silder.val
	yaw2 = yaw2_silder.val
	roll3 = roll3_silder.val
	pitch3 = pitch3_silder.val
	yaw3 = yaw3_silder.val
	orientation1 = np.array([roll1,pitch1,yaw1])
	orientation2 = np.array([roll2,pitch2,yaw2])
	orientation3 = np.array([roll3,pitch3,yaw3])
	O = np.vstack((orientation1,orientation2,orientation3))
	Robot.forward_kinematics(O)

	

	
roll1_silder.on_changed(update_slider)
pitch1_silder.on_changed(update_slider)
yaw1_silder.on_changed(update_slider)
roll2_silder.on_changed(update_slider)
pitch2_silder.on_changed(update_slider)
yaw2_silder.on_changed(update_slider)
roll3_silder.on_changed(update_slider)
pitch3_silder.on_changed(update_slider)
yaw3_silder.on_changed(update_slider)

#fig.tight_layout()
plt.show()
