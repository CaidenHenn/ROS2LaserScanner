import rclpy
import math
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SimpleLaserDrive(Node):
	def __init__(self):
		super().__init__("laserdrive")
		self.pub = self.create_publisher(Twist, "cmd_vel", 10)
		self.sub = self.create_subscription(LaserScan, "scan", self.laser_callback, 10)
		self.timer = self.create_timer(.05, self.move)  # 1 Hz
		self.laserdata = None
		self.min_angle = None
		self.max_angle = None
		self.delta_angle = None
		self.maxrange = None
		self.desired_distance = 3.0
		self.angular_z = 0.0
		self.state = 1
		
		


	def laser_callback(self, data):
		self.min_angle = data.angle_min
		self.max_angle = data.angle_max
		self.delta_angle = data.angle_increment
		self.maxrange = data.range_max
		self.laserdata = data.ranges

		for i in range(0, len(self.laserdata)):
			if math.isinf(self.laserdata[i]):
				self.laserdata[i] = self.maxrange

		
		
	
	def rightwallfollow(self):
		msg = Twist()
		angular = 0.0
		if self.laserdata:
		#find distance to left wall
		#if the left distance > desired distanced, then turn left
		#if left distance < desired distance, turn right
			
		#min distance to left wall:
			index_min_angle = self.angletoindex(self.min_angle)
			piover2 = self.angletoindex(-math.pi/4)
			subset = self.laserdata[index_min_angle:piover2]
			if subset:
				disleftwall = max(subset)

				if disleftwall == self.desired_distance:
					angular= 0.5
				
				elif(disleftwall > self.desired_distance):
					angular= -1.3
				
				elif(disleftwall < self.desired_distance):
					angular = 2.0
					
			
		return angular
	def leftwallfollow(self):
		msg = Twist()
		angular = 0.0
		if self.laserdata:
			index_min_angle = self.angletoindex(self.max_angle)
			piover2 = self.angletoindex(math.pi/4)
			subset = self.laserdata[piover2:index_min_angle]
			
			if subset:
				disrightwall = max(subset)

				if disrightwall == self.desired_distance:
					angular= -0.5
				
				elif(disrightwall > self.desired_distance):
					angular= 1.3
	
				elif(disrightwall < self.desired_distance):
					angular = -2.0
		return angular
		
		
	def vector_lat(self):
		lat = 0.0
		for i in range(0, len(self.laserdata)):
			lat += (self.laserdata[i] / self.maxrange) * math.sin(self.indextoangle(i)) 
		
		lat /= len(self.laserdata)
		return lat * 6

	def vector_fow(self):
		forward = 0.0
		for i in range(0, len(self.laserdata)):
			forward += (self.laserdata[i] / self.maxrange) * math.cos(self.indextoangle(i))
		forward /= len(self.laserdata) * 1.2
		forward = forward
		return forward*8
	
	def distance_to_wall(self):
		if self.laserdata:
			# Find the index of the laser scan data point corresponding to the forward direction.
			forward_index = self.angletoindex(0.0)
		
			# Check if the forward index is within the valid range of indices.
			if 0 <= forward_index < len(self.laserdata):
				# Get the distance to the nearest obstacle in the forward direction.
				distance = self.laserdata[forward_index]

			# Check if the distance is within the valid range (not infinity or NaN).
			if not math.isinf(distance) and not math.isnan(distance):
				return distance
		return None
	def next_behavior(self):
		forward_sum = sum(self.laserdata[self.angletoindex(-10 * math.pi / 180):self.angletoindex(10 * math.pi / 180) + 1])
		left_clear = self.left_wall_clear()
		right_clear = self.right_wall_clear()
		
		if (left_clear and right_clear):
			left_clear = random.choice([True, False])
			right_clear = random.choice([True, False])
		
		if self.state == 1 and self.vector_fow()< 0.7 and left_clear:
			self.state = 2
		elif self.state == 1 and self.vector_fow() < 0.7 and right_clear:
			self.state = 3
		elif self.vector_fow() < .1:
			self.state = 4
		else:
			self.state = 1

	def left_wall_clear(self):
        	left_sum = sum(self.laserdata[:len(self.laserdata) // 2])
        	return left_sum > 1.0
	def right_wall_clear(self):
		right_sum = sum(self.laserdata[-len(self.laserdata) // 2:])
		return right_sum > 1.0
	def move(self):
		msg = Twist()
		if self.laserdata:
			if self.laserdata:
				right_sum = 0  # Corrected variable name
				count = 0
				
			
			
				for i in range(self.angletoindex(self.min_angle), self.angletoindex(-10 * math.pi / 180), 1):
					right_sum += self.laserdata[i]
					count += 1
				# average the arc
				right_sum /= count
				# normalize via max distance
				right_sum /= self.maxrange

				left_sum = 0  # Corrected variable name
				count = 0

				for i in range(self.angletoindex(10 * math.pi / 180), self.angletoindex(self.max_angle), 1):
					left_sum += self.laserdata[i]
					count += 1
				# average the arc
				left_sum /= count
				# normalize via max distance
				left_sum /= self.maxrange
				
				
				
				self.next_behavior()

				if self.state == 1:
					msg.angular.z = self.vector_lat()
					msg.linear.x = self.vector_fow()

				elif self.state == 2:
					msg.linear.x = self.vector_fow()
					msg.angular.z = self.leftwallfollow()

				elif self.state == 3:
					msg.linear.x = self.vector_fow()
					msg.angular.z = self.rightwallfollow()

				elif self.state == 4:
					msg.linear.x = -1.0
					msg.angular.z = (left_sum - right_sum) * 2

				self.pub.publish(msg)

	def angletoindex(self, angle):
		return math.floor((angle - self.min_angle) / self.delta_angle)

	def indextoangle(self, index):
		return (index * self.delta_angle) + self.min_angle

def main(args=None):
	
	rclpy.init(args=args)
	mover = SimpleLaserDrive()
	rclpy.spin(mover)
	
        
	mover.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()

