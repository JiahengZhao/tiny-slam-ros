import math
import random
import rospy
import collections

#define TS_SCAN_SIZE 8192
#define TS_MAP_SIZE 2048
#define TS_MAP_SCALE 0.1
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0

class RobotParams:
    # Scaner
    detectionMargin = 0
    holeWidth = 0.1 # m

    # Method
    sigmaCoord = 0.2
    sigmaTheta = 0.07
    badMCIterations = 10
    maxMCIterations = 50
    
class RobotPose:
    def __init__(self, x = 0, y = 0, theta = 0): # x,y - mm. theta - rad
        self.x, self.y, self.theta = x, y, theta

    def copy(self):
        return RobotPose(self.x, self.y, self.theta)
        
    def translate_by_dist(self, d):
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)
        return self

    def translate(self, dx, dy):
        self.x += dx
        self.y += dy
        return self

    def rotate(self, delta):
        self.theta += delta
        return self

    def __sub__(self, other):
        return (self.x - other.x, self.y - other.y, self.theta - other.theta)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.theta == other.theta

    def __str__(self):
        return "x:%.2f y:%.2f theta:%.2f" % (self.x, self.y, self.theta)

class RobotState:
    def __init__(self):
        self.pose = RobotPose(0, 0, 0)
        self.angular_v = 0
        self.linear_v = 0
        
    def update(self, robot_pose, lin_speed, ang_speed):
        self.pose = robot_pose
        self.linear_v = lin_speed
        self.angular_v = ang_speed
        #dist = RobotParams.metersPerOdomTick * sensor_data.odom_ticks * 1000 # to mm
        #self.pose.translate(dist).rotate(0) # TODO: rotation from ROS

        # TODO: Update speed from ROS odometry
        # v *= 1000000.0 / (sd->timestamp - state->timestamp);
        # psidot *= 1000000.0 / (sd->timestamp - state->timestamp);
        
    def __str__(self):
        return "Pose:%s; Lin:%.2f; Ang:%.2f" % (self.pose, self.linear_v, self.angular_v)

# broken
def line2D(x1, y1, x2, y2):
    dx, dy = abs(x2 - x1), abs(y2 - y1)
    if dy > dx:
        dx, dy = dy, dx
        x1, x2, y1, y2 = y1, y2, x1, x2 
    dir_x = 1 if x1 < x2 else -1
    dir_y = 1 if y1 < y2 else -1 
    e = 2 * dy - dx
    py = y1
    for px in range(x1, x2, dir_x):
        yield (px, py)
        e += 2 * (dy - dx) if 0 < e else 2 * dy
        py += dir_y if 0 < e else 0

def line2D(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in xrange(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
#        if 20 <= len(points):
#            break
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points
        
class Map:
    OBSTACLE = 100
    NO_OBSTACLE = 0
    DEFAULT = (OBSTACLE + NO_OBSTACLE) // 2
    
    def __init__(self, cell_count, cell_size_cm):
        if cell_size_cm < 1:
            raise Exception("The finest map resolution in 1cm")
        self.size = cell_count
        #self.data = [Map.DEFAULT for _ in range(self.size**2)]
        self.data = collections.defaultdict(lambda :Map.DEFAULT)
        self.scale = cell_size_cm / 100.0 # m per celll

    def to_cell(self, x, y):
        return (int(round(1.0*x / self.scale)), int(round(1.0*y / self.scale)))

    def __to_list_index(self, x, y):
        if not self.__hasPoint(x, y):
            return None
        # TODO: fix it with tf
        return (y + self.size/2) * self.size + x + self.size/2
        
    def __getitem__(self, key):
        #if not self.__hasPoint(*key):
        #    return Map.DEFAULT
        #return self.data[self.__to_list_index(*key)]
        return self.data[key]
    
    def __setitem__(self, key, value):
        # if self.__hasPoint(*key):
        #self.data[self.__to_list_index(*key)] = value
        self.data[key] = value

    def __hasPoint(self, x, y):
        size_half = self.size / 2
        return -size_half <= x < size_half and -size_half <= y < size_half

    def to_ros_grid(self):
        #return self.data
        grid = [Map.DEFAULT for _ in xrange(self.size**2)]
        for (x, y), v in self.data.items():
            i = self.__to_list_index(x, y)
            if i:
                grid[i] = v
        return grid

    def __scanDistance(self, robot_pose, laser_scan_obstacles):
        r_x, r_y = robot_pose.x, robot_pose.y
        r_t = robot_pose.theta

        c, s = math.cos(r_t), math.sin(r_t)
        point_cnt, sum = 0, 0

        cells = set()
        for sx, sy, val in laser_scan_obstacles:
            # Change basis: scan -> world    
            w_coord = self.to_cell(r_x + c * sx - s * sy, r_y + s * sx + c * sy)
            if w_coord in cells:
                continue
            if self.__hasPoint(*w_coord):
                sum += Map.OBSTACLE - self[w_coord]
                point_cnt += 1
            cells.add(w_coord)
        #rospy.logdebug("[ScanDistance] Hit Rate %.2f", 1 - 1.0*len(cells) / len(laser_scan_obstacles))
        return sum * 1024.0 / point_cnt if point_cnt else 2000000000

    def localize(self, robot_pose, laser_scan):
        sigma_coord, sigma_theta = RobotParams.sigmaCoord, RobotParams.sigmaTheta
        laser_scan_obstacles = filter(lambda rec: rec[2] == Map.OBSTACLE, laser_scan.data)
        
        estimated_pose = robot_pose.copy().translate_by_dist(0) # laser offset
        estimated_dist = self.__scanDistance(estimated_pose, laser_scan_obstacles)
        init_dist = estimated_dist
    
        bad_samples_cnt = total_samples_cnt = 0
        while bad_samples_cnt < RobotParams.badMCIterations and total_samples_cnt < RobotParams.maxMCIterations:
            total_samples_cnt += 1
            
            sample = estimated_pose.copy() \
                                   .translate(random.gauss(0, sigma_coord), random.gauss(0, sigma_coord)) \
                                   .rotate(random.gauss(0, sigma_theta))
            sample_dist = self.__scanDistance(sample, laser_scan_obstacles)
            
            if estimated_dist <= sample_dist:
                bad_samples_cnt += 1 
                continue
            
            estimated_dist, estimated_pose = sample_dist, sample
            if RobotParams.badMCIterations // 3 < bad_samples_cnt:
                bad_samples_cnt = 0
                sigma_coord *= 0.5
                sigma_theta *= 0.5

        pose_diff = estimated_pose - robot_pose
        d_pose = max(abs(pose_diff[0]), abs(pose_diff[1]), abs(pose_diff[2]))
        if 0.0 < d_pose:
            rospy.loginfo("MC Correction -- dx: %.2f | dy: %.2f | dt: %.2f ",
                          pose_diff[0], pose_diff[1], pose_diff[2])
        return estimated_pose.translate_by_dist(-0) # laser offset

    def __clipPoint(self, x, y):
        # TODO: implement
        return x, y

    def __distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
    
    def __distance_sq(self, x1, y1, x2, y2):
        return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)

    def __mapLaserRay(self, robot_x, robot_y, mean_scan_x, mean_scan_y, max_scan_x, max_scan_y, quality, value,
                      dist_cache, used_empty_pts):
        if not self.__hasPoint(robot_x, robot_y): return 0
        max_scan_x_clip, max_scan_y_clip = self.__clipPoint(max_scan_x, max_scan_y)
        scan_dist_sq = float(self.__distance_sq(robot_x, robot_y, mean_scan_x, mean_scan_y))
        hole_width_sq = float(self.__distance_sq(robot_x, robot_y, max_scan_x, max_scan_y) - scan_dist_sq)
        
        points_to_be_updated = line2D(robot_x, robot_y, max_scan_x_clip, max_scan_y_clip)
        for x, y in points_to_be_updated:
            w_coord = (x, y)
#            if w_coord in used_empty_pts:
#                continue
            pixval = Map.NO_OBSTACLE
            if value != Map.NO_OBSTACLE:
                if not w_coord in dist_cache:
                    dist_cache[w_coord] = float(self.__distance_sq(robot_x, robot_y, x, y))
                d = dist_cache[w_coord]
                if hole_width_sq:
                    if d >= scan_dist_sq - hole_width_sq:
                        k = (1.0 - abs(scan_dist_sq - d) / hole_width_sq)
                        v = Map.DEFAULT if d > scan_dist_sq else Map.NO_OBSTACLE
                        pixval = v + k * abs(value - v)
                elif d >= scan_dist_sq:
                    pixval = value

            self[w_coord] = ((256 - quality) * self[w_coord] + quality * pixval) // 256
            if pixval == Map.NO_OBSTACLE:
                used_empty_pts.add(w_coord)
        return len(points_to_be_updated)
        """
        if not self.__hasPoint(robot_x, robot_y): return 0
        max_scan_x_clip, max_scan_y_clip = self.__clipPoint(max_scan_x, max_scan_y)
        scan_dist = self.__distance_sq(robot_x, robot_y, mean_scan_x, mean_scan_y)
        # FIXME: replace mean_scan_* with max_scan_*_clip
        points_to_be_updated = line2D(robot_x, robot_y, mean_scan_x, mean_scan_y)
        for x, y in points_to_be_updated:
            d = self.__distance_sq(robot_x, robot_y, x, y)
            pixval = Map.NO_OBSTACLE if d < scan_dist else value
            self[(x, y)] = ((256 - quality) * self[(x, y)] + quality * pixval) // 256
            if d > scan_dist: break
        return len(points_to_be_updated)
        """

    def update(self, robot_pose, laser_scan, quality):
        upd_pts = 0

        c, s = math.cos(robot_pose.theta), math.sin(robot_pose.theta)
        
        map_robot_x, map_robot_y = self.to_cell(robot_pose.x, robot_pose.y)
        dist_cache = {}
        used_empty_pts = set()
        for sx, sy, val in laser_scan.data:
            robot_scan_x = c * sx - s * sy
            robot_scan_y = s * sx + c * sy
            map_scan_x, map_scan_y = self.to_cell(robot_pose.x + robot_scan_x, robot_pose.y + robot_scan_y)
            map_scan_dist = float(self.__distance(0, 0, sx, sy))
            if not map_scan_dist:
                continue
            
            hole_scale = 1.0 + (RobotParams.holeWidth / (2.0 * map_scan_dist) if map_scan_dist != 0 else 0)
            map_hole_x, map_hole_y = self.to_cell(robot_pose.x + robot_scan_x * hole_scale,
                                                  robot_pose.y + robot_scan_y * hole_scale)

            q, v = (quality / 4, Map.NO_OBSTACLE) if val == Map.NO_OBSTACLE else (quality, Map.OBSTACLE)
            #start_time = rospy.get_time()
            upd_pts += self.__mapLaserRay(map_robot_x, map_robot_y, map_scan_x, map_scan_y, map_hole_x, map_hole_y, q, v,
                                          dist_cache, used_empty_pts)

class LaserScan:
    def __init__(self, sensor_data, robot_state, slam_map):
        self.data = [] 

        cells = set()
        for i in xrange(sensor_data.scan_size):
            if i < RobotParams.detectionMargin or sensor_data.scan_size - RobotParams.detectionMargin < i: 
                continue
            if sensor_data.scan_data[i] == -1:
                continue

            dist, val = (sensor_data.scan_data[i], Map.OBSTACLE)
            if dist == 0:
                dist, val = (sensor_data.max_distance, Map.NO_OBSTACLE)
            
            if dist < RobotParams.holeWidth / 2.0: 
                continue
            
            angle_offset = i * (sensor_data.angle_max - sensor_data.angle_min) / (sensor_data.scan_size - 1)
            angle = sensor_data.angle_min + angle_offset
            # TODO: derivation & to radians
            #angle_deg += robot_state.angular_v * angle_offset / 3600.0

            
            x = dist * math.cos(angle)
            # TODO: derivation & to radians
            #x -= robot_state.linear_v * 1000 * angle_offset / 3600.0
            y = dist * math.sin(angle)
            cell = slam_map.to_cell(x, y)

            if cell in cells:
                continue
            cells.add(cell)
            self.data.append((x, y, val))
        rospy.logdebug("[ScanInit] Hit rate: %.3f", 1.0 - 1.0*len(cells)/sensor_data.scan_size)

    def __str__(self):
        return str(random.choice(self.data))
        
class RawLaserScanData:
    def __init__(self):
#        self.timestamp = 0
#        self.odom_ticks = 0
#        self.angle_v = 0
#        self.linear_v = 0
        self.scan_data = []
        self.scan_size = 0
        self.max_distance = 0
        self.angle_min = 0
        self.angle_max = 0


