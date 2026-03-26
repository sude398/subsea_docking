import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mavros_msgs.msg import State, OverrideRCIn
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .aruco_detector import ArUcoDetector

class DockingMissionNode(Node):
    def __init__(self):
        super().__init__('docking_mission_node')
        
        # Parameters
        self.declare_parameter('video_source', '/home/vedat/center_detector-v2_ws/media/video_05.mp4')
        self.declare_parameter('target_mode', 'GUIDED')
        self.declare_parameter('use_camera_topic', False) 
        
        self.video_source = self.get_parameter('video_source').value
        self.target_mode = self.get_parameter('target_mode').value
        self.use_camera_topic = self.get_parameter('use_camera_topic').value

        # State variables
        self.mission_enabled = True
        self.current_state = "SEARCHING"
        self.mavros_state = None
        self.bridge = CvBridge()
        self.detector = ArUcoDetector()
        
        # HUD Variables (Ekrana basılacak değerler)
        self.hud_distance = 0.0
        self.hud_centroid = None
        self.hud_num_detected = 0
        self.current_pwm = [1500] * 18
        
        # Mission parameters
        self.alignment_start_time = None
        self.hold_duration = 10.0
        self.is_mission_complete = False
        
        # RC PWM neutral
        self.PWM_NEUTRAL = 1500
        
        # Video Capture
        self.cap = None
        if not self.use_camera_topic:
            self.cap = cv2.VideoCapture(self.video_source)
            if not self.cap.isOpened():
                self.get_logger().error(f"❌ Failed to open video source: {self.video_source}")
        
        # Setup
        self.setup_qos_profiles()
        self.setup_subscriptions()
        self.setup_publishers()
        self.setup_services()
        
        # Timers
        self.create_timer(0.04, self.control_loop)  # 25 Hz
        if not self.use_camera_topic:
            self.create_timer(0.033, self.frame_processing_loop) 
        
        self.get_logger().info("⚓ ArUco Docking Mission Node initialized! HUD Active.")

    def setup_qos_profiles(self):
        self.mavros_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def setup_subscriptions(self):
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, self.mavros_state_qos)
        if self.use_camera_topic:
            self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def setup_publishers(self):
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.status_pub = self.create_publisher(String, '/target_status', 10)

    def setup_services(self):
        self.start_srv = self.create_service(Trigger, 'start_docking', self.start_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_docking', self.stop_callback)

    def start_callback(self, request, response):
        if self.mission_enabled:
            response.success = True; response.message = "Mission already enabled."
            return response
            
        self.mission_enabled = True
        self.is_mission_complete = False
        self.current_state = "SEARCHING"
        self.get_logger().info("🚀 ArUco Docking enabled!")
        response.success = True; response.message = "ArUco Docking started."
        return response

    def stop_callback(self, request, response):
        self.mission_enabled = False
        self.send_rc_override(surge=0, sway=0, heave=0, yaw=0) 
        self.get_logger().info("🛑 ArUco Docking disabled.")
        response.success = True; response.message = "ArUco Docking stopped."
        return response

    def state_callback(self, msg: State):
        self.mavros_state = msg

    def image_callback(self, msg: Image):
        if not self.mission_enabled: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def frame_processing_loop(self):
        if not self.mission_enabled or self.cap is None: return
            
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
            
        self.process_frame(frame)

    def process_frame(self, frame):
        # 1. Detect Markers
        detected_info, img_pts, obj_pts, corners = self.detector.detect_markers(frame)
        self.hud_num_detected = len(detected_info)
        self.hud_centroid = self.detector.get_centroid_centering(detected_info)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners)
        
        # 2. State Machine
        if self.current_state == "DOCKED":
            self.handle_docked_state()
        
        elif self.hud_num_detected == 0:
            self.current_state = "SEARCHING"
            self.detector.reset_pids()
            self.alignment_start_time = None
            self.hud_distance = 0.0
            self.command_rov(surge=200) 
            
        elif self.hud_num_detected >= 3:
            self.current_state = "PRECISION"
            pose = self.detector.calculate_pnp_pose(img_pts, obj_pts)
            if pose:
                f_x, f_y, f_z, f_yaw = pose
                self.hud_distance = f_z
                
                # Dedektörden ham (işareti ters) değerleri alıyoruz
                raw_surge, raw_sway, raw_heave, raw_yaw = self.detector.compute_pid_outputs(f_x, f_y, f_z, f_yaw)
                
                # --- YÖN DÜZELTMELERİ (CRITICAL FIX) ---
                # Hedef öndeyken ileri gitmek için negatifi pozitife çevir
                surge = -raw_surge
                
                # Hedef sağdayken sağa gitmek için negatifi pozitife çevir
                sway = -raw_sway
                
                # Hedef alttayken aşağı inmek (<1500) için negatif KALMALI.
                # (Eskiden önüne eksi koyuyordun, o yüzden yukarı kaçıyordu. Eksiyi sildik.)
                heave = raw_heave 
                
                # Yaw dönüşü de muhtemelen terstir, onu da tersliyoruz
                yaw = -raw_yaw 
                
                # Düzeltilmiş değerleri motora gönder
                self.command_rov(surge=surge, sway=sway, heave=heave, yaw=yaw)
                
                # Hold logic
                is_aligned = (abs(f_x) < 0.05 and abs(f_y) < 0.05 and abs(f_yaw) < 5.0 and f_z < 0.55)
                if is_aligned:
                    if self.alignment_start_time is None:
                        self.alignment_start_time = time.time()
                    if (time.time() - self.alignment_start_time) >= self.hold_duration:
                        self.current_state = "DOCKED"
                else:
                    self.alignment_start_time = None
            
        elif self.hud_num_detected >= 1:
            self.current_state = "APPROACHING"
            self.alignment_start_time = None
            if self.hud_centroid:
                cx, cy = self.hud_centroid
                sway = int(((cx - 960) / 960.0) * 400)
                heave_offset = int(((cy - 540) / 540.0) * 250)
                self.command_rov(surge=150, sway=sway, heave=-heave_offset)

        # 3. HUD Çizimini Çağır
        self.draw_hud(frame)

    def draw_hud(self, frame):
        """Frame üzerine otonom sistem durumunu çizer"""
        h, w = frame.shape[:2]
        color = (0, 255, 0) if self.hud_num_detected > 0 else (0, 0, 255)

        # 1. Hedef Merkez Crosshair (Yeşil Nişangah)
        if self.hud_centroid is not None:
            tgt_cx, tgt_cy = int(self.hud_centroid[0]), int(self.hud_centroid[1])
            cross_size = 30
            cv2.line(frame, (tgt_cx - cross_size, tgt_cy), (tgt_cx + cross_size, tgt_cy), (0, 255, 0), 3)
            cv2.line(frame, (tgt_cx, tgt_cy - cross_size), (tgt_cx, tgt_cy + cross_size), (0, 255, 0), 3)
            cv2.circle(frame, (tgt_cx, tgt_cy), 20, (0, 255, 0), 3)
            cv2.putText(frame, "TARGET", (tgt_cx + 25, tgt_cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 2. Sol Üst Ana Bilgiler
        cv2.putText(frame, f"MODE: {self.current_state}", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.8, color, 4)
        cv2.putText(frame, f"TAGS: {self.hud_num_detected} | DIST: {self.hud_distance:.2f}m", (40, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 0), 3)

        # 3. Alt PWM Paneli (Dinamik Konumlandırma)
        panel_y1 = h - 170
        panel_y2 = h - 20
        cv2.rectangle(frame, (20, panel_y1), (1060, panel_y2), (0, 0, 0), -1)
        
        hud_line1 = f"SURGE: {self.current_pwm[4]} | SWAY: {self.current_pwm[5]}"
        hud_line2 = f"HEAVE: {self.current_pwm[2]} | YAW : {self.current_pwm[3]}"
        cv2.putText(frame, hud_line1, (40, panel_y1 + 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        cv2.putText(frame, hud_line2, (40, panel_y1 + 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

        # 4. Görüntüyü Yeniden Boyutlandırma ve Gösterme
        # (Aspect Ratio'yu koruyarak %80 küçültüyoruz)
        scale_percent = 80
        width = int(w * scale_percent / 100)
        height = int(h * scale_percent / 100)
        small_frame = cv2.resize(frame, (width, height))
        
        cv2.imshow("TAC 2026 - RAMI ROV", small_frame)
        cv2.waitKey(1)

    def handle_docked_state(self):
        self.command_rov(surge=0, sway=0, heave=0, yaw=0)
        if not self.is_mission_complete:
            self.status_pub.publish(String(data="DOCKING_COMPLETE"))
            self.get_logger().info("🏁 DOCKING COMPLETE!")
            self.is_mission_complete = True
            self.mission_enabled = False

    def command_rov(self, surge=0, sway=0, heave=0, yaw=0):
        # MAVROS bağlantısı olmasa bile HUD'da PWM değerlerini test edebilmek için 
        # local test durumlarında Override gönderilmiş gibi kaydediyoruz.
        self.send_rc_override(surge, sway, heave, yaw)

        # Güvenlik Kontrolü (ROS üzerindeyse)
        if self.mavros_state is not None:
            autonomous_active = (self.mavros_state.armed and self.mavros_state.mode == self.target_mode)
            if not autonomous_active:
                # Gerçek görevde güvenli moda geçiş
                pass

    def send_rc_override(self, surge, sway, heave, yaw):
        msg = OverrideRCIn()
        channels = [65535] * 18
        
        def map_val(val):
            val = max(-400, min(400, val))
            return self.PWM_NEUTRAL + val

        # Değerleri maple
        mapped_heave = map_val(heave)
        mapped_yaw = map_val(yaw)
        mapped_surge = map_val(surge)
        mapped_sway = map_val(sway)

        # HUD için güncel durumu kaydet (MAVROS array sırasına göre)
        self.current_pwm = [1500] * 18
        self.current_pwm[2] = mapped_heave
        self.current_pwm[3] = mapped_yaw
        self.current_pwm[4] = mapped_surge
        self.current_pwm[5] = mapped_sway

        channels[2] = mapped_heave  # CH 3
        channels[3] = mapped_yaw    # CH 4
        channels[4] = mapped_surge  # CH 5
        channels[5] = mapped_sway   # CH 6
        
        msg.channels = channels
        self.rc_pub.publish(msg)

    def control_loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DockingMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap: node.cap.release()
        neutral_msg = OverrideRCIn()
        neutral_msg.channels = [0] * 18
        node.rc_pub.publish(neutral_msg)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()