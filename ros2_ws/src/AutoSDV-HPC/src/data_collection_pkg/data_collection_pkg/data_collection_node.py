import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import sys
import tty
import termios
import threading
from interfaces_pkg.msg import MotionCommand

def getch():
    """Gets a single character from stdin"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class DataCollector:
    def __init__(self):
        # --- [수정 1] 타이머와 상태 변수 다시 추가 ---
        self.action_active = False # 동작 활성화 상태
        self.action_timer = None   # 타이머 객체
        self.reset_values()

    def reset_values(self):
        self.exit_flag = False
        # 동작 중이 아닐 때만 값을 초기화
        if not self.action_active:
            self.steering = 0
            self.left_speed = 0
            self.right_speed = 0
    
    # --- [추가 1] 시간제한 동작을 시작하는 일반 함수 ---
    def start_timed_action(self, node, steering, speed, duration, description):
        if self.action_active:
            node.get_logger().info("Another action is already in progress.")
            return

        node.get_logger().info(f"Starting action: {description} for {duration} seconds...")
        self.action_active = True
        self.steering = steering
        self.left_speed = speed
        self.right_speed = speed
        
        # 주어진 시간(duration) 후에 stop_action 함수를 호출
        self.action_timer = threading.Timer(duration, self.stop_action, [node])
        self.action_timer.start()

    # --- [추가 2] 동작을 멈추는 함수 ---
    def stop_action(self, node):
        node.get_logger().info("Action finished. Stopping.")
        self.action_active = False
        self.reset_values()

    def process_key(self, key, node):
        # --- [수정 2] 키 처리 로직을 start_timed_action 호출로 변경 ---
        if self.action_active:
            if key == 'q' or key == 'z': # q 또는 z로 종료
                self.exit_flag = True
                if self.action_timer: self.action_timer.cancel()
                print("\nExiting...\n")
            elif key == ' ':
                if self.action_timer: self.action_timer.cancel()
                self.stop_action(node)
                print("\nEmergency Stop!")
            return

        # 각 키에 맞는 동작과 시간 설정
        if key == 'a': # 정방향 1단계 (7초)
            self.start_timed_action(node, steering=0, speed=100, duration=6.0, description="Forward Straight")
        elif key == 's': # 정방향 2단계 (5초)
            self.start_timed_action(node, steering=-6, speed=30, duration=5.0, description="Forward Left Turn")
        elif key == 'd': # 정방향 3단계 (9초)
            self.start_timed_action(node, steering=7, speed=-30, duration=8.0, description="Reverse Right Turn")
        elif key == 'e': # 역방향 1단계 (9초)
            self.start_timed_action(node, steering=7, speed=30, duration=8.0, description="Reverse Sequence's Forward Right")
        elif key == 'w': # 역방향 2단계 (5초)
            self.start_timed_action(node, steering=-6, speed=-30, duration=5.0, description="Reverse Left Turn")
        elif key == 'q': # 역방향 3단계 (7초)
            self.start_timed_action(node, steering=0, speed=-30, duration= 8.0, description="Reverse Straight")
        
        # 기타 제어 키
        elif key == ' ':
            self.reset_values()
            print("\nEmergency Stop!")
        elif key == 'r':
            self.reset_values()
            print(f"\nReset all values to zero")
        elif key == 'z': # 종료 키
            self.exit_flag = True
            print("\nExiting...\n")
        # --- [수정 2 끝] ---

    def get_control_values(self):
        return {
            'steering': self.steering,
            'left_speed': self.left_speed,
            'right_speed': self.right_speed
        }

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.publisher = self.create_publisher(MotionCommand, 'topic_control_signal', self.qos_profile)
        self.data_collector = DataCollector()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.print_instructions()

    # --- [수정 3] 키보드 조작 설명에 시간 추가 ---
    def print_instructions(self):
        print("\n" + "="*60)
        print("ROBOT MOTION CONTROL - 6 TIMED ACTIONS")
        print("="*60)
        print("Forward Actions:")
        print("  A : Forward Straight (7s)")
        print("  S : Forward Left   (5s)")
        print("  D : Reverse Right  (9s)")
        print("\nReverse Actions:")
        print("  E : Forward Right  (9s)")
        print("  W : Reverse Left   (5s)")
        print("  Q : Reverse Straight (7s)")
        print("\nOther Controls:")
        print("  SPACE: Emergency stop")
        print("  R    : Reset all values to zero")
        print("  Z    : Quit")
        print("="*60)
        
    def input_loop(self):
        while rclpy.ok():
            try:
                key = getch()
                self.data_collector.process_key(key, self)
                if self.data_collector.exit_flag:
                    break
            except KeyboardInterrupt:
                break
        
    def timer_callback(self):
        if self.data_collector.exit_flag:
            self.publish_stop_command()
            rclpy.shutdown()
            return
            
        control_values = self.data_collector.get_control_values()
        
        msg = MotionCommand()
        msg.steering = control_values['steering']
        msg.left_speed = control_values['left_speed']
        msg.right_speed = control_values['right_speed']
        self.publisher.publish(msg)
    
    def publish_stop_command(self):
        msg = MotionCommand()
        msg.steering = 0
        msg.left_speed = 0
        msg.right_speed = 0
        self.publisher.publish(msg)
        self.get_logger().info("Published stop command")
    
    def cleanup(self):
        self.data_collector.exit_flag = True
        if self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
        self.publish_stop_command()

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()