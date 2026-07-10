#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ota_update_interfaces.msg import UpdateNotification
import paho.mqtt.client as mqtt
import json
import os
import requests
from pathlib import Path
import threading
from urllib.parse import urlparse, unquote
import http.server
import socketserver
import socket


class OTAManagerNode(Node):
    def __init__(self):
        super().__init__('ota_manager_node')
        
        # ROS2 Publisher - ZCU에게 업데이트 알림
        self.update_publisher = self.create_publisher(
            UpdateNotification,
            'zcu_update_ready',
            10
        )
        
        # 파라미터 선언
        self.declare_parameter('mqtt_broker', 'skku-ecu-ota-server.duckdns.org')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'ota/update')
        self.declare_parameter('download_dir', './Updates')
        self.declare_parameter('http_port', 8000)
        
        # 파라미터 가져오기
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.download_dir = self.get_parameter('download_dir').value
        self.http_port = self.get_parameter('http_port').value
        
        # 다운로드 디렉토리 생성
        Path(self.download_dir).mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Download directory: {os.path.abspath(self.download_dir)}')
        
        # HPC IP 주소 파라미터로 설정
        self.declare_parameter('hpc_ip', '10.0.0.10')
        self.hpc_ip = self.get_parameter('hpc_ip').value
        self.get_logger().info(f'HPC IP address: {self.hpc_ip}')
        
        # HTTP 파일 서버 시작
        self.start_http_server()
        
        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client(client_id='ros2_ota_manager')
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # MQTT 연결
        self.connect_mqtt()
        
        # MQTT 루프를 별도 스레드에서 실행
        self.mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()
        
        self.get_logger().info('OTA Manager Node has been started')
    
    def start_http_server(self):
        """Updates 폴더를 제공하는 HTTP 서버 시작"""
        download_dir = os.path.abspath(self.download_dir)
        
        class CustomHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=download_dir, **kwargs)
            
            def log_message(self, format, *args):
                # 로그를 ROS2 logger로 리다이렉트하거나 조용히 함
                pass
        
        def serve():
            with socketserver.TCPServer(("", self.http_port), CustomHandler) as httpd:
                self.get_logger().info(f'HTTP file server started on port {self.http_port}')
                self.get_logger().info(f'Serving files from: {download_dir}')
                self.get_logger().info(f'Access files at: http://{self.hpc_ip}:{self.http_port}/')
                httpd.serve_forever()
        
        http_thread = threading.Thread(target=serve, daemon=True)
        http_thread.start()
    
    def connect_mqtt(self):
        """MQTT 브로커에 연결"""
        try:
            self.get_logger().info(f'Connecting to MQTT broker: {self.mqtt_broker}:{self.mqtt_port}')
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {str(e)}')
            self.create_timer(5.0, self.connect_mqtt)
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT 연결 콜백"""
        if rc == 0:
            self.get_logger().info(f'Connected to MQTT broker successfully')
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f'Subscribed to topic: {self.mqtt_topic}')
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker with code: {rc}')
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT 연결 해제 콜백"""
        if rc != 0:
            self.get_logger().warn(f'Unexpected MQTT disconnection. Will auto-reconnect')
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT 메시지 수신 콜백"""
        try:
            payload = json.loads(msg.payload.decode())
            self.get_logger().info(f'Received update notification: {payload}')
            
            if 'target' not in payload or 'version' not in payload or 'url' not in payload:
                self.get_logger().error('Invalid update notification: missing required fields')
                return
            
            target = payload['target']
            version = payload['version']
            url = payload['url']
            
            self.get_logger().info(f'New update available - Target: {target}, Version: {version}')
            
            # 파일 다운로드 (별도 스레드에서)
            download_thread = threading.Thread(
                target=self.download_update_file,
                args=(target, version, url)
            )
            download_thread.start()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse MQTT message: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {str(e)}')
    
    def extract_filename_from_url(self, url):
        """URL에서 파일명 추출"""
        parsed_url = urlparse(url)
        filename = os.path.basename(parsed_url.path)
        filename = unquote(filename)
        
        if not filename or filename == '':
            filename = 'update_file.bin'
            
        return filename
    
    def download_update_file(self, target, version, url):
        """업데이트 파일 다운로드"""
        try:
            self.get_logger().info(f'Starting download from: {url}')
            
            filename = self.extract_filename_from_url(url)
            filepath = os.path.join(self.download_dir, filename)
            
            self.get_logger().info(f'Extracted filename: {filename}')
            
            response = requests.get(url, stream=True, timeout=30)
            response.raise_for_status()
            
            total_size = int(response.headers.get('content-length', 0))
            self.get_logger().info(f'Downloading file: {filename} ({total_size} bytes)')
            
            downloaded_size = 0
            
            with open(filepath, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    if chunk:
                        f.write(chunk)
                        downloaded_size += len(chunk)
                        
                        if total_size > 0:
                            progress = (downloaded_size / total_size) * 100
                            if progress % 10 < 1:
                                self.get_logger().info(f'Download progress: {progress:.1f}%')
            
            self.get_logger().info(f'Download completed: {filepath}')
            self.get_logger().info(f'Downloaded size: {downloaded_size} bytes')
            
            # ZCU에게 업데이트 알림
            self.notify_zcu(target, version, filepath, downloaded_size)
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Failed to download file: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error during file download: {str(e)}')
    
    def notify_zcu(self, target, version, filepath, file_size):
        """ZCU에게 업데이트 파일 준비 완료 알림"""
        msg = UpdateNotification()
        msg.target = target
        msg.version = version
        
        # 로컬 파일 경로를 HTTP URL로 변환
        filename = os.path.basename(filepath)
        msg.file_path = f"http://{self.hpc_ip}:{self.http_port}/{filename}"
        
        msg.file_size = file_size
                
        self.update_publisher.publish(msg)
        self.get_logger().info(f'Published update notification to ZCU: {target} v{version}')
        self.get_logger().info(f'  Download URL: {msg.file_path}')
        self.get_logger().info(f'  File size: {msg.file_size} bytes')
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.get_logger().info('Shutting down OTA Manager Node')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OTAManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()