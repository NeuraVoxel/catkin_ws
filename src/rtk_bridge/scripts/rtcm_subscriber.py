#!/usr/bin/env python3
"""
RTCM Subscriber Node - Receives RTCM correction data (ROS1 version)
"""

import rospy
from std_msgs.msg import UInt8MultiArray, String
from sensor_msgs.msg import NavSatFix, NavSatStatus
import base64
from collections import deque
from datetime import datetime
import random


class RTCMSubscriber:
    """ROS1 Node that subscribes to RTCM correction data"""
    
    def __init__(self):
        # Initialize node
        rospy.init_node('rtcm_subscriber', anonymous=True)
        
        # Get parameters
        self.input_topic = rospy.get_param('~input_topic', '/rtcm/data')
        self.status_topic = rospy.get_param('~status_topic', '/rtcm/status')
        self.buffer_size = rospy.get_param('~buffer_size', 100)
        
        # Create subscription for RTCM data
        self.subscription = rospy.Subscriber(
            self.input_topic,
            UInt8MultiArray,
            self.rtcm_callback,
            queue_size=10
        )
        
        # Create publisher for status
        self.status_publisher = rospy.Publisher(self.status_topic, String, queue_size=10)
        
        # Create publisher for simulated RTK GNSS data
        self.gnss_publisher = rospy.Publisher('rtk/gnss', NavSatFix, queue_size=10)
        
        # Statistics
        self.total_messages = 0
        self.total_bytes = 0
        self.message_buffer = deque(maxlen=self.buffer_size)
        self.last_message_time = None
        
        # Simulated RTK state for dynamic status changes
        self.simulated_status = -1  # Start with no fix
        self.status_change_counter = 0
        self.status_cycle = [-1, 0, 1, 2, 2, 2, 1, 0]  # Cycle through statuses
        
        # Timer for publishing simulated RTK status (1 Hz)
        self.rtk_timer = rospy.Timer(rospy.Duration(1.0), self.publish_simulated_rtk)
        
        rospy.loginfo(f'RTCM Subscriber started')
        rospy.loginfo(f'Subscribed to: {self.input_topic}')
        rospy.loginfo(f'Status topic: {self.status_topic}')
    
    def rtcm_callback(self, msg: UInt8MultiArray):
        """Process incoming RTCM data"""
        try:
            # Convert UInt8MultiArray to bytes
            raw_data = msg.data
            
            if isinstance(raw_data, bytes):
                data = raw_data
            elif isinstance(raw_data, list):
                # List of single-byte bytes objects (from rosbridge)
                if len(raw_data) > 0:
                    if isinstance(raw_data[0], int):
                        # List of integers (0-255)
                        data = bytes(raw_data)
                    elif isinstance(raw_data[0], bytes):
                        # List of single-byte bytes objects
                        data = b''.join(raw_data)
                    else:
                        rospy.logerr(f'Unknown list element type: {type(raw_data[0])}')
                        return
                else:
                    data = b''
            elif isinstance(raw_data, str):
                data = base64.b64decode(raw_data)
            else:
                rospy.logerr(f'Unknown data type: {type(raw_data)}')
                return
            
            if len(data) == 0:
                rospy.logwarn('Received empty RTCM message')
                return
            
            # Update statistics
            self.total_messages += 1
            self.total_bytes += len(data)
            self.last_message_time = datetime.now()
            
            # Print received data as hex string
            hex_str = data.hex()
            rospy.loginfo(f'RTCM received: {len(data)} bytes, hex: {hex_str[:100]}...' if len(hex_str) > 100 else f'RTCM received: {len(data)} bytes, hex: {hex_str}')
            
            # Store in buffer
            self.message_buffer.append({
                'timestamp': self.last_message_time.isoformat(),
                'size': len(data),
                'hex': hex_str
            })
            
            # Publish status
            self.publish_status(len(data))
        
        except Exception as e:
            rospy.logerr(f'Error processing RTCM message: {e}')
    
    def publish_status(self, size: int):
        """Publish RTCM status message"""
        status_msg = String()
        status_msg.data = (
            f'{{"size": {size}, "total_messages": {self.total_messages}, "total_bytes": {self.total_bytes}}}'
        )
        self.status_publisher.publish(status_msg)
    
    def get_statistics(self) -> dict:
        """Get current statistics"""
        return {
            'total_messages': self.total_messages,
            'total_bytes': self.total_bytes,
            'last_message_time': self.last_message_time.isoformat() if self.last_message_time else None,
            'buffer_size': len(self.message_buffer)
        }
    
    def publish_simulated_rtk(self, event):
        """Publish simulated RTK GNSS data with dynamic status changes"""
        msg = NavSatFix()
        
        # Header
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'gps_link'
        
        # Simulate position (example coordinates)
        msg.latitude = 31.2304 + random.uniform(-0.0001, 0.0001)
        msg.longitude = 121.4737 + random.uniform(-0.0001, 0.0001)
        msg.altitude = 10.0 + random.uniform(-1.0, 1.0)
        
        # Dynamic RTK status simulation
        # If receiving RTCM data, cycle through improving statuses
        # If no RTCM, degrade status over time
        if self.total_messages > 0 and self.last_message_time:
            time_since_last = (datetime.now() - self.last_message_time).total_seconds()
            
            if time_since_last < 3:
                # Active RTCM reception - cycle through statuses
                self.status_change_counter += 1
                # Change status every 3 seconds
                if self.status_change_counter >= 3:
                    cycle_index = (self.status_change_counter // 3) % len(self.status_cycle)
                    self.simulated_status = self.status_cycle[cycle_index]
            elif time_since_last < 10:
                # Recent RTCM - hold at differential or better
                self.simulated_status = max(self.simulated_status, 1)
            elif time_since_last < 30:
                # Aging RTCM - degrade to single point
                self.simulated_status = max(self.simulated_status - 1, 0)
            else:
                # Old RTCM - no fix
                self.simulated_status = -1
        else:
            # No RTCM received yet - gradually improve then cycle
            self.status_change_counter += 1
            if self.status_change_counter < 5:
                # First 5 seconds: no fix
                self.simulated_status = -1
            else:
                # Cycle through statuses to simulate acquisition
                cycle_index = self.status_change_counter % len(self.status_cycle)
                self.simulated_status = self.status_cycle[cycle_index]
        
        msg.status.status = self.simulated_status
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Position covariance (better fix = smaller values)
        covariance_scale = 10.0 if self.simulated_status < 0 else (5.0 if self.simulated_status < 2 else 1.0)
        msg.position_covariance = [covariance_scale, 0.0, 0.0,
                                   0.0, covariance_scale, 0.0,
                                   0.0, 0.0, covariance_scale * 2.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gnss_publisher.publish(msg)
        
        status_labels = {-1: '无解', 0: '单点解', 1: '差分解', 2: '固定解'}
        rospy.loginfo(f'Published RTK status: {status_labels.get(msg.status.status, "未知")} (status={msg.status.status})')


if __name__ == '__main__':
    try:
        node = RTCMSubscriber()
        rospy.spin()
        # Print final statistics
        stats = node.get_statistics()
        rospy.loginfo(f'Final statistics: {stats}')
    except rospy.ROSInterruptException:
        pass
