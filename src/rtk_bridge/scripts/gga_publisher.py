#!/usr/bin/env python3
"""
GGA Publisher Node - Publishes simulated GGA NMEA sentences (ROS1 version)
"""

import rospy
from std_msgs.msg import String
import time
import math
import random
from datetime import datetime


def calculate_nmea_checksum(sentence: str) -> str:
    """Calculate NMEA checksum for a sentence (without $ and *)"""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"


def generate_gga_sentence(lat: float, lon: float, altitude: float, 
                          num_satellites: int, hdop: float, 
                          fix_quality: int, utc_time: str) -> str:
    """
    Generate GPGGA NMEA sentence
    
    Args:
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees
        altitude: Altitude in meters
        num_satellites: Number of satellites in use
        hdop: Horizontal dilution of precision
        fix_quality: GPS fix quality (0=invalid, 1=GPS fix, 2=DGPS fix, 4=RTK fixed, 5=RTK float)
        utc_time: UTC time in HHMMSS.ss format
    
    Returns:
        Complete GPGGA sentence with checksum
    """
    # Convert latitude to NMEA format (DDMM.MMMMM)
    lat_deg = int(abs(lat))
    lat_min = (abs(lat) - lat_deg) * 60
    lat_dir = 'N' if lat >= 0 else 'S'
    lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
    
    # Convert longitude to NMEA format (DDDMM.MMMMM)
    lon_deg = int(abs(lon))
    lon_min = (abs(lon) - lon_deg) * 60
    lon_dir = 'E' if lon >= 0 else 'W'
    lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
    
    # Build GGA sentence (without $ and checksum)
    sentence = (
        f"GPGGA,{utc_time},{lat_str},{lat_dir},{lon_str},{lon_dir},"
        f"{fix_quality},{num_satellites:02d},{hdop:.1f},{altitude:.1f},M,0.0,M,,"
    )
    
    # Calculate and append checksum
    checksum = calculate_nmea_checksum(sentence)
    return f"${sentence}*{checksum}"


class GGAPublisher:
    """ROS1 Node that publishes simulated GGA data"""
    
    def __init__(self):
        # Initialize node
        rospy.init_node('gga_publisher', anonymous=True)
        
        # Get parameters
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        self.base_lat = rospy.get_param('~base_latitude', 39.9042)  # Beijing
        self.base_lon = rospy.get_param('~base_longitude', 116.4074)
        self.base_alt = rospy.get_param('~base_altitude', 50.0)
        self.simulate_movement = rospy.get_param('~simulate_movement', True)
        self.movement_speed = rospy.get_param('~movement_speed', 5.0)  # m/s
        self.fix_quality = rospy.get_param('~fix_quality', 4)  # 4 = RTK Fixed
        self.num_satellites = rospy.get_param('~num_satellites', 12)
        self.hdop = rospy.get_param('~hdop', 0.8)
        self.topic_name = rospy.get_param('~topic_name', '/rtk/gga')
        
        # Create publisher
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        
        # Initialize state
        self.time_offset = 0.0
        self.start_time = time.time()
        
        rospy.loginfo(f'GGA Publisher started on topic: {self.topic_name}')
        rospy.loginfo(f'Base position: ({self.base_lat}, {self.base_lon}, {self.base_alt})')
        rospy.loginfo(f'Publish rate: {self.publish_rate} Hz')
        rospy.loginfo(f'Fix quality: {self.fix_quality} (RTK Fixed)')
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            # Calculate position (with optional movement simulation)
            if self.simulate_movement:
                # Simulate circular movement
                radius = 10.0  # meters
                angular_speed = self.movement_speed / radius  # rad/s
                
                # Convert to approximate degree offsets
                lat_offset = radius * math.cos(elapsed * angular_speed) / 111111.0
                lon_offset = radius * math.sin(elapsed * angular_speed) / (111111.0 * math.cos(math.radians(self.base_lat)))
                
                lat = self.base_lat + lat_offset
                lon = self.base_lon + lon_offset
                alt = self.base_alt + random.uniform(-0.5, 0.5)  # Small altitude noise
            else:
                lat = self.base_lat + random.uniform(-0.00001, 0.00001)
                lon = self.base_lon + random.uniform(-0.00001, 0.00001)
                alt = self.base_alt + random.uniform(-0.1, 0.1)
            
            # Generate UTC time
            utc_now = datetime.utcnow()
            utc_time = utc_now.strftime('%H%M%S.') + f'{random.randint(0, 99):02d}'
            
            # Add some variation to satellites and HDOP
            num_sats = self.num_satellites + random.randint(-2, 2)
            hdop = self.hdop + random.uniform(-0.1, 0.1)
            
            # Generate GGA sentence
            gga_sentence = generate_gga_sentence(
                lat=lat,
                lon=lon,
                altitude=alt,
                num_satellites=num_sats,
                hdop=hdop,
                fix_quality=self.fix_quality,
                utc_time=utc_time
            )
            
            # Publish
            msg = String()
            msg.data = gga_sentence
            self.publisher.publish(msg)
            
            rospy.logdebug(f'Published: {gga_sentence}')
            
            rate.sleep()


if __name__ == '__main__':
    try:
        node = GGAPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
