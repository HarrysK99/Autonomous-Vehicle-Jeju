#!/usr/bin/env python3

import rospy
from rtcm_msgs.msg import Message
import socket
import time
import base64
import datetime

rospy.init_node('NTRIP_NODE', anonymous=False)
pub = rospy.Publisher('/ublox_gps/rtcm', Message, queue_size=10)
rate=rospy.Rate(10)

def generate_gga(latitude, longitude):
    timestamp = datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-4]

    latitude_val = abs(latitude)
    latitude_deg = int(latitude_val)
    latitude_min = (latitude_val - latitude_deg) * 60
    latitude_str = f"{latitude_deg:02}{latitude_min:09.6f}"
    latitude_dir = "N" if latitude >= 0 else "S"

    longitude_val = abs(longitude)
    longitude_deg = int(longitude_val)
    longitude_min = (longitude_val - longitude_deg) * 60
    longitude_str = f"{longitude_deg:03}{longitude_min:09.6f}"
    longitude_dir = "E" if longitude >= 0 else "W"

    gga = f"$GPGGA,{timestamp},{latitude_str},{latitude_dir},{longitude_str},{longitude_dir},1,04,1.0,10.0,M,0.0,M,,"
    gga += f"*{hex(generate_checksum(gga))[2:].upper():>02}"  # Add checksum
    return gga


def generate_checksum(sentence):
    sentence = sentence.lstrip("$")
    checksum = 0

    for char in sentence:
        checksum ^= ord(char)

    return checksum


# Example usage
latitude = 37.54126567
longitude = 127.07978833
nmea_gga = generate_gga(latitude, longitude)

# NTRIP 서버, 사용자 정보 및 스트림 설정
ntrip_server = "RTS1.ngii.go.kr"
ntrip_port = 2101
ntrip_user = "TEamKAI23"
ntrip_pass = "ngii"
ntrip_stream = "RTK-RTCM32"


# RTCM 데이터를 처리하는 함수
def handle_rtcm_data(data):
    print("Receved RTCM Data")
    rtcm_data=Message()
    rtcm_data.message=data
    pub.publish(rtcm_data)    

# NTRIP 클라이언트 생성 및 실행
def run_ntrip_client():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)  # Timeout 설정 (초)

    try:
        sock.connect((ntrip_server, ntrip_port))
        auth = base64.b64encode(f"{ntrip_user}:{ntrip_pass}".encode()).decode()
        request = f"GET /{ntrip_stream} HTTP/1.1\r\nUser-Agent: NTRIP PyExample/0.1\r\nAuthorization: Basic {auth}\r\n\r\n"
        sock.send(request.encode())

        response = sock.recv(1024).decode()

        if "ICY 200 OK" in response:
            print("Connected to NTRIP server.")

            while not rospy.is_shutdown():
                # NMEA GGA 데이터를 서버로 전송
                sock.send((nmea_gga + "\r\n").encode())
                # print("Sent NMEA GGA data to NTRIP server.")

                # 서버로부터 RTCM 데이터를 수신
                sock.settimeout(None)  # recv()에 타임아웃 적용하지 않음
                rtcm_data = sock.recv(1024)
                if rtcm_data:
                    handle_rtcm_data(rtcm_data)
                else:
                    break

                # NMEA GGA 데이터를 주기적으로 전송 (예: 5초마다)
                #time.sleep(0.125)
        else:
            print(f"Error connecting to NTRIP server: {response}")

    except socket.error as e:
        print(f"Error connecting to NTRIP server: {e}")

    finally:
        sock.close()
        print("Disconnected from NTRIP server.")




run_ntrip_client()