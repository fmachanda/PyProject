from pymavlink import mavutil
import os

os.environ['MAVLINK20'] = '1'
mavutil.set_dialect('fmuas')

import threading
import struct
import socket

import time
time_boot=time.time_ns()//1000

FREQ = 60

print("Looking for X-Plane")
import common.find_xp as xp
beacon=xp.find_xp(wait=0)
X_PLANE_IP=beacon['ip']
UDP_PORT=beacon['port']

print("X-Plane found at IP: %s, port: %s" % (X_PLANE_IP,UDP_PORT))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((X_PLANE_IP, 0))

DEG_TO_RAD = 1.745328e-2
FT_TO_M = 3.048e-1
KT_TO_CMS = 5.14444e1

ap_udp_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14571')

att_udp_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14551')
att_udp_conn.srcComponent = mavutil.mavlink.MAV_COMP_ID_USER1
att_udp_conn.srcSystem = 1

def send_att_heartbeat():
    print("ATT Activated")
    while True:
        try:
            att_udp_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                mavutil.mavlink.MAV_MODE_PREFLIGHT,
                0,
                mavutil.mavlink.MAV_STATE_STANDBY)

            time.sleep(1)

        except KeyboardInterrupt:
            break


alt_udp_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14552')
alt_udp_conn.srcComponent = mavutil.mavlink.MAV_COMP_ID_USER2
alt_udp_conn.srcSystem = 1

def send_alt_heartbeat():
    print("ALT Activated")
    while True:
        try:
            alt_udp_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                mavutil.mavlink.MAV_MODE_PREFLIGHT,
                0,
                mavutil.mavlink.MAV_STATE_STANDBY)

            time.sleep(1)

        except KeyboardInterrupt:
            break


gps_udp_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14553')
gps_udp_conn.srcComponent = mavutil.mavlink.MAV_COMP_ID_USER3
gps_udp_conn.srcSystem = 1

def send_gps_heartbeat():
    print("GPS Activated")
    while True:
        try:
            gps_udp_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                mavutil.mavlink.MAV_MODE_PREFLIGHT,
                0,
                mavutil.mavlink.MAV_STATE_STANDBY)

            time.sleep(1)

        except KeyboardInterrupt:
            break


adc_udp_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14554')
adc_udp_conn.srcComponent = mavutil.mavlink.MAV_COMP_ID_USER4
adc_udp_conn.srcSystem = 1

def send_adc_heartbeat():
    print("ADC Activated")
    while True:
        try:
            adc_udp_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                mavutil.mavlink.MAV_MODE_PREFLIGHT,
                0,
                mavutil.mavlink.MAV_STATE_STANDBY)

            time.sleep(1)

        except KeyboardInterrupt:
            break

clk_udp_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14555')
clk_udp_conn.srcComponent = mavutil.mavlink.MAV_COMP_ID_USER5
clk_udp_conn.srcSystem = 1

def send_clk_heartbeat():
    print("CLK Activated")
    while True:
        try:
            clk_udp_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                mavutil.mavlink.MAV_MODE_PREFLIGHT,
                0,
                mavutil.mavlink.MAV_STATE_STANDBY)

            time.sleep(1)

        except KeyboardInterrupt:
            break


def rx_ap():
    
    tx_dref_list = [
        b'fmuas/afcs/output/elevon1',
        b'fmuas/afcs/output/elevon2',
        b'fmuas/afcs/output/yaw',
        b'fmuas/afcs/output/wing_tilt',
        b'fmuas/afcs/output/wing_stow',
        b'fmuas/afcs/output/throttle1',
        b'fmuas/afcs/output/throttle2',
        b'fmuas/afcs/output/throttle3',
        b'fmuas/afcs/output/throttle4'
    ]

    try:
        while True:
            msg = ap_udp_conn.recv_msg()
            if msg is not None and msg.get_type() in ['SERVO_OUTPUT_RAW']:
                control_data = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw, msg.servo9_raw]

                for index, dref in enumerate(tx_dref_list):
                    value = float(120*(control_data[index]-1000)/1000 - 15) if index in [0,1] else float((control_data[index]-1000)/1000)
                    msg = struct.pack('<4sxf500s', b'DREF', value, dref)
                    sock.sendto(msg, (X_PLANE_IP, UDP_PORT))
                    # if index == 1:
                    #     print('\033[1A', end='\x1b[2K')
                    #     print('recieved %d, sending %f at time %f' % (control_data[index], value, time.time()))

    except Exception as e:
        print("Error in AP thread:", str(e))

def rx_drefs(dref_list, freq=FREQ):

    for index, dref in enumerate(dref_list):
        msg = struct.pack("<4sxii400s", b'RREF', freq, index, dref[0])
        sock.sendto(msg, (X_PLANE_IP, UDP_PORT))

    try:
        print("Data streaming...\n")
        while True:
            data, addr = sock.recvfrom(2048)

            header = data[0:4]

            if header == b'RREF':

                num_values = int(len(data[5:]) / 8)

                for i in range(num_values):

                    dref_info = data[(5 + 8 * i):(5 + 8 * (i + 1))]
                    (index, value) = struct.unpack("<if", dref_info)
                    
                    if index < len(dref_list):

                        dref_list[index][1] = value

                        time_usec = time.time_ns()//1000 - time_boot

                        if index in [0,1,2,3,4,5]:
                            att_udp_conn.mav.attitude_send(time_usec//1000, DEG_TO_RAD*dref_list[0][1], DEG_TO_RAD*dref_list[1][1], DEG_TO_RAD*dref_list[2][1], DEG_TO_RAD*dref_list[3][1], DEG_TO_RAD*dref_list[4][1], DEG_TO_RAD*dref_list[5][1])
                        
                        elif index in [6]:
                            alt_udp_conn.mav.altitude_send(time_usec, 0.0, 0.0, 0.0, 0.0, FT_TO_M*dref_list[6][1], 0.0)
                        
                        elif index in [7,8,9,10]:
                            gps_udp_conn.mav.global_position_int_send(time_usec, int(1e7*dref_list[7][1]), int(1e7*dref_list[8][1]), 0, 0, int(KT_TO_CMS*dref_list[9][1]), int(KT_TO_CMS*dref_list[10][1]), 0, 0)

                        elif index in [11, 12, 13]:
                            adc_udp_conn.mav.air_data_send(time_usec//1000, KT_TO_CMS*dref_list[11][1]/100, dref_list[12][1], dref_list[13][1])

                        elif index in [14]:
                            dref_list[14][1]
                            clk_udp_conn.mav.system_time_send(time_usec, int(1000*dref_list[14][1]))


    except KeyboardInterrupt:

        for index, dref in enumerate(dref_list):
            msg = struct.pack("<4sxii400s", b'RREF', 0, index, dref[0])
            sock.sendto(msg, (X_PLANE_IP, UDP_PORT))
        print('Stopped listening for drefs')

        
        msg = struct.pack('<4sxf500s', b'DREF', 0.0, b'fmuas/python_running')
        sock.sendto(msg, (X_PLANE_IP, UDP_PORT))
        print('Lua Suspended\n')

    except Exception as e:
        print("Socket exiting due to error: ", e)

if __name__ == '__main__':
        
    a = threading.Thread(target=send_att_heartbeat)
    a.daemon = True
    a.start()
        
    b = threading.Thread(target=send_alt_heartbeat)
    b.daemon = True
    b.start()
        
    c = threading.Thread(target=send_gps_heartbeat)
    c.daemon = True
    c.start()
        
    d = threading.Thread(target=send_adc_heartbeat)
    d.daemon = True
    d.start()
        
    e = threading.Thread(target=send_clk_heartbeat)
    e.daemon = True
    e.start()
        
    ap_udp_conn.wait_heartbeat()
    print("Heartbeat from AP (system %u component %u)" % (ap_udp_conn.target_system, ap_udp_conn.target_component))
    msg = struct.pack('<4sxf500s', b'DREF', 1.0, b'fmuas/python_running')
    sock.sendto(msg, (X_PLANE_IP, UDP_PORT))

    print('XP-AP Connection Ready')
        
    main_ap = threading.Thread(target=rx_ap)
    main_ap.daemon = True
    main_ap.start()

    my_dref_list=[

        [b'fmuas/gpsins/roll', 0.0],
        [b'fmuas/gpsins/pitch', 0.0],
        [b'fmuas/gpsins/yaw', 0.0],
        [b'fmuas/gpsins/rollrate', 0.0],
        [b'fmuas/gpsins/pitchrate', 0.0],
        [b'fmuas/gpsins/yawrate', 0.0],

        [b'fmuas/radalt/altitude', 0.0],

        [b'fmuas/gpsins/latitude', 0.0],
        [b'fmuas/gpsins/longitude', 0.0],
        [b'fmuas/gpsins/xspeed', 0.0],
        [b'fmuas/gpsins/yspeed', 0.0],

        [b'fmuas/adc/ias', 0.0],
        [b'fmuas/adc/aoa', 0.0],
        [b'fmuas/adc/slip', 0.0],

        [b'fmuas/clock/time', 0.0]

    ]

    rx_drefs(my_dref_list)