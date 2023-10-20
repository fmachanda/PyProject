from pymavlink import mavutil
import os

os.environ['MAVLINK20'] = '1'
mavutil.set_dialect('fmuas')

import threading
import math
import time
time_boot=time.time_ns()//1000

# region matplotlib

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

def init_plot(name='Output'):
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_title(name)
    return fig, ax, line

class grapher:

    def __init__(self, name='Output', deque_len=1500):
        self.fig,self.ax,self.line = init_plot(name=name)
        self.data = deque(maxlen=deque_len)

    def add(self, new):
        self.data.append(new)
    
    def graph(self):
        self.line.set_data(range(len(self.data)), self.data)
        try:
            self.ax.relim()
        except:
            pass
        self.ax.autoscale_view()
        plt.pause(0.001)

to_graph_list = []
to_graph_vars = []

# endregion

# region setup
M_TO_FT = 3.28084
CMS_TO_KT = 0.0194384

STOP = threading.Event()
PAUSE = False

FREQ = 40

DR_time = 0.0
DR_vs = 0.0
DR_xspeed = 0.0 # N/I
DR_yspeed = 0.0 # N/I
DR_track = 0.0
DR_gspeed = 0.0 # N/I
DR_vs = 0.0 # N/I
DR_vpath = 0.0
DR_roll = 0.0
DR_pitch = 0.0
DR_yaw = 0.0
DR_rollspeed = 0.0
DR_pitchspeed = 0.0 # N/I
DR_yawspeed = 0.0 # N/I
DR_alt = 0.0
DR_ias = 0.0
DR_aoa = 0.0
DR_slip = 0.0
DR_latitude = 0.0 # N/I
DR_longitude = 0.0 # N/I

DR_CMD_elevon1 = 1000
DR_CMD_elevon2 = 1000
DR_CMD_wing_tilt = 1000
DR_CMD_wing_stow = 1000
DR_CMD_throttle1 = 1000
DR_CMD_throttle2 = 1000
DR_CMD_throttle3 = 1000
DR_CMD_throttle4 = 1000
DR_CMD_rudder = 1000
DR_FLIGHT_CMD_elevon_pitch = 0.0
DR_FLIGHT_CMD_elevon_roll = 0.0
DR_FLIGHT_CMD_throttle_all = 0.23
DR_FLIGHT_CMD_aoa = 8.5
DR_FLIGHT_CMD_vpath = 0.0
DR_FLIGHT_CMD_altitude = 200.0
DR_FLIGHT_CMD_rollspeed = 0.0
DR_FLIGHT_CMD_roll = 0.0
DR_FLIGHT_CMD_heading = 0.0
DR_FLIGHT_CMD_ias = 80.0

main_udp_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14571')

att_udp_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
alt_udp_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14552')
gps_udp_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14553')
adc_udp_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14554')
clk_udp_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14555')
# endregion

# debug=grapher()

def rx_clk():

    global PAUSE
    global DR_time

    try:
        while not STOP.is_set():
            msg = clk_udp_conn.recv_msg()
            if msg is not None and msg.get_type() in ['SYSTEM_TIME']:
                RX_time = msg.time_boot_ms

                if RX_time/1000 != DR_time:
                    PAUSE = False
                    DR_time = RX_time/1000
                else:
                    PAUSE = True

    except Exception as e:
        print("Error in CLK thread:", str(e))

def rx_att():

    global DR_roll, DR_pitch, DR_yaw, DR_rollspeed, DR_pitchspeed, DR_yawspeed

    try:
        while not STOP.is_set():
            msg = att_udp_conn.recv_msg()
            if msg is not None and msg.get_type() in ['ATTITUDE']:
                RX_roll = msg.roll
                RX_pitch = msg.pitch
                RX_yaw = msg.yaw
                RX_rollspeed = msg.rollspeed
                RX_pitchspeed = msg.pitchspeed
                RX_yawspeed = msg.yawspeed

                # RX_att_timestamp = msg.time_boot_ms / 1e3

                # if STORE_att_timestamp == 0.0:
                #     DT_att = 0.0
                # else:
                #     DT_att = RX_att_timestamp - STORE_att_timestamp
                
                # STORE_att_timestamp = RX_att_timestamp

                DR_roll = math.degrees(RX_roll)
                DR_pitch = math.degrees(RX_pitch)
                DR_yaw = math.degrees(RX_yaw)
                DR_rollspeed = math.degrees(RX_rollspeed)
                DR_pitchspeed = math.degrees(RX_pitchspeed)
                DR_yawspeed = math.degrees(RX_yawspeed)
                

    except Exception as e:
        print("Error in ATT thread:", str(e))

def rx_alt():

    global DR_vs, DR_alt, DR_vpath, DR_pitch, DR_roll, DR_aoa

    STORE_alt_timestamp = 0.0

    try:
        while not STOP.is_set():
            msg = alt_udp_conn.recv_msg()
            if msg is not None and msg.get_type() in ['ALTITUDE']:
                RX_alt = msg.altitude_terrain

                RX_alt_timestamp = msg.time_usec / 1e6

                if STORE_alt_timestamp == 0.0:
                    DT_alt = 0.0
                else:
                    DT_alt = RX_alt_timestamp - STORE_alt_timestamp
                
                STORE_alt_timestamp = RX_alt_timestamp

                DR_alt = RX_alt * M_TO_FT

                if DT_alt>0:
                    DR_vs = (DR_alt - STORE_alt) / (DT_alt)

                DR_vpath = DR_pitch - (DR_aoa * math.cos(math.degrees(DR_roll)))

                STORE_alt = DR_alt
                
    except Exception as e:
        print("Error in ALT thread:", str(e))

def rx_gps():

    global DR_xspeed, DR_yspeed, DR_track, DR_gspeed, DR_latitude, DR_longitude, DR_yaw

    try:
        while not STOP.is_set():
            msg = gps_udp_conn.recv_msg()
            if msg is not None and msg.get_type() in ['GLOBAL_POSITION_INT']:
                RX_latitude = msg.lat
                RX_longitude = msg.lon
                RX_latitude_speed = msg.vx
                RX_longitude_speed = msg.vy

                # RX_gps_timestamp = msg.time_boot_ms / 1e3

                # if STORE_gps_timestamp == 0.0:
                #     DT_gps = 0.0
                # else:
                #     DT_gps = (RX_gps_timestamp - STORE_gps_timestamp)/1000

                # STORE_gps_timestamp = RX_gps_timestamp

                if RX_latitude_speed > 0 and RX_longitude_speed >= 0:
                    DR_track = math.degrees(math.atan(RX_longitude_speed / RX_latitude_speed))
                elif RX_latitude_speed <= 0 and RX_longitude_speed > 0:
                    DR_track = 90.0 - math.degrees(math.atan(RX_latitude_speed / RX_longitude_speed))
                elif RX_latitude_speed < 0 and RX_longitude_speed <= 0:
                    DR_track = 180.0 + math.degrees(math.atan(RX_longitude_speed / RX_latitude_speed))
                elif RX_latitude_speed >= 0 and RX_longitude_speed < 0:
                    DR_track = 270.0 - math.degrees(math.atan(RX_latitude_speed / RX_longitude_speed))
                else:
                    DR_track = 0.0
                
                DR_gspeed = CMS_TO_KT * math.sqrt(RX_latitude_speed**2 + RX_longitude_speed**2)
                
                delta = DR_yaw - DR_track
                if delta < 0: delta += 360.0

                DR_xspeed = CMS_TO_KT * math.cos(delta) * DR_gspeed
                DR_yspeed = CMS_TO_KT * math.sin(delta) * DR_gspeed

                DR_latitude = RX_latitude
                DR_longitude = RX_longitude

    except Exception as e:
        print("Error in GPS thread:", str(e))

def rx_adc():

    global DR_ias, DR_aoa, DR_slip

    try:
        while not STOP.is_set():
            msg = adc_udp_conn.recv_msg()

            if msg is not None and msg.get_type() in ['AIR_DATA']:
                RX_ias = msg.ias
                RX_aoa = msg.aoa
                RX_slip = msg.slip

                # RX_adc_timestamp = msg.time_boot_ms / 1e3

                # if STORE_adc_timestamp == 0.0:
                #     DT_adc = 0.0
                # else:
                #     DT_adc = RX_adc_timestamp - STORE_adc_timestamp
                
                # STORE_adc_timestamp = RX_adc_timestamp

                DR_ias = 100*CMS_TO_KT * RX_ias
                DR_aoa = RX_aoa
                DR_slip = RX_slip

    except Exception as e:
        print("Error in ADC thread:", str(e))

def tx_srv():

    try:
        while not STOP.is_set():
            time_usec = time.time_ns()//1000 - time_boot
            main_udp_conn.mav.servo_output_raw_send(time_usec, 0, DR_CMD_elevon1, DR_CMD_elevon2, DR_CMD_rudder, DR_CMD_wing_tilt, DR_CMD_wing_stow, DR_CMD_throttle1, DR_CMD_throttle2, DR_CMD_throttle3, DR_CMD_throttle4)
    except Exception as e:
        print("Error in SRV thread:", str(e))

# region misc
def clamp(minn, n, maxn):
    return max(min(maxn, n), minn)

def send_main_heartbeat():
    print("AUTOPILOT Activated")
    while True:
        try:
            main_udp_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_VTOL_TILTWING,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                mavutil.mavlink.MAV_MODE_PREFLIGHT,
                0,
                mavutil.mavlink.MAV_STATE_STANDBY)

            time.sleep(1)

        except KeyboardInterrupt:
            break

def wait_att():
    att_udp_conn.wait_heartbeat()
    print("Heartbeat from ATT (system %u component %u)" % (att_udp_conn.target_system, att_udp_conn.target_component))

def wait_alt():
    alt_udp_conn.wait_heartbeat()
    print("Heartbeat from ALT (system %u component %u)" % (alt_udp_conn.target_system, alt_udp_conn.target_component))

def wait_gps():
    gps_udp_conn.wait_heartbeat()
    print("Heartbeat from GPS (system %u component %u)" % (gps_udp_conn.target_system, gps_udp_conn.target_component))
    
def wait_adc():
    adc_udp_conn.wait_heartbeat()
    print("Heartbeat from ADC (system %u component %u)" % (adc_udp_conn.target_system, adc_udp_conn.target_component))

def wait_clk():
    clk_udp_conn.wait_heartbeat()
    print("Heartbeat from CLK (system %u component %u)" % (clk_udp_conn.target_system, clk_udp_conn.target_component))
# endregion

class PID():

    def __init__(self, kp=None, ti=None, td=None, ki=None, kd=None, integral_limit=None, minimum=None, maximum=None):
        
        if kp is not None:
            self.kp = kp
        
        if ti is not None:
            self.ti = ti

            if ti != 0.0:
                self.ki = self.kp / ti
            else:
                self.ki = 0.0
        
        if td is not None:
            self.td = td
            self.kd = self.kp * td
        
        if ki is not None:
            self.ki = ki

            if ki != 0.0:
                self.ti = self.kp / ki
            else:
                self.ti = 0.0
        
        if kd is not None:
            self.kd = kd
            
            if self.kp != 0.0:
                self.td = kd / self.kp
            else:
                self.td = 0.0
                
        self.integral_limit = integral_limit
        self.minimum = minimum
        self.maximum = maximum

        self.proportional = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.error = 0.0
        self.output = 0.0

    def set(self, kp=None, ti=None, td=None, ki=None, kd=None, integral_limit='place', minimum='place', maximum='place'):
        
        if kp is not None:
            self.kp = kp
        
        if ti is not None:
            self.ti = ti

            if ti != 0.0:
                self.ki = self.kp / ti
            else:
                self.ki = 0.0
        
        if td is not None:
            self.td = td
            self.kd = self.kp * td
        
        if ki is not None:
            self.ki = ki

            if ki != 0.0:
                self.ti = self.kp / ki
            else:
                self.ti = 0.0
        
        if kd is not None:
            self.kd = kd
            
            if self.kp != 0.0:
                self.td = kd / self.kp
            else:
                self.td = 0.0

        if integral_limit != 'place':
            self.integral_limit = integral_limit

        if minimum != 'place':
            self.minimum = minimum

        if maximum != 'place':
            self.maximum = maximum

    def reset(self):
        self.proportional = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.error = 0.0
        self.output = 0.0

    def cycle(self, value, setpoint, time_step):

        time_step = max(1e-6, time_step)

        error = setpoint - value

        self.proportional = error

        self.integral += (self.ki * error * time_step)

        if self.integral_limit is not None:
            self.integral = min(self.integral,  (self.integral_limit / self.kp))
            self.integral = max(self.integral, -(self.integral_limit / self.kp))

        self.derivative = self.td * (error - self.error) / time_step

        output = (self.proportional + self.integral + self.derivative) * self.kp
        
        self.error = error

        if self.minimum is not None:
            output = max(output, self.minimum)

        if self.maximum is not None:
            output = min(output, self.maximum)

        self.output = output
        return self.output

PID_FLIGHT_vpath_from_altitude = PID(kp=0.1, td=1.0, ti=2.0, integral_limit=0.2, maximum=4.0, minimum=-3.0)
PID_FLIGHT_aoa_from_vpath = PID(kp=1.0, ti=2.5, td=0.03, integral_limit=10.0, maximum=16.0, minimum=-3.0)

PID_FLIGHT_control_from_aoa = PID(kp=-0.1, ti=-0.05, td=0.0, maximum=0.0, minimum=-15.0)
PID_FLIGHT_rudder_from_slip = PID(kp=-0.15, ti=-0.1, td=0.6, maximum=1.0, minimum=-1.0)
PID_FLIGHT_throttleall_from_ias = PID(kp=0.04, ti=0.2, td=0.0, integral_limit=0.25, maximum=0.25, minimum=0.02)

PID_FLIGHT_roll_from_heading = PID(kp=1.0, ti=1.0, td=0.0, integral_limit=2.0, maximum=30.0, minimum=-30.0)

PID_FLIGHT_rollspeed_from_roll = PID(kp=5.0, ti=2.5, td=0.01, integral_limit=10.0, maximum=10.0, minimum=-10.0)
PID_FLIGHT_control_from_rollrate = PID(kp=0.11, ti=0.16, td=0.06, maximum=5.0, minimum=-5.0)

##################################################

def input_thread():

    global STOP
    global to_graph_vars

    CLI=True
    global_vars = globals()

    while CLI and not STOP.is_set():
        try:
                
            try:
                original = input('UAS-MAIN CLI: ')
            except:
                STOP.set()
                break
            
            args = original.split()
            try: cmd = args.pop(0).lower()
            except: cmd = None

            if cmd in ['halt','stop']:

                STOP.set()
                print("\nMain program terminated by user")
        
                att_udp_conn.close()  # Close the MAVLink connection
                print("ATT thread terminated")

                alt_udp_conn.close()  # Close the MAVLink connection
                print("ALT thread terminated")

                gps_udp_conn.close()  # Close the MAVLink connection
                print("GPS thread terminated")

                adc_udp_conn.close()  # Close the MAVLink connection
                print("ADC thread terminated")

                clk_udp_conn.close()  # Close the MAVLink connection
                print("CLK thread terminated")

                main_udp_conn.close()  # Close the MAVLink connection
                print("SRV thread terminated\n")
            
            elif cmd == 'set':
                try:
                    args[1]=float(args[1])
                    if args[0] in global_vars.keys(): global_vars[args[0]] = args[1]
                except Exception as e:
                    print("Error: ", e)
            
            elif cmd == 'print':
                if ('.' in args[0]) and (args[0].split('.')[0] in global_vars):
                    x=global_vars[args[0].split('.')[0]]
                    y=args[0].split('.')[1]
                    exec('print(x.%s)' % (y), global_vars, locals())
                elif args[0] in global_vars.keys():
                    print(global_vars[args[0]])
                else: print("Error: Unrecognized variable \'%s\'" % args[0])
            
            elif cmd == 'printloop':
                if ('.' in args[0]) and (args[0].split('.')[0] in global_vars):
                    x=global_vars[args[0].split('.')[0]]
                    y=args[0].split('.')[1]
                    while True: exec('print(x.%s)' % (y), global_vars, locals())
                elif args[0] in global_vars.keys():
                    while True: print(global_vars[args[0]])
                else: print("Error: Unrecognized variable \'%s\'" % args[0])

            elif cmd == 'graph':
                if (args[0] not in to_graph_vars) and (args[0].split('.')[0] in global_vars.keys()): to_graph_vars.append(args[0])
                elif args[0] in to_graph_vars: print("Error: Variable \'%s\' already in graph list" % args[0])
                else: print("Error: Unrecognized variable \'%s\'" % args[0])

            elif cmd == 'exec':
                try:
                    exec(original[5:], global_vars, locals())
                except Exception as e:
                    print("Error: ", e)

            elif cmd == 'hide':
                CLI = False

            else:
                print("Error: Unrecognized command \'%s\'" % cmd)

        except Exception as e:
            if not STOP.is_set(): print('\nError: ', e)

def run():

    global STOP

    global to_graph_list
    
    global DR_CMD_elevon1, DR_CMD_elevon2, DR_CMD_wing_tilt, DR_CMD_wing_stow, DR_CMD_throttle1, DR_CMD_throttle2, DR_CMD_throttle3, DR_CMD_throttle4, DR_CMD_rudder
    global DR_FLIGHT_CMD_elevon_pitch, DR_FLIGHT_CMD_elevon_roll, DR_FLIGHT_CMD_throttle_all
    global DR_FLIGHT_CMD_aoa, DR_FLIGHT_CMD_vpath, DR_FLIGHT_CMD_altitude, DR_FLIGHT_CMD_rollspeed, DR_FLIGHT_CMD_heading, DR_FLIGHT_CMD_roll, DR_FLIGHT_CMD_ias

    # region init

    waitatt = threading.Thread(target=wait_att)
    waitatt.start()
    waitalt = threading.Thread(target=wait_alt)
    waitalt.start()
    waitgps = threading.Thread(target=wait_gps)
    waitgps.start()
    waitadc = threading.Thread(target=wait_adc)
    waitadc.start()
    waitclk = threading.Thread(target=wait_clk)
    waitclk.start()

    print("Waiting for systems...")
    waitatt.join()
    waitalt.join()
    waitgps.join()
    waitadc.join()
    waitclk.join()
    print('All systems online')
    
    a = threading.Thread(target=rx_att)
    a.daemon = True
    a.start()
    
    b = threading.Thread(target=rx_alt)
    b.daemon = True
    b.start()
    
    c = threading.Thread(target=rx_gps)
    c.daemon = True
    c.start()
    
    d = threading.Thread(target=rx_adc)
    d.daemon = True
    d.start()
    
    e = threading.Thread(target=rx_clk)
    e.daemon = True
    e.start()
        
    z = threading.Thread(target=send_main_heartbeat)
    z.daemon = True
    z.start()
    
    y = threading.Thread(target=tx_srv)
    y.daemon = True
    y.start()
    
    inputs = threading.Thread(target=input_thread)
    inputs.daemon = True
    inputs.start()
    
    # endregion
    
    try:

        DR_time_stored = DR_time if DR_time>0 else 0.0

        while not STOP.is_set():

            time.sleep(1/FREQ)

            if not PAUSE:

                dt = DR_time - DR_time_stored

                delta_heading = DR_FLIGHT_CMD_heading - DR_track
                if delta_heading > 180:
                    delta_heading = delta_heading - 360
                elif delta_heading <= -180:
                    delta_heading = delta_heading + 360
                delta_heading = -delta_heading
                
                # DR_FLIGHT_CMD_throttle_all = PID_FLIGHT_throttleall_from_ias.cycle(DR_ias, DR_FLIGHT_CMD_ias, dt)

                # DR_FLIGHT_CMD_vpath = PID_FLIGHT_vpath_from_altitude.cycle(DR_alt, DR_FLIGHT_CMD_altitude, dt)
                # DR_FLIGHT_CMD_aoa = PID_FLIGHT_aoa_from_vpath.cycle(DR_vpath, DR_FLIGHT_CMD_vpath, dt)
                DR_FLIGHT_CMD_elevon_pitch = PID_FLIGHT_control_from_aoa.cycle(DR_aoa, DR_FLIGHT_CMD_aoa, dt)

                # DR_FLIGHT_CMD_roll = PID_FLIGHT_roll_from_heading.cycle(delta_heading, 0.0, dt)
                # DR_FLIGHT_CMD_rollspeed = PID_FLIGHT_rollspeed_from_roll.cycle(DR_roll, DR_FLIGHT_CMD_roll, dt)
                # DR_FLIGHT_CMD_elevon_roll = PID_FLIGHT_control_from_rollrate.cycle(DR_rollspeed, DR_FLIGHT_CMD_rollspeed, dt)
                
                # DR_CMD_rudder = int((PID_FLIGHT_rudder_from_slip.cycle(DR_slip, 0.0, dt)*500) + 1500)

                DR_time_stored = DR_time

            DR_CMD_elevon1 = int(1000*clamp(0,(DR_FLIGHT_CMD_elevon_pitch + DR_FLIGHT_CMD_elevon_roll + 15),120)/120 + 1000)
            DR_CMD_elevon2 = int(1000*clamp(0,(DR_FLIGHT_CMD_elevon_pitch - DR_FLIGHT_CMD_elevon_roll + 15),120)/120 + 1000)

            DR_CMD_throttle1 = int(1000 + DR_FLIGHT_CMD_throttle_all*1000)
            DR_CMD_throttle2 = int(1000 + DR_FLIGHT_CMD_throttle_all*1000)
            DR_CMD_throttle3 = int(1000 + DR_FLIGHT_CMD_throttle_all*1000)
            DR_CMD_throttle4 = int(1000 + DR_FLIGHT_CMD_throttle_all*1000)

            # debug.graph()

            if len(to_graph_vars) != len(to_graph_list):
                to_graph_list.append(grapher(name=to_graph_vars[-1]))

            if len(to_graph_list)>0:
                for index, graph in enumerate(to_graph_list):
                    if not PAUSE and ('.' in to_graph_vars[index]):
                        x=to_graph_vars[index].split('.')[0]
                        y=to_graph_vars[index].split('.')[1]
                        exec("graph.add(%s.%s)" % (x,y), globals(), locals())
                    elif not PAUSE: 
                        exec("graph.add(%s)" % to_graph_vars[index], globals(), locals())

                    graph.graph()
            
    except KeyboardInterrupt:

        STOP.set()
        print("\n\nMain program terminated by user")
        
        att_udp_conn.close()  # Close the MAVLink connection
        print("ATT thread terminated")

        alt_udp_conn.close()  # Close the MAVLink connection
        print("ALT thread terminated")

        gps_udp_conn.close()  # Close the MAVLink connection
        print("GPS thread terminated")

        adc_udp_conn.close()  # Close the MAVLink connection
        print("ADC thread terminated")

        clk_udp_conn.close()  # Close the MAVLink connection
        print("CLK thread terminated")

        main_udp_conn.close()  # Close the MAVLink connection
        print("SRV thread terminated\n")

if __name__ == '__main__':

    run()