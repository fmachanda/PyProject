import os
import asyncio
# import threading
import logging
# import numpy as np
# import math
import time
from configparser import ConfigParser

os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil
m = mavutil.mavlink

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
logging.getLogger('pymavlink').setLevel(logging.ERROR)

config = ConfigParser()
config.read('./config.ini')

systemid = 0
mav_conn = mavutil.mavlink_connection(config.get('mavlink', 'mavlink_conn_gcs'), source_system=systemid)

ids = []

import key
mav_conn.setup_signing(key.KEY.encode('utf-8'))

class Connect:

    def __repr__(self) -> str:
        self.close()
        return 'Must store Connect instance'

    def __init__(self, target:int, auto:bool=False, timeout_cycles:int=5) -> None:
        
        self.target = None
        assert 0<target<256 and isinstance(target, int) and target!=systemid and target not in ids, 'System ID target must be unique UINT8'

        try:

            # Flush buffer
            while True:
                msg = mav_conn.recv_match(blocking=False)
                if msg is None:
                    break
            
            logging.warning(f"Connecting to #{target}...")

            mav_conn.mav.change_operator_control_send(target, 0, 0, key.KEY.encode('utf-8'))

            time.sleep(1)

            for i in range(timeout_cycles):

                msg = mav_conn.recv_msg()

                if msg is not None and msg.get_type() == 'CHANGE_OPERATOR_CONTROL_ACK' and msg.get_srcSystem()==target and msg.gcs_system_id==systemid:
                    if msg.ack==0:
                        self.target = target        
                        ids.append(self.target)
                        logging.info(f'Connected to #{self.target}')
                        break
                    elif msg.ack in [1,2]:
                        logging.info(f'Bad connection key for #{target}')
                        break
                    elif msg.ack==3:
                        logging.info(f'#{target} is connected to another GCS')
                        break

                mav_conn.mav.change_operator_control_send(target, 0, 0, key.KEY.encode('utf-8'))
                time.sleep(1)

                if i >= timeout_cycles - 1: logging.warning('Connection failed (timeout)')

            # asyncio.create_task(self._heartbeat())

        except KeyboardInterrupt:
            self.close()
    
    async def _command(self, name:str, *params, acknowledge:bool=True, period:float=1) -> None:

        params_full = (list(params)+[0]*7)[:7]
        command = eval(f'm.MAV_CMD_{name}')

        try:

            # Flush buffer
            while True:
                msg = mav_conn.recv_match(blocking=False)
                if msg is None:
                    break
            
            logging.warning(f"{name} sent to #{self.target}...")
            logging.debug(f'    {name} params: {params}')

            mav_conn.mav.command_long_send(
                self.target,
                0,
                command,
                0,
                params_full[0],
                params_full[1],
                params_full[2],
                params_full[3],
                params_full[4],
                params_full[5],
                params_full[6],
            )

            await asyncio.sleep(period)

            if acknowledge:

                while True:

                    msg = mav_conn.recv_msg()

                    if msg is not None and msg.get_type() == 'COMMAND_ACK' and msg.get_srcSystem()==self.target and msg.target_system==systemid and msg.command==command:
                        if msg.result in [m.MAV_RESULT_ACCEPTED, m.MAV_RESULT_IN_PROGRESS]:
                            logging.info(f'{name} #{self.target}')
                            break
                        elif msg.result==m.MAV_RESULT_TEMPORARILY_REJECTED:
                            pass
                        elif msg.result==m.MAV_RESULT_COMMAND_INT_ONLY:
                            self._command_int(name, *params)
                            break
                        else:
                            logging.info(f'{name} failed on #{self.target}')
                            break

                    mav_conn.mav.command_long_send(
                        self.target,
                        0,
                        command,
                        0,
                        params_full[0],
                        params_full[1],
                        params_full[2],
                        params_full[3],
                        params_full[4],
                        params_full[5],
                        params_full[6],
                    )

                    await asyncio.sleep(period)

        except KeyboardInterrupt:
            self.close()

    async def _command_int(self, name:str, *params, acknowledge:bool=True, period:float=1, frame:int=m.MAV_FRAME_GLOBAL) -> None:

            params_full = (list(params)+[0]*7)[:7]
            command = eval(f'm.MAV_CMD_{name}')

            try:

                # Flush buffer
                while True:
                    msg = mav_conn.recv_match(blocking=False)
                    if msg is None:
                        break
                
                logging.warning(f"{name} sent to #{self.target}...")
                logging.debug(f'    {name} params: {params}')

                mav_conn.mav.command_int_send(
                    self.target,
                    0,
                    frame,
                    command,
                    0,0,
                    params_full[0],
                    params_full[1],
                    params_full[2],
                    params_full[3],
                    params_full[4],
                    params_full[5],
                    params_full[6],
                )

                await asyncio.sleep(period)

                if acknowledge:

                    while True:

                        msg = mav_conn.recv_msg()

                        if msg is not None and msg.get_type() == 'COMMAND_ACK' and msg.get_srcSystem()==self.target and msg.target_system==systemid and msg.command==command:
                            if msg.result in [m.MAV_RESULT_ACCEPTED, m.MAV_RESULT_IN_PROGRESS]:
                                logging.info(f'{name} #{self.target}')
                                break
                            elif msg.result==m.MAV_RESULT_TEMPORARILY_REJECTED:
                                pass
                            elif msg.result==m.MAV_RESULT_COMMAND_INT_ONLY:
                                self._command(name, *params)
                                break
                            else:
                                logging.info(f'{name} failed on #{self.target}')
                                break

                        mav_conn.mav.command_int_send(
                            self.target,
                            0,
                            frame,
                            command,
                            0,0,
                            params_full[0],
                            params_full[1],
                            params_full[2],
                            params_full[3],
                            params_full[4],
                            params_full[5],
                            params_full[6],
                        )

                        await asyncio.sleep(period)

            except KeyboardInterrupt:
                self.close()


    def boot(self) -> None:
        asyncio.run(self._command('DO_SET_MODE', m.MAV_MODE_PREFLIGHT, 1, 10))

    def set_mode(self, mode:int, custom_mode:int, custom_submode:int) -> None:
        asyncio.run(self._command('DO_SET_MODE', mode, custom_mode, custom_submode))

    def set_alt(self, alt:float) -> None:
        asyncio.run(self._command('DO_CHANGE_ALTITUDE', alt, m.MAV_FRAME_GLOBAL_TERRAIN_ALT))

    def set_speed(self, airspeed:float) -> None:
        asyncio.run(self._command('DO_CHANGE_SPEED', m.SPEED_TYPE_AIRSPEED, airspeed, -1))

    def reposition(self, latitude:float, longitude:float, altitude:float, speed:float=-1, radius:float=0, yaw:float=1):
        asyncio.run(self._command_int('DO_REPOSITION', speed, 0, radius, yaw, latitude, longitude, altitude))

    def f_takeoff(self, latitude:float, longitude:float, altitude:float, yaw:float=float('nan'), pitch:float=10):
        asyncio.run(self._command_int('NAV_TAKEOFF', pitch, 0,0, yaw, latitude, longitude, altitude))

    def v_takeoff(self, latitude:float, longitude:float, altitude:float, transit_heading:float=m.VTOL_TRANSITION_HEADING_TAKEOFF, yaw:float=float('nan')):
        asyncio.run(self._command_int('NAV_TAKEOFF', 0, transit_heading, 0, yaw, latitude, longitude, altitude))

    async def _heartbeat(self) -> None:
        while True:
            try:
                mav_conn.mav.heartbeat_send(
                            m.MAV_TYPE_GCS,
                            m.MAV_AUTOPILOT_INVALID,
                            m.MAV_MODE_PREFLIGHT,
                            0,
                            m.MAV_STATE_ACTIVE
                        )
                
                await asyncio.sleep(1)

            except KeyboardInterrupt: 
                self.close()
            except asyncio.exceptions.CancelledError:
                self.close()
                raise

    def close(self) -> None:

        # Flush buffer
        while True:
            msg = mav_conn.recv_match(blocking=False)
            if msg is None:
                break

        try:
            ids.remove(self.target)
            mav_conn.mav.change_operator_control_send(self.target, 1, 0, key.KEY.encode('utf-8'))
            logging.warning('Closing GCS')
        except ValueError:
            logging.error('Connection not open')

if __name__=='__main__':
    os.system('cls' if os.name == 'nt' else 'clear')
    x = Connect(132)