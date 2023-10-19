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

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
logging.getLogger('pymavlink').setLevel(logging.ERROR)

config = ConfigParser()
config.read('./config.ini')

systemid = 0
mav_conn = mavutil.mavlink_connection(config.get('main', 'mavlink_conn_gcs'), source_system=systemid)

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

    def boot(self) -> None:

        try:

            # Flush buffer
            while True:
                msg = mav_conn.recv_match(blocking=False)
                if msg is None:
                    break
            
            logging.warning(f"Booting #{self.target}...")

            mav_conn.mav.command_long_send(
                self.target,
                0,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_PREFLIGHT,
                1,
                10,
                0,0,0,0
            )

            time.sleep(1)

            while True:

                msg = mav_conn.recv_msg()

                if msg is not None and msg.get_type() == 'COMMAND_ACK' and msg.get_srcSystem()==self.target and msg.target_system==systemid and msg.command==mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if msg.result==mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        logging.info(f'Booted #{self.target}')
                        break
                    else:
                        logging.info(f'Boot failed on #{self.target}')
                        break

                mav_conn.mav.command_long_send(
                    self.target,
                    0,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_PREFLIGHT,
                    1,
                    10,
                    0,0,0,0
                )
                time.sleep(1)

        except KeyboardInterrupt:
            self.close()

    async def _heartbeat(self) -> None:
        while True:
            try:
                mav_conn.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            mavutil.mavlink.MAV_MODE_PREFLIGHT,
                            0,
                            mavutil.mavlink.MAV_STATE_ACTIVE
                        )
                
                await asyncio.sleep(1)

            except KeyboardInterrupt or asyncio.exceptions.CancelledError:
                self.close()

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