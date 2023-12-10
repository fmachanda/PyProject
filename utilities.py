"""Utilities

DEVELOPER ONLY
"""

import logging
import os
import shutil
import time


class PIDValues:
    def __init__(self, id: int, sp_name: str | None = None, kp: float | None =None, ti: float | None =None, td: float | None =None, sp: float | None =None):
        self.id = id
        self.sp_name = sp_name
        self.kp = kp
        self.ti = ti
        self.td = td
        self.sp = sp


pid_tune_map = {
    'pid_any': PIDValues(0),
    'pidf_alt_vpa': PIDValues(1, '_sp_altitude'),
    'pidf_vpa_aoa': PIDValues(2, '_spf_vpath'),
    'pidf_aoa_out': PIDValues(3, '_spf_aoa'),
    'pidf_dyw_rol': PIDValues(4),
    'pidf_rol_rls': PIDValues(5, '_spf_roll'),
    'pidf_rls_out': PIDValues(6, '_spf_rollspeed'),
    'pidf_ias_thr': PIDValues(7, '_spf_ias'),
    'pidv_xdp_xsp': PIDValues(8, '_spf_vpath'),
    'pidv_xsp_rol': PIDValues(9, '_spv_xspeed'),
    'pidv_rol_rls': PIDValues(10, '_spv_roll'),
    'pidv_rls_out': PIDValues(11, '_spv_rollspeed'),
    'pidv_ydp_ysp': PIDValues(12),
    'pidv_ysp_pit': PIDValues(13, '_spv_yspeed'),
    'pidv_pit_pts': PIDValues(14, '_spv_pitch'),
    'pidv_pts_out': PIDValues(15, '_spv_pitchspeed'),
    'pidv_alt_vsp': PIDValues(16, '_sp_altitude'),
    'pidv_vsp_out': PIDValues(17, '_spv_vs'),
    'pidv_dyw_yws': PIDValues(18),
    'pidv_yws_out': PIDValues(19, '_spv_yawspeed'),
    'pidt_dep_out': PIDValues(20),
    'pidt_arr_out': PIDValues(21),
}

pid_tune_map_names = {pid.id: name for name, pid in pid_tune_map.items()}
pid_tune_map_sps = {pid.id: pid.sp_name for pid in pid_tune_map.values()}


def watcher() -> None:
    try:
        xp_path = r'C:\X-Plane 12\Output\screenshots'
        destination_dir = r'./stored_images'

        logging.warning("Starting watcher cycle...")
        while True:
            if not os.path.exists(destination_dir):
                os.makedirs(destination_dir)

            previous_file_list = [f for f in os.listdir(xp_path) if os.path.isfile(os.path.join(xp_path, f))]

            time.sleep(1)

            new_file_list = [f for f in os.listdir(xp_path) if os.path.isfile(os.path.join(xp_path, f))]
            file_diff = [x for x in new_file_list if x not in previous_file_list]
            previous_file_list = new_file_list

            if len(file_diff) != 0:
                logging.warning(f"Detected file_diff: {file_diff}")
                for f in file_diff:
                    ready = False
                    last_modified = os.stat(os.path.join(xp_path, f)).st_mtime
                    time.sleep(1)
                    while not ready:
                        time.sleep(1)
                        new_last_modified = os.stat(os.path.join(xp_path, f)).st_mtime
                        ready = (last_modified == new_last_modified)
                        last_modified = new_last_modified
                        logging.info('waiting...')
                    logging.info("moving")
                    if not os.path.exists(destination_dir):
                        os.makedirs(destination_dir)
                    file_path = os.path.join(xp_path, f)
                    try:
                        shutil.move(file_path, destination_dir)
                        logging.warning(f"Moving file {file_path} to {destination_dir}")
                    except shutil.Error:
                        logging.error(f"Error moving file {file_path} to {destination_dir}")
                    except PermissionError:
                        logging.error(f"Error moving file {file_path} to {destination_dir}")
                    finally:
                        pass
    except KeyboardInterrupt:
        logging.warning("Closed watcher cycle")
