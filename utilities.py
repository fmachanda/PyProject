import logging
import os
import shutil
import time


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
