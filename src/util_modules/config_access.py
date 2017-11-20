import json
import os

config_file_path = os.path.join(os.path.dirname(__file__), '../../resources/configurations/leslie.json')

with open(config_file_path, 'r') as f:
    config = json.load(f)

"""
    Static strings in order to access the keys
"""
KEY_VERSION = 'version'
KEY_CAMERA_INDEX_TABLE = 'camera_index_table'
KEY_CAMERA_INDEX_FACE = 'camera_index_face'
KEY_TABLE_MIN_BGR_THRESHOLD = 'table_min_bgr_threshold'
KEY_TABLE_MAX_BGR_THRESHOLD = 'table_max_bgr_threshold'
KEY_CHANGE_MIN_TABLE_WIDTH = 'change_min_table_width'
KEY_CHANGE_FRAME_QUEUE_SIZE = 'change_frame_queue_size'
KEY_CHANGE_MIN_CONTOUR_AREA = 'change_min_contour_area'
KEY_CHANGE_SIGNIFICANT_CHANGE_THRESHOLD = 'change_significant_change_threshold'
KEY_VIDEO_OUTPUT_DIR = 'video_output_dir'
KEY_FACE_FRAME_THRESHOLD = 'face_frame_threshold'
KEY_MIN_LOCKED_TIME_SECONDS = 'min_locked_time_seconds'
KEY_EMAIL_ADDRESS = 'email_address'
KEY_EMAIL_PASSWORD = 'email_password'
KEY_EMAIL_SERVER = 'email_server'
KEY_EMAIL_PORT = 'email_port'
KEY_THRESHOLD_FRAME = 'threshold_frame'
KEY_ALARM_COUNT_THRESHOLD = 'alarm_count_threshold'
KEY_LOCKING_FRAME_COUNT = 'locking_frame_count'
KEY_MIN_PIXEL_CHANGE_COUNT = 'min_pixel_change_count'
KEY_OFFSET = 'offset'
KEY_MIN_HEIGHT_CROPPED = 'min_height_cropped'
KEY_MAX_HEIGHT_CROPPED = 'max_height_cropped'
KEY_MIN_WIDTH_CROPPED = 'min_width_cropped'
KEY_MAX_WIDTH_CROPPED = 'max_height_cropped'

def get_config(key):
    """
    Retrieve a specific configuration
    :param key: An identifier for the specific requested configuration
    :type key: String
    :return: The configuration according to the key from the JSON file.
    :rtype: T
    """
    return config[key]
