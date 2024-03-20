import numpy as np
import cv2
import time

from multiprocessing import Process
from multiprocessing import Queue

# Function which runs in a separate process and loads parts of the recording
def load_next_recording(file_name, q):
    f = open(file_name, "rb")
    recording_file = np.load(f)
    while True:
        # Cycle through file names in the recording file
        for file_name in recording_file.files:
            # Load selected file
            q.put(recording_file[file_name])
            # Do nothing until recording was popped from queue by main thread
            while not q.empty():
                pass


if __name__ == "__main__":
    # Start process to load parts of the recording in the background
    q = Queue(maxsize=1)
    p = Process(
        target=load_next_recording, args=("2023_02_15_empty_belt_recording.npz", q)
    )
    p.start()

    # INITIALIZATION IS HERE
    # ----------------------

    # How fast the recording should be played
    fps = 15

    # ----------------------

    # Loop variables
    recording = None
    frame_count = 0
    frame_index = 0
    freeze_frame = False

    while True:
        # Read initial recording file
        if frame_index >= frame_count or recording is None:
            frame_index = 0
            recording = q.get()

        start_time = time.time()

        frame_height, frame_width, channel_count, frame_count = recording.shape

        # Read frames
        rgb_frame = recording[:, :, 0:3, frame_index].astype(np.uint8)
        depth_frame = recording[:, :, 3, frame_index].astype(np.uint16)

        # Colorize depth frame
        clahe = cv2.createCLAHE(clipLimit=20.0, tileGridSize=(5, 5))
        depth_frame_hist = clahe.apply(depth_frame.astype(np.uint8))
        cv2_colorized_depth = cv2.applyColorMap(depth_frame_hist, cv2.COLORMAP_JET)

        # FRAMES ARE PROCESSED HERE
        # -------------------------

        rgb_frame = cv2.resize(rgb_frame, (frame_width // 2, frame_height // 2))
        cv2_colorized_depth = cv2.resize(
            cv2_colorized_depth, (frame_width // 2, frame_height // 2)
        )

        # -------------------------

        cv2.imshow("RGB Frame", rgb_frame)
        # cv2.imshow("Depth Frame", cv2_colorized_depth)

        if not freeze_frame:
            frame_index += 1

        end_time = time.time()
        time_to_wait = (1 / fps) - (end_time - start_time)
        time.sleep(max(time_to_wait, 0))

        key = cv2.waitKey(1)
        if key == 27:  # 'Esc'
            break
        elif key == ord("f"):
            freeze_frame = not freeze_frame
        elif key == ord("n"):
            frame_index = max(frame_index - 1, 0)
        elif key == ord("m"):
            frame_index = min(frame_index + 1, frame_count)

    # Close the file
    p.kill()
