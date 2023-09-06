import cv2
import glob
import os
from pathlib import Path


FPS = 100

if __name__ == "__main__":

    recordings = glob.glob(os.path.join(
        "reid_demo", "recordings", "recording_*"))

    # print(recordings)

    for recording in recordings:

        print(recording)

        images = glob.glob(os.path.join(recording, "*.jpg"))
        images = sorted(images)

        if not len(images) > 0:
            continue

        test_img = cv2.imread(images[0])

        if test_img is None:
            continue

        size = test_img.shape[:2]

        fileName = os.path.split(recording)[-1]

        out = cv2.VideoWriter(
            os.path.join("reid_demo", "videos", f'{fileName}.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), FPS, (size[1], size[0]))

        last_time = float(Path(images[0]).stem)
        last_frame = cv2.imread(images[0])

        total_count = 0

        for img in images:
            current_time = float(Path(img).stem)
            delta_time = round(current_time - last_time, 2)

            repetitions = int(delta_time * FPS) - 1
            for _ in range(repetitions):
                out.write(last_frame)
                total_count += 1

            current_frame = cv2.imread(img)
            out.write(current_frame)
            last_time = current_time
            last_frame = current_frame

            total_count += 1

        print("     ", total_count)

        out.release()
