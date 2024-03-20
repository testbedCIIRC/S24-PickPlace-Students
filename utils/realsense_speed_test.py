import numpy as np

recording = []
with open("2023_02_15_empty_belt_recording.npy", "rb") as f:
    recording = np.load(f)
    print("Recording loaded")

print("Saving recording...")
np.savez_compressed(
    "2023_02_15_empty_belt_recording.npz",
    rec_1=recording[:, :, :, 0:100],
    rec_2=recording[:, :, :, 101:200],
    rec_3=recording[:, :, :, 201:300],
    rec_4=recording[:, :, :, 301:400],
    rec_5=recording[:, :, :, 401:500],
)
print("Recording compressed and saved")
