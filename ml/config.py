from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATASET_RAW = ROOT / "dataset" / "raw"
DATASET_PROCESSED = ROOT / "dataset" / "processed"
BUILD_DIR = ROOT / "build"

SAMPLE_RATE = 16_000
CLIP_SECONDS = 1.0
CLIP_SAMPLES = int(SAMPLE_RATE * CLIP_SECONDS)

FRAME_LENGTH = 480       # 30 ms @ 16 kHz
FRAME_STEP = 320         # 20 ms @ 16 kHz
FFT_LENGTH = 512
N_MELS = 32
LOWER_HZ = 80.0
UPPER_HZ = 7_600.0

CLASSES = [
    "background",
    "normal_speech",
    "scream_panic",
    "cry_groan",
    "laugh_positive",
]

RANDOM_SEED = 1337
