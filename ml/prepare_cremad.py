from pathlib import Path
import wave
import numpy as np

# CREMA-D original
SOURCE = Path.home() / "Documents/datasets/CREMA-D/AudioWAV"

# Dataset que será usado pelo treinamento
DEST = Path(__file__).parent / "dataset/processed"

SAMPLE_RATE = 16000
WINDOW_SAMPLES = 16000
HOP_SAMPLES = 8000       # 0.5 s
MIN_LAST_SAMPLES = 12000 # não usar pedaços muito curtos

EMOTION_MAP = {
    "NEU": "normal",
    "HAP": "positive",
    "SAD": "distress",
    "FEA": "danger",
    "ANG": "danger",
}


def read_wav(path):
    with wave.open(str(path), "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        frames = wf.readframes(wf.getnframes())

    if channels != 1:
        raise ValueError(f"{path}: esperado mono, recebido {channels} canais")

    if sample_width != 2:
        raise ValueError(
            f"{path}: esperado PCM16, recebido {sample_width * 8} bits"
        )

    if sample_rate != SAMPLE_RATE:
        raise ValueError(
            f"{path}: esperado 16000 Hz, recebido {sample_rate} Hz"
        )

    return np.frombuffer(frames, dtype=np.int16)


def save_wav(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)

    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(samples.astype(np.int16).tobytes())


def normalize_length(samples):
    if len(samples) >= WINDOW_SAMPLES:
        return samples[:WINDOW_SAMPLES]

    result = np.zeros(WINDOW_SAMPLES, dtype=np.int16)
    result[:len(samples)] = samples
    return result


def main():
    total_original = 0
    total_segments = 0

    counts = {
        "normal": 0,
        "positive": 0,
        "distress": 0,
        "danger": 0,
    }

    for wav_path in sorted(SOURCE.glob("*.wav")):
        parts = wav_path.stem.split("_")

        if len(parts) < 4:
            print(f"Ignorando nome inválido: {wav_path.name}")
            continue

        actor = parts[0]
        emotion = parts[2]

        # DIS e qualquer outra classe ficam fora da v1
        if emotion not in EMOTION_MAP:
            continue

        target_class = EMOTION_MAP[emotion]

        try:
            samples = read_wav(wav_path)
        except Exception as e:
            print(f"ERRO {wav_path.name}: {e}")
            continue

        total_original += 1

        segment_index = 0

        for start in range(0, len(samples), HOP_SAMPLES):
            end = start + WINDOW_SAMPLES
            segment = samples[start:end]

            if len(segment) < MIN_LAST_SAMPLES:
                break

            segment = normalize_length(segment)

            output_name = (
                f"{actor}__{wav_path.stem}"
                f"__seg{segment_index:02d}.wav"
            )

            output_path = DEST / target_class / output_name

            save_wav(output_path, segment)

            counts[target_class] += 1
            total_segments += 1
            segment_index += 1

    print()
    print("================================")
    print("CREMA-D preparado")
    print("================================")
    print(f"Arquivos originais usados: {total_original}")
    print(f"Segmentos de 1 s:          {total_segments}")
    print()

    for cls, count in counts.items():
        print(f"{cls:12s}: {count}")

    print()
    print(f"Destino: {DEST.resolve()}")


if __name__ == "__main__":
    main()


