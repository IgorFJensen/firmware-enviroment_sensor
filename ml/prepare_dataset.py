#!/usr/bin/env python3
"""Converte os WAVs brutos para mono, 16 kHz, PCM16 e segmentos de 1 segundo."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import soundfile as sf
from scipy.signal import resample_poly

from config import CLASSES, CLIP_SAMPLES, DATASET_PROCESSED, DATASET_RAW, SAMPLE_RATE


def to_mono(audio: np.ndarray) -> np.ndarray:
    if audio.ndim == 1:
        return audio.astype(np.float32)
    return np.mean(audio, axis=1, dtype=np.float32)


def resample(audio: np.ndarray, src_rate: int) -> np.ndarray:
    if src_rate == SAMPLE_RATE:
        return audio.astype(np.float32, copy=False)
    gcd = math.gcd(src_rate, SAMPLE_RATE)
    up = SAMPLE_RATE // gcd
    down = src_rate // gcd
    return resample_poly(audio, up, down).astype(np.float32)


def segment_one_second(audio: np.ndarray) -> list[np.ndarray]:
    if len(audio) <= CLIP_SAMPLES:
        out = np.zeros(CLIP_SAMPLES, dtype=np.float32)
        start = (CLIP_SAMPLES - len(audio)) // 2
        out[start:start + len(audio)] = audio
        return [out]

    segments = []
    for start in range(0, len(audio), CLIP_SAMPLES):
        chunk = audio[start:start + CLIP_SAMPLES]
        if len(chunk) < CLIP_SAMPLES:
            # Ignora sobra muito pequena; caso contrario centraliza e completa.
            if len(chunk) < CLIP_SAMPLES // 2:
                continue
            padded = np.zeros(CLIP_SAMPLES, dtype=np.float32)
            pad_start = (CLIP_SAMPLES - len(chunk)) // 2
            padded[pad_start:pad_start + len(chunk)] = chunk
            chunk = padded
        segments.append(chunk.astype(np.float32, copy=False))
    return segments


def process_file(src: Path, dst_dir: Path) -> int:
    audio, sr = sf.read(src, dtype="float32", always_2d=False)
    audio = to_mono(audio)
    audio = resample(audio, sr)
    audio = np.nan_to_num(audio, nan=0.0, posinf=0.0, neginf=0.0)
    audio = np.clip(audio, -1.0, 1.0)

    count = 0
    for idx, segment in enumerate(segment_one_second(audio)):
        dst = dst_dir / f"{src.stem}__seg{idx:03d}.wav"
        sf.write(dst, segment, SAMPLE_RATE, subtype="PCM_16")
        count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DATASET_RAW)
    parser.add_argument("--output", type=Path, default=DATASET_PROCESSED)
    args = parser.parse_args()

    total = 0
    for class_name in CLASSES:
        src_dir = args.input / class_name
        dst_dir = args.output / class_name
        dst_dir.mkdir(parents=True, exist_ok=True)

        files = sorted(src_dir.glob("*.wav"))
        print(f"[{class_name}] {len(files)} arquivos brutos")
        for src in files:
            total += process_file(src, dst_dir)

    print(f"\nPronto: {total} segmentos de 1 s em {args.output}")
    print("Convencao recomendada de nome: speakerID__descricao.wav")


if __name__ == "__main__":
    main()
