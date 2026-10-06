#!/usr/bin/env python3
"""Coletor simples usando o microfone do PC. Ideal para prototipagem inicial."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import sounddevice as sd
import soundfile as sf

from config import CLASSES, CLIP_SAMPLES, DATASET_RAW, SAMPLE_RATE


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("class_name", choices=CLASSES)
    parser.add_argument("--speaker", required=True, help="ID anonimo, ex.: spk01")
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--output", type=Path, default=DATASET_RAW)
    args = parser.parse_args()

    out_dir = args.output / args.class_name
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Classe: {args.class_name} | speaker: {args.speaker}")
    print("Cada captura tera 1 segundo. Ctrl+C para sair.")

    for idx in range(args.count):
        input(f"[{idx + 1}/{args.count}] ENTER para gravar...")
        print("gravando...")
        audio = sd.rec(CLIP_SAMPLES, samplerate=SAMPLE_RATE, channels=1, dtype="float32")
        sd.wait()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = out_dir / f"{args.speaker}__{args.class_name}__{stamp}__{idx:03d}.wav"
        sf.write(path, audio[:, 0], SAMPLE_RATE, subtype="PCM_16")
        print(f"salvo: {path.name}")


if __name__ == "__main__":
    main()
