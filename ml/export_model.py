#!/usr/bin/env python3
"""Incorpora o TFLite V6.1 em components/tinyml_model/model_data.cc."""

from argparse import ArgumentParser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "ml" / "build" / "cremad_model_v61_voicevote_int8.tflite"
DEFAULT_OUTPUT = ROOT / "components" / "tinyml_model" / "model_data.cc"


def format_bytes(data: bytes) -> str:
    rows = []
    for offset in range(0, len(data), 12):
        row = ", ".join(f"0x{value:02x}" for value in data[offset:offset + 12])
        rows.append(f"    {row},")
    return "\n".join(rows)


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    data = args.input.read_bytes()
    if len(data) < 8 or data[4:8] != b"TFL3":
        raise SystemExit(f"Arquivo nao parece ser TFLite valido: {args.input}")

    source = f'''#include "model_data.h"

alignas(16) const unsigned char g_model[] = {{
{format_bytes(data)}
}};

const unsigned int g_model_len = {len(data)}U;
'''

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(source, encoding="utf-8")
    print(f"Modelo incorporado: {args.input}")
    print(f"Destino           : {args.output}")
    print(f"Tamanho           : {len(data)} bytes")


if __name__ == "__main__":
    main()

