from pathlib import Path
from collections import defaultdict
import argparse
import csv
import json
import shutil
import wave

import numpy as np


# ============================================================
# PATHS
# ============================================================

CREMAD_ROOT = (
    Path.home()
    / "Documents"
    / "datasets"
    / "CREMA-D"
)

AUDIO_DIR = CREMAD_ROOT / "AudioWAV"

TABULATED_VOTES_CSV = (
    CREMAD_ROOT
    / "processedResults"
    / "tabulatedVotes.csv"
)

DEST = (
    Path(__file__).parent
    / "dataset"
    / "voicevote_v61"
)

MANIFEST_PATH = (
    Path(__file__).parent
    / "dataset"
    / "voicevote_v61_manifest.csv"
)

REPORT_PATH = (
    Path(__file__).parent
    / "dataset"
    / "voicevote_v61_report.json"
)


# ============================================================
# AUDIO
# ============================================================

SAMPLE_RATE = 16000
WINDOW_SAMPLES = 16000
HOP_SAMPLES = 8000
MIN_LAST_SAMPLES = 12000


# ============================================================
# LABEL QUALITY
#
# Risco: anger + fear + sad
# Não risco: happy + neutral
# Disgust permanece ambíguo e não é forçado para uma classe.
# ============================================================

MIN_RESPONSES = 7
MIN_BINARY_CONFIDENCE = 0.70
MAX_DISGUST_FRACTION = 0.25

RISK_COLUMNS = ("A", "F", "S")
NON_RISK_COLUMNS = ("H", "N")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Prepara o CREMA-D V6.1 usando apenas votos de voz "
            "e remove rótulos binários ambíguos."
        )
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help=(
            "Remove somente dataset/voicevote_v61 e seus relatórios "
            "antes de recriar o dataset."
        ),
    )

    return parser.parse_args()


def prepare_destination(force):
    generated_paths = (
        DEST,
        MANIFEST_PATH,
        REPORT_PATH,
    )

    existing = [path for path in generated_paths if path.exists()]

    if existing and not force:
        names = "\n".join(str(path) for path in existing)
        raise RuntimeError(
            "A saída V6.1 já existe:\n"
            f"{names}\n\n"
            "Use --force somente se quiser recriar essa saída."
        )

    if force:
        if DEST.exists():
            shutil.rmtree(DEST)

        if MANIFEST_PATH.exists():
            MANIFEST_PATH.unlink()

        if REPORT_PATH.exists():
            REPORT_PATH.unlink()

    (DEST / "non_risk").mkdir(parents=True, exist_ok=True)
    (DEST / "risk").mkdir(parents=True, exist_ok=True)


def read_wav(path):
    with wave.open(str(path), "rb") as wav_file:
        channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        sample_rate = wav_file.getframerate()
        frames = wav_file.readframes(wav_file.getnframes())

    if channels != 1:
        raise ValueError(
            f"{path.name}: esperado mono, recebido {channels} canais"
        )

    if sample_width != 2:
        raise ValueError(
            f"{path.name}: esperado PCM16"
        )

    if sample_rate != SAMPLE_RATE:
        raise ValueError(
            f"{path.name}: esperado {SAMPLE_RATE} Hz, "
            f"recebido {sample_rate} Hz"
        )

    return np.frombuffer(frames, dtype=np.int16)


def save_wav(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)

    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(
            samples.astype(np.int16).tobytes()
        )


def normalize_length(samples):
    if len(samples) >= WINDOW_SAMPLES:
        return samples[:WINDOW_SAMPLES]

    output = np.zeros(WINDOW_SAMPLES, dtype=np.int16)
    output[:len(samples)] = samples
    return output


def integer_value(row, column):
    value = row[column].strip()
    return int(float(value))


def float_value(row, column):
    value = row[column].strip()
    return float(value)


def load_voice_labels():
    labels = {}
    counters = defaultdict(int)

    with open(
        TABULATED_VOTES_CSV,
        newline="",
        encoding="utf-8-sig",
    ) as csv_file:
        reader = csv.DictReader(csv_file)

        if not reader.fieldnames:
            raise RuntimeError("tabulatedVotes.csv sem cabeçalho")

        row_id_column = reader.fieldnames[0]

        required_columns = {
            "A",
            "D",
            "F",
            "H",
            "N",
            "S",
            "fileName",
            "numResponses",
            "agreement",
            "emoVote",
        }

        missing = required_columns.difference(reader.fieldnames)

        if missing:
            raise RuntimeError(
                "Colunas ausentes em tabulatedVotes.csv: "
                + ", ".join(sorted(missing))
            )

        for row in reader:
            row_id = row[row_id_column].strip()

            # O primeiro dígito identifica a modalidade:
            # 1 = voz, 2 = rosto, 3 = multimodal.
            if not row_id.startswith("1"):
                counters["ignored_non_voice_rows"] += 1
                continue

            counters["voice_rows"] += 1

            stem = Path(row["fileName"].strip()).stem

            counts = {
                emotion: integer_value(row, emotion)
                for emotion in ("A", "D", "F", "H", "N", "S")
            }

            num_responses = integer_value(row, "numResponses")
            vote_sum = sum(counts.values())

            if num_responses < MIN_RESPONSES:
                counters["skipped_few_responses"] += 1
                continue

            if vote_sum != num_responses:
                counters["skipped_inconsistent_vote_sum"] += 1
                continue

            risk_votes = sum(counts[name] for name in RISK_COLUMNS)
            non_risk_votes = sum(
                counts[name]
                for name in NON_RISK_COLUMNS
            )
            disgust_votes = counts["D"]

            disgust_fraction = disgust_votes / num_responses

            if disgust_fraction > MAX_DISGUST_FRACTION:
                counters["skipped_disgust_ambiguous"] += 1
                continue

            usable_votes = risk_votes + non_risk_votes

            if usable_votes == 0:
                counters["skipped_no_binary_votes"] += 1
                continue

            risk_score = risk_votes / usable_votes
            binary_confidence = max(risk_score, 1.0 - risk_score)

            if binary_confidence < MIN_BINARY_CONFIDENCE:
                counters["skipped_low_binary_confidence"] += 1
                continue

            target_class = (
                "risk"
                if risk_score >= 0.50
                else "non_risk"
            )

            labels[stem] = {
                "target_class": target_class,
                "risk_score": risk_score,
                "binary_confidence": binary_confidence,
                "agreement": float_value(row, "agreement"),
                "emotion_vote": row["emoVote"].strip(),
                "num_responses": num_responses,
                "counts": counts,
                "row_id": row_id,
            }

            counters[f"accepted_{target_class}"] += 1

    return labels, counters


def main():
    args = parse_args()

    if not AUDIO_DIR.exists():
        raise RuntimeError(
            f"AudioWAV não encontrado:\n{AUDIO_DIR}"
        )

    if not TABULATED_VOTES_CSV.exists():
        raise RuntimeError(
            "tabulatedVotes.csv não encontrado:\n"
            f"{TABULATED_VOTES_CSV}"
        )

    prepare_destination(args.force)

    labels, counters = load_voice_labels()

    manifest_rows = []
    original_counts = defaultdict(int)
    segment_counts = defaultdict(int)
    errors = []

    for wav_path in sorted(AUDIO_DIR.glob("*.wav")):
        stem = wav_path.stem

        if stem not in labels:
            counters["audio_not_selected"] += 1
            continue

        metadata = labels[stem]
        target_class = metadata["target_class"]

        try:
            audio = read_wav(wav_path)
        except Exception as error:
            errors.append(f"{wav_path.name}: {error}")
            counters["audio_errors"] += 1
            continue

        original_counts[target_class] += 1
        speaker = stem.split("_")[0]
        segment_index = 0

        for start in range(0, len(audio), HOP_SAMPLES):
            segment = audio[start:start + WINDOW_SAMPLES]

            if len(segment) < MIN_LAST_SAMPLES:
                break

            segment = normalize_length(segment)

            output_name = (
                f"{speaker}__{stem}__seg{segment_index:02d}.wav"
            )

            output_path = DEST / target_class / output_name
            save_wav(output_path, segment)

            counts = metadata["counts"]

            manifest_rows.append(
                {
                    "segment_path": str(output_path.relative_to(DEST.parent)),
                    "source_file": wav_path.name,
                    "speaker": speaker,
                    "original_id": stem,
                    "segment_index": segment_index,
                    "start_sample": start,
                    "target_class": target_class,
                    "risk_score": f"{metadata['risk_score']:.8f}",
                    "binary_confidence": (
                        f"{metadata['binary_confidence']:.8f}"
                    ),
                    "agreement": f"{metadata['agreement']:.8f}",
                    "emotion_vote": metadata["emotion_vote"],
                    "num_responses": metadata["num_responses"],
                    "A": counts["A"],
                    "D": counts["D"],
                    "F": counts["F"],
                    "H": counts["H"],
                    "N": counts["N"],
                    "S": counts["S"],
                }
            )

            segment_counts[target_class] += 1
            segment_index += 1

    fieldnames = [
        "segment_path",
        "source_file",
        "speaker",
        "original_id",
        "segment_index",
        "start_sample",
        "target_class",
        "risk_score",
        "binary_confidence",
        "agreement",
        "emotion_vote",
        "num_responses",
        "A",
        "D",
        "F",
        "H",
        "N",
        "S",
    ]

    with open(MANIFEST_PATH, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(manifest_rows)

    report = {
        "configuration": {
            "sample_rate": SAMPLE_RATE,
            "window_samples": WINDOW_SAMPLES,
            "hop_samples": HOP_SAMPLES,
            "minimum_last_samples": MIN_LAST_SAMPLES,
            "minimum_responses": MIN_RESPONSES,
            "minimum_binary_confidence": MIN_BINARY_CONFIDENCE,
            "maximum_disgust_fraction": MAX_DISGUST_FRACTION,
            "risk_votes": list(RISK_COLUMNS),
            "non_risk_votes": list(NON_RISK_COLUMNS),
        },
        "original_clips": dict(original_counts),
        "segments": dict(segment_counts),
        "counters": dict(counters),
        "errors": errors,
    }

    with open(REPORT_PATH, "w") as json_file:
        json.dump(report, json_file, indent=4)

    print()
    print("================================")
    print("CREMA-D V6.1 - VOICE VOTES")
    print("================================")
    print()
    print("Configuração:")
    print(f"  MIN_RESPONSES          = {MIN_RESPONSES}")
    print(f"  MIN_BINARY_CONFIDENCE  = {MIN_BINARY_CONFIDENCE:.2f}")
    print(f"  MAX_DISGUST_FRACTION   = {MAX_DISGUST_FRACTION:.2f}")
    print()
    print("Clips originais aceitos:")
    print(f"  non_risk: {original_counts['non_risk']}")
    print(f"  risk    : {original_counts['risk']}")
    print()
    print("Segmentos gerados:")
    print(f"  non_risk: {segment_counts['non_risk']}")
    print(f"  risk    : {segment_counts['risk']}")
    print()
    print("Descartes e verificações:")

    for name in sorted(counters):
        print(f"  {name:32s}: {counters[name]}")

    print()
    print(f"Manifesto: {MANIFEST_PATH}")
    print(f"Relatório: {REPORT_PATH}")
    print(f"Destino  : {DEST}")

    if errors:
        print()
        print("AVISO: ocorreram erros de áudio. Veja o relatório JSON.")


if __name__ == "__main__":
    main()
