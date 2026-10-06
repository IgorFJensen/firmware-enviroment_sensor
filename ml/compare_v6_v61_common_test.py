from pathlib import Path
from collections import defaultdict
import json

import numpy as np
import tensorflow as tf

from sklearn.metrics import (
    accuracy_score,
    confusion_matrix,
    f1_score,
    precision_score,
    recall_score,
    roc_auc_score,
)


# ============================================================
# PATHS
# ============================================================

ROOT = Path(__file__).parent
DATASET = ROOT / "dataset" / "voicevote_v61"
BUILD = ROOT / "build"

SPLIT_PATH = BUILD / "cremad_v61_split.json"
REPORT_PATH = BUILD / "cremad_v6_vs_v61_common_test.json"

MODELS = {
    "V6": {
        "model": BUILD / "cremad_model_v6_voicevote_binary.keras",
        "threshold": BUILD / "cremad_v6_threshold.json",
    },
    "V6.1": {
        "model": BUILD / "cremad_model_v61_voicevote_binary.keras",
        "threshold": BUILD / "cremad_v61_threshold.json",
    },
}


# ============================================================
# DSP — IGUAL AO TREINAMENTO E AO ESP32-C6
# ============================================================

SAMPLE_RATE = 16000
AUDIO_SAMPLES = 16000
FRAME_LENGTH = 480
FRAME_STEP = 320
FFT_LENGTH = 512
N_MELS = 32
LOWER_HZ = 80.0
UPPER_HZ = 7600.0
BATCH_SIZE = 64

SOURCE_MAP = {
    "non_risk": 0,
    "risk": 1,
}


mel_matrix = tf.signal.linear_to_mel_weight_matrix(
    num_mel_bins=N_MELS,
    num_spectrogram_bins=FFT_LENGTH // 2 + 1,
    sample_rate=SAMPLE_RATE,
    lower_edge_hertz=LOWER_HZ,
    upper_edge_hertz=UPPER_HZ,
)


def validate_paths():
    required = [DATASET, SPLIT_PATH]

    for config in MODELS.values():
        required.append(config["model"])
        required.append(config["threshold"])

    missing = [path for path in required if not path.exists()]

    if missing:
        formatted = "\n".join(str(path) for path in missing)
        raise RuntimeError(
            "Arquivos necessários não encontrados:\n"
            f"{formatted}"
        )


def load_samples():
    with open(SPLIT_PATH) as split_file:
        split = json.load(split_file)

    test_speakers = set(split["test_speakers"])
    samples = []

    for folder_name, label in SOURCE_MAP.items():
        folder = DATASET / folder_name

        for path in sorted(folder.glob("*.wav")):
            stem = path.stem
            speaker = stem.split("__")[0]

            if speaker not in test_speakers:
                continue

            original_id = (
                stem.rsplit("__seg", 1)[0]
                if "__seg" in stem
                else stem
            )

            samples.append(
                {
                    "path": str(path),
                    "label": label,
                    "speaker": speaker,
                    "original_id": original_id,
                }
            )

    if not samples:
        raise RuntimeError("Nenhuma amostra de teste encontrada.")

    return samples, sorted(test_speakers)


def process_audio(path, label):
    binary = tf.io.read_file(path)

    audio, _ = tf.audio.decode_wav(
        binary,
        desired_channels=1,
        desired_samples=AUDIO_SAMPLES,
    )

    audio = tf.squeeze(audio, axis=-1)

    stft = tf.signal.stft(
        audio,
        frame_length=FRAME_LENGTH,
        frame_step=FRAME_STEP,
        fft_length=FFT_LENGTH,
        window_fn=tf.signal.hann_window,
        pad_end=False,
    )

    power = tf.square(tf.abs(stft))
    mel = tf.matmul(power, mel_matrix)
    logmel = tf.math.log(mel + 1e-6)
    logmel = tf.expand_dims(logmel, axis=-1)

    return logmel, tf.cast(label, tf.float32)


def create_dataset(samples):
    paths = [sample["path"] for sample in samples]
    labels = [sample["label"] for sample in samples]

    dataset = tf.data.Dataset.from_tensor_slices((paths, labels))
    dataset = dataset.map(
        process_audio,
        num_parallel_calls=tf.data.AUTOTUNE,
    )
    dataset = dataset.batch(BATCH_SIZE)
    dataset = dataset.prefetch(tf.data.AUTOTUNE)
    return dataset


def predict(model, dataset):
    labels = []
    probabilities = []

    for features, batch_labels in dataset:
        batch_probabilities = model(
            features,
            training=False,
        ).numpy().reshape(-1)

        labels.extend(batch_labels.numpy())
        probabilities.extend(batch_probabilities)

    return (
        np.asarray(labels, dtype=np.int32),
        np.asarray(probabilities, dtype=np.float32),
    )


def aggregate_by_clip(samples, labels, probabilities):
    grouped_probabilities = defaultdict(list)
    grouped_labels = {}

    for sample, label, probability in zip(
        samples,
        labels,
        probabilities,
    ):
        original_id = sample["original_id"]
        grouped_probabilities[original_id].append(float(probability))

        previous = grouped_labels.get(original_id)

        if previous is not None and previous != int(label):
            raise RuntimeError(
                f"Rótulos conflitantes para {original_id}."
            )

        grouped_labels[original_id] = int(label)

    original_ids = sorted(grouped_probabilities)

    clip_labels = np.asarray(
        [grouped_labels[item] for item in original_ids],
        dtype=np.int32,
    )

    clip_probabilities = np.asarray(
        [
            np.mean(grouped_probabilities[item])
            for item in original_ids
        ],
        dtype=np.float32,
    )

    return clip_labels, clip_probabilities


def calculate_metrics(labels, probabilities, threshold):
    prediction = (probabilities >= threshold).astype(np.int32)

    matrix = confusion_matrix(
        labels,
        prediction,
        labels=[0, 1],
    )

    tn, fp, fn, tp = matrix.ravel()
    specificity = tn / (tn + fp + 1e-9)
    recall = recall_score(labels, prediction, zero_division=0)

    return {
        "samples": int(len(labels)),
        "threshold": float(threshold),
        "roc_auc": float(roc_auc_score(labels, probabilities)),
        "accuracy": float(accuracy_score(labels, prediction)),
        "precision": float(
            precision_score(labels, prediction, zero_division=0)
        ),
        "recall": float(recall),
        "specificity": float(specificity),
        "false_positive_rate": float(1.0 - specificity),
        "balanced_accuracy": float((recall + specificity) / 2.0),
        "f1": float(f1_score(labels, prediction, zero_division=0)),
        "confusion_matrix": matrix.tolist(),
        "tn": int(tn),
        "fp": int(fp),
        "fn": int(fn),
        "tp": int(tp),
    }


def percent(value):
    return f"{value * 100:6.2f}%"


def print_comparison(results, section):
    print()
    print(section)
    print()
    print(
        "Modelo | Threshold | ROC-AUC | Accuracy | Recall  | "
        "Specificity | Balanced | F1"
    )
    print(
        "-------+-----------+---------+----------+---------+"
        "-------------+----------+---------"
    )

    for model_name, result in results.items():
        metrics = result[section]

        print(
            f"{model_name:6s} | "
            f"{metrics['threshold']:9.3f} | "
            f"{percent(metrics['roc_auc'])} | "
            f"{percent(metrics['accuracy'])} | "
            f"{percent(metrics['recall'])} | "
            f"{percent(metrics['specificity']):11s} | "
            f"{percent(metrics['balanced_accuracy'])} | "
            f"{percent(metrics['f1'])}"
        )


def main():
    validate_paths()
    samples, test_speakers = load_samples()
    dataset = create_dataset(samples)

    results = {}
    reference_labels = None

    print()
    print("========================================")
    print("COMPARAÇÃO V6 x V6.1 — TESTE COMUM")
    print("========================================")
    print()
    print(f"Speakers de teste: {len(test_speakers)}")
    print(f"Segmentos comuns : {len(samples)}")

    for model_name, config in MODELS.items():
        print()
        print(f"Avaliando {model_name}...")

        with open(config["threshold"]) as threshold_file:
            threshold_config = json.load(threshold_file)

        threshold = float(threshold_config["threshold"])
        model = tf.keras.models.load_model(config["model"])
        labels, probabilities = predict(model, dataset)

        if reference_labels is None:
            reference_labels = labels
        elif not np.array_equal(reference_labels, labels):
            raise RuntimeError(
                "A ordem dos rótulos mudou entre as avaliações."
            )

        clip_labels, clip_probabilities = aggregate_by_clip(
            samples,
            labels,
            probabilities,
        )

        results[model_name] = {
            "model_path": str(config["model"]),
            "threshold_path": str(config["threshold"]),
            "segmentos": calculate_metrics(
                labels,
                probabilities,
                threshold,
            ),
            "clips": calculate_metrics(
                clip_labels,
                clip_probabilities,
                threshold,
            ),
        }

        tf.keras.backend.clear_session()

    print_comparison(results, "segmentos")
    print_comparison(results, "clips")

    report = {
        "dataset": str(DATASET),
        "split": str(SPLIT_PATH),
        "test_speakers": test_speakers,
        "results": results,
    }

    with open(REPORT_PATH, "w") as report_file:
        json.dump(report, report_file, indent=4)

    print()
    print(f"Relatório salvo em: {REPORT_PATH}")


if __name__ == "__main__":
    main()
