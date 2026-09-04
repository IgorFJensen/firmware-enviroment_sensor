from pathlib import Path
from collections import defaultdict
import json
import random

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

KERAS_MODEL_PATH = (
    BUILD / "cremad_model_v61_voicevote_binary.keras"
)

SPLIT_PATH = BUILD / "cremad_v61_split.json"

TFLITE_PATH = (
    BUILD / "cremad_model_v61_voicevote_int8.tflite"
)

THRESHOLD_PATH = (
    BUILD / "cremad_v61_int8_threshold.json"
)

REPORT_PATH = (
    BUILD / "cremad_v61_int8_report.json"
)


# ============================================================
# AUDIO / DSP — DEVE PERMANECER IGUAL AO FIRMWARE
# ============================================================

SAMPLE_RATE = 16000
AUDIO_SAMPLES = 16000
FRAME_LENGTH = 480
FRAME_STEP = 320
FFT_LENGTH = 512
N_MELS = 32
LOWER_HZ = 80.0
UPPER_HZ = 7600.0

EXPECTED_SHAPE = (49, 32, 1)


# ============================================================
# CONFIGURAÇÃO
# ============================================================

SEED = 42
REPRESENTATIVE_SAMPLES = 600

SOURCE_MAP = {
    "non_risk": 0,
    "risk": 1,
}

random.seed(SEED)
np.random.seed(SEED)
tf.random.set_seed(SEED)


mel_matrix = tf.signal.linear_to_mel_weight_matrix(
    num_mel_bins=N_MELS,
    num_spectrogram_bins=FFT_LENGTH // 2 + 1,
    sample_rate=SAMPLE_RATE,
    lower_edge_hertz=LOWER_HZ,
    upper_edge_hertz=UPPER_HZ,
)


def validate_paths():
    required = (
        DATASET,
        KERAS_MODEL_PATH,
        SPLIT_PATH,
    )

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

    speaker_sets = {
        "train": set(split["train_speakers"]),
        "validation": set(split["validation_speakers"]),
        "test": set(split["test_speakers"]),
    }

    samples_by_split = {
        "train": [],
        "validation": [],
        "test": [],
    }

    for folder_name, label in SOURCE_MAP.items():
        folder = DATASET / folder_name

        if not folder.exists():
            raise RuntimeError(f"Pasta não encontrada: {folder}")

        for path in sorted(folder.glob("*.wav")):
            stem = path.stem
            speaker = stem.split("__")[0]

            original_id = (
                stem.rsplit("__seg", 1)[0]
                if "__seg" in stem
                else stem
            )

            sample = {
                "path": str(path),
                "label": label,
                "speaker": speaker,
                "original_id": original_id,
            }

            found_split = None

            for split_name, speakers in speaker_sets.items():
                if speaker in speakers:
                    found_split = split_name
                    break

            if found_split is None:
                raise RuntimeError(
                    f"Speaker sem split definido: {speaker}"
                )

            samples_by_split[found_split].append(sample)

    for split_name, samples in samples_by_split.items():
        if not samples:
            raise RuntimeError(
                f"Split vazio: {split_name}"
            )

    return samples_by_split


def load_logmel(path):
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

    if tuple(logmel.shape) != EXPECTED_SHAPE:
        raise RuntimeError(
            f"Feature com shape {tuple(logmel.shape)}, "
            f"esperado {EXPECTED_SHAPE}."
        )

    return logmel.numpy().astype(np.float32)


def select_representative_samples(train_samples):
    by_class = defaultdict(list)

    for sample in train_samples:
        by_class[sample["label"]].append(sample)

    rng = random.Random(SEED)

    for samples in by_class.values():
        rng.shuffle(samples)

    per_class = REPRESENTATIVE_SAMPLES // 2

    selected = (
        by_class[0][:per_class]
        + by_class[1][:per_class]
    )

    if len(selected) < REPRESENTATIVE_SAMPLES:
        selected_paths = {sample["path"] for sample in selected}
        remaining = [
            sample
            for sample in train_samples
            if sample["path"] not in selected_paths
        ]
        rng.shuffle(remaining)
        selected.extend(
            remaining[:REPRESENTATIVE_SAMPLES - len(selected)]
        )

    rng.shuffle(selected)
    return selected


def representative_dataset(selected_samples):
    def generator():
        for index, sample in enumerate(selected_samples, start=1):
            if index == 1 or index % 100 == 0:
                print(
                    "Calibração INT8:",
                    f"{index}/{len(selected_samples)}"
                )

            feature = load_logmel(sample["path"])
            yield [np.expand_dims(feature, axis=0)]

    return generator


def quantize_model(model, representative_samples):
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_dataset(
        representative_samples
    )
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8
    ]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8

    return converter.convert()


def quantize_tensor(values, scale, zero_point):
    if scale <= 0.0:
        raise RuntimeError("Escala de quantização de entrada inválida.")

    quantized = np.round(values / scale + zero_point)
    quantized = np.clip(quantized, -128, 127)
    return quantized.astype(np.int8)


def dequantize_tensor(values, scale, zero_point):
    if scale <= 0.0:
        raise RuntimeError("Escala de quantização de saída inválida.")

    return (
        values.astype(np.float32) - zero_point
    ) * scale


def predict_both(model, interpreter, samples, title):
    input_details = interpreter.get_input_details()[0]
    output_details = interpreter.get_output_details()[0]

    input_scale, input_zero_point = input_details["quantization"]
    output_scale, output_zero_point = output_details["quantization"]

    if input_details["dtype"] != np.int8:
        raise RuntimeError(
            f"Entrada TFLite não é INT8: {input_details['dtype']}"
        )

    if output_details["dtype"] != np.int8:
        raise RuntimeError(
            f"Saída TFLite não é INT8: {output_details['dtype']}"
        )

    labels = []
    keras_probabilities = []
    int8_probabilities = []

    print()
    print(f"Avaliando {title}: {len(samples)} segmentos")

    for index, sample in enumerate(samples, start=1):
        if index == 1 or index % 500 == 0:
            print(f"  {index}/{len(samples)}")

        feature = load_logmel(sample["path"])
        batched_feature = np.expand_dims(feature, axis=0)

        keras_probability = model(
            batched_feature,
            training=False,
        ).numpy().reshape(-1)[0]

        quantized_input = quantize_tensor(
            batched_feature,
            input_scale,
            input_zero_point,
        )

        interpreter.set_tensor(
            input_details["index"],
            quantized_input,
        )
        interpreter.invoke()

        quantized_output = interpreter.get_tensor(
            output_details["index"]
        )

        int8_probability = dequantize_tensor(
            quantized_output,
            output_scale,
            output_zero_point,
        ).reshape(-1)[0]

        labels.append(sample["label"])
        keras_probabilities.append(float(keras_probability))
        int8_probabilities.append(float(int8_probability))

    return (
        np.asarray(labels, dtype=np.int32),
        np.asarray(keras_probabilities, dtype=np.float32),
        np.asarray(int8_probabilities, dtype=np.float32),
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
    matrix = confusion_matrix(labels, prediction, labels=[0, 1])
    tn, fp, fn, tp = matrix.ravel()

    recall = recall_score(labels, prediction, zero_division=0)
    specificity = tn / (tn + fp + 1e-9)

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


def select_threshold(labels, probabilities):
    best_threshold = None
    best_balanced = -1.0

    for threshold in np.arange(0.05, 0.951, 0.005):
        prediction = (probabilities >= threshold).astype(np.int32)
        matrix = confusion_matrix(labels, prediction, labels=[0, 1])
        tn, fp, fn, tp = matrix.ravel()

        recall = tp / (tp + fn + 1e-9)
        specificity = tn / (tn + fp + 1e-9)
        balanced = (recall + specificity) / 2.0

        if balanced > best_balanced:
            best_balanced = balanced
            best_threshold = float(threshold)

    return best_threshold, best_balanced


def quantization_error(reference, quantized):
    absolute_error = np.abs(reference - quantized)

    return {
        "mean_absolute_error": float(np.mean(absolute_error)),
        "maximum_absolute_error": float(np.max(absolute_error)),
        "correlation": float(np.corrcoef(reference, quantized)[0, 1]),
    }


def print_metrics(title, metrics):
    print()
    print(title)
    print(f"  Threshold          : {metrics['threshold']:.3f}")
    print(f"  ROC-AUC            : {metrics['roc_auc'] * 100:.2f}%")
    print(f"  Accuracy           : {metrics['accuracy'] * 100:.2f}%")
    print(f"  Precision          : {metrics['precision'] * 100:.2f}%")
    print(f"  Recall             : {metrics['recall'] * 100:.2f}%")
    print(f"  Specificity        : {metrics['specificity'] * 100:.2f}%")
    print(f"  False Positive Rate: {metrics['false_positive_rate'] * 100:.2f}%")
    print(f"  Balanced Accuracy  : {metrics['balanced_accuracy'] * 100:.2f}%")
    print(f"  F1                 : {metrics['f1'] * 100:.2f}%")
    print(f"  Matriz             : {metrics['confusion_matrix']}")


def main():
    validate_paths()
    samples_by_split = load_samples()

    representative_samples = select_representative_samples(
        samples_by_split["train"]
    )

    print()
    print("================================")
    print("QUANTIZAÇÃO V6.1 — INT8 COMPLETO")
    print("================================")
    print()
    print(f"Modelo Keras : {KERAS_MODEL_PATH}")
    print(f"Calibração   : {len(representative_samples)} segmentos de treino")

    model = tf.keras.models.load_model(KERAS_MODEL_PATH)

    if tuple(model.input_shape[1:]) != EXPECTED_SHAPE:
        raise RuntimeError(
            f"Modelo com entrada {model.input_shape}, "
            f"esperado (None, {EXPECTED_SHAPE})."
        )

    tflite_bytes = quantize_model(model, representative_samples)
    TFLITE_PATH.write_bytes(tflite_bytes)

    print()
    print(f"TFLite salvo : {TFLITE_PATH}")
    print(f"Tamanho      : {len(tflite_bytes)} bytes")

    interpreter = tf.lite.Interpreter(model_content=tflite_bytes)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()[0]
    output_details = interpreter.get_output_details()[0]

    print()
    print("Entrada INT8:")
    print(f"  shape      : {input_details['shape'].tolist()}")
    print(f"  dtype      : {input_details['dtype']}")
    print(f"  quantização: {input_details['quantization']}")
    print("Saída INT8:")
    print(f"  shape      : {output_details['shape'].tolist()}")
    print(f"  dtype      : {output_details['dtype']}")
    print(f"  quantização: {output_details['quantization']}")

    validation = predict_both(
        model,
        interpreter,
        samples_by_split["validation"],
        "VALIDATION",
    )

    val_labels, val_keras_probs, val_int8_probs = validation

    int8_threshold, val_int8_balanced = select_threshold(
        val_labels,
        val_int8_probs,
    )

    print()
    print("Threshold INT8 selecionado na validação:")
    print(f"  threshold: {int8_threshold:.3f}")
    print(f"  balanced : {val_int8_balanced * 100:.2f}%")

    test = predict_both(
        model,
        interpreter,
        samples_by_split["test"],
        "TEST",
    )

    test_labels, test_keras_probs, test_int8_probs = test

    with open(BUILD / "cremad_v61_threshold.json") as threshold_file:
        keras_threshold = float(json.load(threshold_file)["threshold"])

    keras_segment_metrics = calculate_metrics(
        test_labels,
        test_keras_probs,
        keras_threshold,
    )

    int8_segment_metrics = calculate_metrics(
        test_labels,
        test_int8_probs,
        int8_threshold,
    )

    keras_clip_labels, keras_clip_probs = aggregate_by_clip(
        samples_by_split["test"],
        test_labels,
        test_keras_probs,
    )

    int8_clip_labels, int8_clip_probs = aggregate_by_clip(
        samples_by_split["test"],
        test_labels,
        test_int8_probs,
    )

    if not np.array_equal(keras_clip_labels, int8_clip_labels):
        raise RuntimeError("Rótulos por clip incompatíveis.")

    keras_clip_metrics = calculate_metrics(
        keras_clip_labels,
        keras_clip_probs,
        keras_threshold,
    )

    int8_clip_metrics = calculate_metrics(
        int8_clip_labels,
        int8_clip_probs,
        int8_threshold,
    )

    print_metrics("KERAS — TEST SEGMENTOS", keras_segment_metrics)
    print_metrics("INT8 — TEST SEGMENTOS", int8_segment_metrics)
    print_metrics("KERAS — TEST POR CLIP", keras_clip_metrics)
    print_metrics("INT8 — TEST POR CLIP", int8_clip_metrics)

    validation_error = quantization_error(
        val_keras_probs,
        val_int8_probs,
    )

    test_error = quantization_error(
        test_keras_probs,
        test_int8_probs,
    )

    print()
    print("ERRO DE QUANTIZAÇÃO")
    print(
        "  Validation MAE / máximo / correlação: "
        f"{validation_error['mean_absolute_error']:.6f} / "
        f"{validation_error['maximum_absolute_error']:.6f} / "
        f"{validation_error['correlation']:.6f}"
    )
    print(
        "  Test MAE / máximo / correlação      : "
        f"{test_error['mean_absolute_error']:.6f} / "
        f"{test_error['maximum_absolute_error']:.6f} / "
        f"{test_error['correlation']:.6f}"
    )

    input_scale, input_zero_point = input_details["quantization"]
    output_scale, output_zero_point = output_details["quantization"]

    threshold_config = {
        "version": "6.1-int8",
        "threshold": float(int8_threshold),
        "threshold_selection": "maximum_validation_balanced_accuracy",
        "sample_rate": SAMPLE_RATE,
        "audio_samples": AUDIO_SAMPLES,
        "frame_length": FRAME_LENGTH,
        "frame_step": FRAME_STEP,
        "fft_length": FFT_LENGTH,
        "mel_bins": N_MELS,
        "lower_hz": LOWER_HZ,
        "upper_hz": UPPER_HZ,
        "input_scale": float(input_scale),
        "input_zero_point": int(input_zero_point),
        "output_scale": float(output_scale),
        "output_zero_point": int(output_zero_point),
    }

    report = {
        "tflite_path": str(TFLITE_PATH),
        "tflite_size_bytes": len(tflite_bytes),
        "representative_samples": len(representative_samples),
        "threshold": threshold_config,
        "validation_quantization_error": validation_error,
        "test_quantization_error": test_error,
        "keras_test_segments": keras_segment_metrics,
        "int8_test_segments": int8_segment_metrics,
        "keras_test_clips": keras_clip_metrics,
        "int8_test_clips": int8_clip_metrics,
    }

    with open(THRESHOLD_PATH, "w") as threshold_file:
        json.dump(threshold_config, threshold_file, indent=4)

    with open(REPORT_PATH, "w") as report_file:
        json.dump(report, report_file, indent=4)

    print()
    print(f"Threshold salvo: {THRESHOLD_PATH}")
    print(f"Relatório salvo : {REPORT_PATH}")


if __name__ == "__main__":
    main()
