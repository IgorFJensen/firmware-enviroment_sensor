#!/usr/bin/env python3
"""Treina uma DS-CNN pequena em Log-Mel e exporta TFLite full-int8."""

from __future__ import annotations

import json
import random
from collections import Counter
from pathlib import Path

import numpy as np
import soundfile as sf
import tensorflow as tf
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import GroupShuffleSplit, train_test_split

from config import (
    BUILD_DIR,
    CLASSES,
    CLIP_SAMPLES,
    DATASET_PROCESSED,
    FFT_LENGTH,
    FRAME_LENGTH,
    FRAME_STEP,
    LOWER_HZ,
    N_MELS,
    RANDOM_SEED,
    SAMPLE_RATE,
    UPPER_HZ,
)

np.random.seed(RANDOM_SEED)
random.seed(RANDOM_SEED)
tf.random.set_seed(RANDOM_SEED)


def speaker_id(path: Path) -> str:
    # Nome recomendado: speakerID__descricao__seg000.wav
    return path.stem.split("__", 1)[0]


def scan_dataset() -> tuple[list[Path], np.ndarray, list[str]]:
    paths: list[Path] = []
    labels: list[int] = []
    groups: list[str] = []
    for class_id, class_name in enumerate(CLASSES):
        files = sorted((DATASET_PROCESSED / class_name).glob("*.wav"))
        print(f"{class_name:16s}: {len(files)}")
        for path in files:
            paths.append(path)
            labels.append(class_id)
            groups.append(speaker_id(path))
    if not paths:
        raise RuntimeError("Dataset vazio. Rode prepare_dataset.py primeiro.")
    return paths, np.asarray(labels, dtype=np.int64), groups


def split_indices(labels: np.ndarray, groups: list[str]):
    indices = np.arange(len(labels))
    unique_groups = sorted(set(groups))

    if len(unique_groups) >= 5:
        print(f"Split speaker-disjoint usando {len(unique_groups)} speakers.")
        gss1 = GroupShuffleSplit(n_splits=1, test_size=0.30, random_state=RANDOM_SEED)
        train_idx, temp_idx = next(gss1.split(indices, labels, groups))

        temp_groups = np.asarray(groups, dtype=object)[temp_idx]
        gss2 = GroupShuffleSplit(n_splits=1, test_size=0.50, random_state=RANDOM_SEED + 1)
        val_rel, test_rel = next(gss2.split(temp_idx, labels[temp_idx], temp_groups))
        val_idx = temp_idx[val_rel]
        test_idx = temp_idx[test_rel]
        return train_idx, val_idx, test_idx

    print("AVISO: menos de 5 speakers identificados; usando split aleatorio temporario.")
    train_idx, temp_idx = train_test_split(
        indices, test_size=0.30, random_state=RANDOM_SEED, stratify=labels
    )
    val_idx, test_idx = train_test_split(
        temp_idx,
        test_size=0.50,
        random_state=RANDOM_SEED + 1,
        stratify=labels[temp_idx],
    )
    return train_idx, val_idx, test_idx


def load_audio(path: Path) -> np.ndarray:
    audio, sr = sf.read(path, dtype="float32", always_2d=False)
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    if sr != SAMPLE_RATE:
        raise ValueError(f"{path}: sample rate {sr}, esperado {SAMPLE_RATE}")
    if len(audio) != CLIP_SAMPLES:
        raise ValueError(f"{path}: {len(audio)} amostras, esperado {CLIP_SAMPLES}")
    return np.asarray(audio, dtype=np.float32)


def augment_audio(audio: np.ndarray) -> np.ndarray:
    x = audio.copy()

    # Ganho aleatorio sem destruir a informacao de intensidade.
    gain_db = np.random.uniform(-6.0, 6.0)
    x *= 10.0 ** (gain_db / 20.0)

    # Deslocamento temporal de ate 100 ms.
    shift = np.random.randint(-1600, 1601)
    x = np.roll(x, shift)

    # Ruido branco com SNR aleatorio 10..30 dB.
    rms = float(np.sqrt(np.mean(np.square(x)) + 1e-12))
    snr_db = np.random.uniform(10.0, 30.0)
    noise_rms = rms / (10.0 ** (snr_db / 20.0))
    x += np.random.normal(0.0, noise_rms, size=x.shape).astype(np.float32)

    return np.clip(x, -1.0, 1.0).astype(np.float32)


def log_mel(audio: np.ndarray) -> np.ndarray:
    x = tf.convert_to_tensor(audio, dtype=tf.float32)
    stft = tf.signal.stft(
        x,
        frame_length=FRAME_LENGTH,
        frame_step=FRAME_STEP,
        fft_length=FFT_LENGTH,
        window_fn=tf.signal.hann_window,
        pad_end=False,
    )
    power = tf.square(tf.abs(stft))
    mel_matrix = tf.signal.linear_to_mel_weight_matrix(
        num_mel_bins=N_MELS,
        num_spectrogram_bins=FFT_LENGTH // 2 + 1,
        sample_rate=SAMPLE_RATE,
        lower_edge_hertz=LOWER_HZ,
        upper_edge_hertz=UPPER_HZ,
    )
    mel = tf.matmul(power, mel_matrix)
    features = tf.math.log(mel + 1e-6)
    return features.numpy().astype(np.float32)[..., np.newaxis]


def build_features(paths: list[Path], labels: np.ndarray, augment_copies: int = 0):
    xs: list[np.ndarray] = []
    ys: list[int] = []
    for i, path in enumerate(paths):
        audio = load_audio(path)
        xs.append(log_mel(audio))
        ys.append(int(labels[i]))
        for _ in range(augment_copies):
            xs.append(log_mel(augment_audio(audio)))
            ys.append(int(labels[i]))
        if (i + 1) % 100 == 0:
            print(f"  features: {i + 1}/{len(paths)}")
    return np.stack(xs), np.asarray(ys, dtype=np.int64)


def ds_block(x, filters: int, stride: int = 1):
    x = tf.keras.layers.DepthwiseConv2D(
        3, strides=stride, padding="same", use_bias=False
    )(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU()(x)
    x = tf.keras.layers.Conv2D(filters, 1, padding="same", use_bias=False)(x)
    x = tf.keras.layers.BatchNormalization()(x)
    return tf.keras.layers.ReLU()(x)


def build_model(input_shape: tuple[int, ...]) -> tf.keras.Model:
    inputs = tf.keras.Input(shape=input_shape, name="log_mel")
    x = tf.keras.layers.Conv2D(8, 3, strides=1, padding="same", use_bias=False)(inputs)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU()(x)
    x = ds_block(x, 16, stride=2)
    x = ds_block(x, 24, stride=2)
    x = ds_block(x, 32, stride=1)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    outputs = tf.keras.layers.Dense(len(CLASSES), activation="softmax", name="classes")(x)
    return tf.keras.Model(inputs, outputs, name="acoustic_dscnn")


def representative_dataset(x_train: np.ndarray):
    count = min(250, len(x_train))
    ids = np.linspace(0, len(x_train) - 1, count, dtype=int)
    for idx in ids:
        yield [x_train[idx:idx + 1].astype(np.float32)]


def export_int8(model: tf.keras.Model, x_train: np.ndarray, path: Path) -> bytes:
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = lambda: representative_dataset(x_train)
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    tflite_model = converter.convert()
    path.write_bytes(tflite_model)
    return tflite_model


def eval_tflite(model_content: bytes, x_test: np.ndarray, y_test: np.ndarray):
    interpreter = tf.lite.Interpreter(model_content=model_content)
    interpreter.allocate_tensors()
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()[0]
    in_scale, in_zero = inp["quantization"]

    preds = []
    for x in x_test:
        q = np.round(x / in_scale + in_zero)
        q = np.clip(q, -128, 127).astype(np.int8)[np.newaxis, ...]
        interpreter.set_tensor(inp["index"], q)
        interpreter.invoke()
        yq = interpreter.get_tensor(out["index"])[0]
        preds.append(int(np.argmax(yq)))

    preds = np.asarray(preds)
    acc = float(np.mean(preds == y_test))
    return acc, preds, inp, out


def main() -> None:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    paths, labels, groups = scan_dataset()
    train_idx, val_idx, test_idx = split_indices(labels, groups)

    def subset(idx):
        return [paths[i] for i in idx], labels[idx]

    train_paths, train_y_raw = subset(train_idx)
    val_paths, val_y_raw = subset(val_idx)
    test_paths, test_y_raw = subset(test_idx)

    print("\nGerando Log-Mel...")
    x_train, y_train = build_features(train_paths, train_y_raw, augment_copies=2)
    x_val, y_val = build_features(val_paths, val_y_raw)
    x_test, y_test = build_features(test_paths, test_y_raw)

    print("Shapes:", x_train.shape, x_val.shape, x_test.shape)
    print("Train classes:", Counter(y_train.tolist()))

    model = build_model(tuple(x_train.shape[1:]))
    model.summary()
    model.compile(
        optimizer=tf.keras.optimizers.Adam(1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )

    callbacks = [
        tf.keras.callbacks.EarlyStopping(
            monitor="val_loss", patience=10, restore_best_weights=True
        ),
        tf.keras.callbacks.ReduceLROnPlateau(
            monitor="val_loss", factor=0.5, patience=4, min_lr=1e-5
        ),
        tf.keras.callbacks.ModelCheckpoint(
            BUILD_DIR / "acoustic_best.keras", monitor="val_loss", save_best_only=True
        ),
    ]

    history = model.fit(
        x_train,
        y_train,
        validation_data=(x_val, y_val),
        epochs=80,
        batch_size=32,
        callbacks=callbacks,
        verbose=2,
    )

    keras_loss, keras_acc = model.evaluate(x_test, y_test, verbose=0)
    print(f"\nKeras test accuracy: {keras_acc:.4f}")

    tflite_path = BUILD_DIR / "acoustic_int8.tflite"
    tflite_model = export_int8(model, x_train, tflite_path)
    size_kb = len(tflite_model) / 1024.0
    print(f"TFLite INT8: {size_kb:.2f} KB -> {tflite_path}")
    if len(tflite_model) > 80 * 1024:
        print("AVISO: modelo ultrapassou o teto de 80 KB.")

    int8_acc, preds, inp, out = eval_tflite(tflite_model, x_test, y_test)
    print(f"TFLite INT8 test accuracy: {int8_acc:.4f}")
    print("\nConfusion matrix:")
    print(confusion_matrix(y_test, preds, labels=list(range(len(CLASSES)))))
    print("\nClassification report:")
    print(classification_report(
        y_test, preds, labels=list(range(len(CLASSES))),
        target_names=CLASSES, zero_division=0
    ))

    metadata = {
        "classes": CLASSES,
        "sample_rate": SAMPLE_RATE,
        "clip_samples": CLIP_SAMPLES,
        "frame_length": FRAME_LENGTH,
        "frame_step": FRAME_STEP,
        "fft_length": FFT_LENGTH,
        "n_mels": N_MELS,
        "lower_hz": LOWER_HZ,
        "upper_hz": UPPER_HZ,
        "input_shape": inp["shape"].tolist(),
        "input_quantization": list(inp["quantization"]),
        "output_quantization": list(out["quantization"]),
        "keras_test_accuracy": float(keras_acc),
        "int8_test_accuracy": float(int8_acc),
        "model_size_bytes": len(tflite_model),
    }
    (BUILD_DIR / "model_metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False)
    )

    (BUILD_DIR / "history.json").write_text(
        json.dumps({k: [float(v) for v in vals] for k, vals in history.history.items()}, indent=2)
    )


if __name__ == "__main__":
    main()
