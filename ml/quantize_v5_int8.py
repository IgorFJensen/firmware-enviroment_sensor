from pathlib import Path
import random
import json

import numpy as np
import tensorflow as tf

from sklearn.metrics import (
    confusion_matrix,
    accuracy_score,
    precision_score,
    recall_score,
    f1_score,
    roc_auc_score,
)


# ============================================================
# PATHS
# ============================================================

ROOT = Path(__file__).parent

DATASET = (
    ROOT
    / "dataset"
    / "voicevote"
)

MODEL_PATH = (
    ROOT
    / "build"
    / "cremad_model_v5_voicevote_binary.keras"
)

BUILD = (
    ROOT
    / "build"
)

BUILD.mkdir(
    exist_ok=True
)


TFLITE_PATH = (
    BUILD
    / "cremad_model_v5_int8.tflite"
)


MODEL_CC_PATH = (
    ROOT.parent
    / "components"
    / "tinyml_model"
    / "model_data.cc"
)


MODEL_H_PATH = (
    ROOT.parent
    / "components"
    / "tinyml_model"
    / "model_data.h"
)


CONFIG_PATH = (
    BUILD
    / "cremad_v5_int8_config.json"
)


# ============================================================
# CLASSES
# ============================================================

SOURCE_MAP = {

    "non_risk": 0,

    "distress": 1,

    "danger": 1,
}


# ============================================================
# DSP
# ============================================================

SAMPLE_RATE = 16000

AUDIO_SAMPLES = 16000

FRAME_LENGTH = 480

FRAME_STEP = 320

FFT_LENGTH = 512

N_MELS = 32

LOWER_HZ = 80.0

UPPER_HZ = 7600.0


# ============================================================
# SPLIT
# ============================================================

SEED = 42

random.seed(SEED)

np.random.seed(SEED)

tf.random.set_seed(SEED)


# ============================================================
# LISTA DE ARQUIVOS
# ============================================================

samples = []


for folder_name, label in SOURCE_MAP.items():

    folder = (
        DATASET
        / folder_name
    )

    for path in folder.glob("*.wav"):

        speaker = (
            path.name
            .split("__")[0]
        )

        samples.append(
            {
                "path": str(path),
                "label": label,
                "speaker": speaker,
            }
        )


print()
print(
    "Segmentos encontrados:",
    len(samples)
)


# ============================================================
# MESMO SPLIT DA V5
# ============================================================

speakers = sorted(
    set(
        x["speaker"]
        for x in samples
    )
)


rng = random.Random(
    SEED
)

rng.shuffle(
    speakers
)


n = len(speakers)

n_train = int(
    n * 0.70
)

n_val = int(
    n * 0.15
)


train_speakers = set(
    speakers[
        :n_train
    ]
)


val_speakers = set(
    speakers[
        n_train:
        n_train + n_val
    ]
)


test_speakers = set(
    speakers[
        n_train + n_val:
    ]
)


train_samples = [
    x
    for x in samples
    if x["speaker"]
    in train_speakers
]


val_samples = [
    x
    for x in samples
    if x["speaker"]
    in val_speakers
]


test_samples = [
    x
    for x in samples
    if x["speaker"]
    in test_speakers
]


print(
    "Train:",
    len(train_samples)
)

print(
    "Validation:",
    len(val_samples)
)

print(
    "Test:",
    len(test_samples)
)


# ============================================================
# MEL MATRIX
# ============================================================

mel_matrix = (
    tf.signal
    .linear_to_mel_weight_matrix(

        num_mel_bins=N_MELS,

        num_spectrogram_bins=
            FFT_LENGTH // 2 + 1,

        sample_rate=
            SAMPLE_RATE,

        lower_edge_hertz=
            LOWER_HZ,

        upper_edge_hertz=
            UPPER_HZ,
    )
)


# ============================================================
# WAV -> LOG-MEL
# ============================================================

def wav_to_logmel(
    path
):

    binary = tf.io.read_file(
        path
    )


    audio, _ = tf.audio.decode_wav(

        binary,

        desired_channels=1,

        desired_samples=
            AUDIO_SAMPLES,
    )


    audio = tf.squeeze(
        audio,
        axis=-1
    )


    stft = tf.signal.stft(

        audio,

        frame_length=
            FRAME_LENGTH,

        frame_step=
            FRAME_STEP,

        fft_length=
            FFT_LENGTH,

        window_fn=
            tf.signal.hann_window,

        pad_end=False,
    )


    power = tf.square(
        tf.abs(
            stft
        )
    )


    mel = tf.matmul(
        power,
        mel_matrix
    )


    logmel = tf.math.log(
        mel + 1e-6
    )


    logmel = tf.expand_dims(
        logmel,
        axis=-1
    )


    return logmel


# ============================================================
# CARREGAR KERAS
# ============================================================

print()
print(
    "Carregando:"
)

print(
    MODEL_PATH
)


model = (
    tf.keras.models
    .load_model(
        MODEL_PATH
    )
)


print(
    "Modelo carregado."
)


# ============================================================
# REPRESENTATIVE DATASET
# ============================================================

#
# Vamos usar somente TRAIN.
#
# Isso é importante:
#
# validation/test não entram
# na calibração INT8.
#

train_non_risk = [
    x
    for x in train_samples
    if x["label"] == 0
]


train_risk = [
    x
    for x in train_samples
    if x["label"] == 1
]


rng.shuffle(
    train_non_risk
)

rng.shuffle(
    train_risk
)


# 250 + 250 = 500 exemplos
representative_samples = (

    train_non_risk[:250]
    +
    train_risk[:250]
)


rng.shuffle(
    representative_samples
)


def representative_dataset():

    for i, sample in enumerate(
        representative_samples
    ):

        feature = wav_to_logmel(
            sample["path"]
        )


        feature = tf.cast(
            feature,
            tf.float32
        )


        feature = tf.expand_dims(
            feature,
            axis=0
        )


        yield [
            feature
        ]


# ============================================================
# CONVERTER FULL INT8
# ============================================================

print()
print(
    "Convertendo para FULL INT8..."
)


converter = (
    tf.lite.TFLiteConverter
    .from_keras_model(
        model
    )
)


converter.optimizations = [
    tf.lite.Optimize.DEFAULT
]


converter.representative_dataset = (
    representative_dataset
)


converter.target_spec.supported_ops = [

    tf.lite.OpsSet
    .TFLITE_BUILTINS_INT8
]


converter.inference_input_type = (
    tf.int8
)


converter.inference_output_type = (
    tf.int8
)


tflite_model = (
    converter.convert()
)


# ============================================================
# SALVAR
# ============================================================

with open(
    TFLITE_PATH,
    "wb"
) as f:

    f.write(
        tflite_model
    )


size_bytes = len(
    tflite_model
)


print()
print(
    "================================"
)

print(
    "MODELO INT8 GERADO"
)

print(
    "================================"
)

print(
    TFLITE_PATH
)

print(
    f"Tamanho: "
    f"{size_bytes} bytes"
)

print(
    f"Tamanho: "
    f"{size_bytes / 1024:.2f} KB"
)


# ============================================================
# TFLITE INTERPRETER
# ============================================================

interpreter = (
    tf.lite.Interpreter(
        model_path=str(
            TFLITE_PATH
        )
    )
)


interpreter.allocate_tensors()


input_details = (
    interpreter
    .get_input_details()[0]
)


output_details = (
    interpreter
    .get_output_details()[0]
)


print()
print(
    "INPUT:"
)

print(
    input_details
)


print()
print(
    "OUTPUT:"
)

print(
    output_details
)


input_scale = (
    input_details[
        "quantization"
    ][0]
)


input_zero_point = (
    input_details[
        "quantization"
    ][1]
)


output_scale = (
    output_details[
        "quantization"
    ][0]
)


output_zero_point = (
    output_details[
        "quantization"
    ][1]
)


print()
print(
    "Quantização:"
)

print(
    "Input scale:",
    input_scale
)

print(
    "Input zero point:",
    input_zero_point
)

print(
    "Output scale:",
    output_scale
)

print(
    "Output zero point:",
    output_zero_point
)


# ============================================================
# INFERÊNCIA INT8
# ============================================================

def infer_int8(
    path
):

    feature = wav_to_logmel(
        path
    ).numpy()


    # ========================================================
    # FLOAT -> INT8
    # ========================================================

    quantized = np.round(

        feature
        /
        input_scale

        +
        input_zero_point

    )


    quantized = np.clip(

        quantized,

        -128,

        127
    )


    quantized = quantized.astype(
        np.int8
    )


    quantized = np.expand_dims(
        quantized,
        axis=0
    )


    interpreter.set_tensor(
        input_details["index"],
        quantized
    )


    interpreter.invoke()


    output_int8 = (
        interpreter.get_tensor(
            output_details["index"]
        )
    )


    output_value = (
        float(
            output_int8.reshape(-1)[0]
        )
        -
        output_zero_point
    ) * output_scale


    return float(
        output_value
    )


# ============================================================
# PREDICT DATASET
# ============================================================

def predict_samples(
    sample_list
):

    labels = []

    probabilities = []


    total = len(
        sample_list
    )


    for index, sample in enumerate(
        sample_list
    ):

        probability = infer_int8(
            sample["path"]
        )


        probabilities.append(
            probability
        )


        labels.append(
            sample["label"]
        )


        if (
            index % 500 == 0
        ):

            print(
                f"{index}/{total}"
            )


    return (

        np.array(
            labels,
            dtype=np.int32
        ),

        np.array(
            probabilities,
            dtype=np.float32
        )
    )


# ============================================================
# VALIDATION INT8
# ============================================================

print()
print(
    "================================"
)

print(
    "AVALIANDO INT8 - VALIDATION"
)

print(
    "================================"
)


val_labels, val_probs = (
    predict_samples(
        val_samples
    )
)


val_auc = roc_auc_score(
    val_labels,
    val_probs
)


print()
print(
    f"INT8 Validation ROC-AUC: "
    f"{val_auc * 100:.2f}%"
)


# ============================================================
# ESCOLHER THRESHOLD NO INT8
# ============================================================

best_threshold = None

best_balanced = -1.0


print()
print(
    "Threshold | Recall | Specificity | Balanced"
)

print(
    "--------------------------------------------"
)


for threshold in np.arange(
    0.10,
    0.91,
    0.025
):

    prediction = (
        val_probs
        >= threshold
    ).astype(
        np.int32
    )


    cm = confusion_matrix(

        val_labels,

        prediction,

        labels=[0, 1]
    )


    tn = cm[0, 0]

    fp = cm[0, 1]

    fn = cm[1, 0]

    tp = cm[1, 1]


    recall = (

        tp
        /
        (tp + fn + 1e-9)
    )


    specificity = (

        tn
        /
        (tn + fp + 1e-9)
    )


    balanced = (

        recall
        +
        specificity

    ) / 2.0


    print(

        f"{threshold:8.3f} | "

        f"{recall * 100:6.2f}% | "

        f"{specificity * 100:11.2f}% | "

        f"{balanced * 100:8.2f}%"
    )


    # mesma regra da V5:
    # recall mínimo 80%

    if recall >= 0.80:

        if balanced > best_balanced:

            best_balanced = (
                balanced
            )

            best_threshold = (
                threshold
            )


if best_threshold is None:

    best_threshold = 0.35


print()
print(
    "Threshold INT8 escolhido:"
)

print(
    f"{best_threshold:.3f}"
)


# ============================================================
# TEST INT8
# ============================================================

print()
print(
    "================================"
)

print(
    "AVALIANDO INT8 - TEST"
)

print(
    "================================"
)


test_labels, test_probs = (
    predict_samples(
        test_samples
    )
)


test_pred = (
    test_probs
    >= best_threshold
).astype(
    np.int32
)


test_auc = roc_auc_score(
    test_labels,
    test_probs
)


accuracy = accuracy_score(
    test_labels,
    test_pred
)


precision = precision_score(
    test_labels,
    test_pred,
    zero_division=0
)


recall = recall_score(
    test_labels,
    test_pred,
    zero_division=0
)


f1 = f1_score(
    test_labels,
    test_pred,
    zero_division=0
)


cm = confusion_matrix(

    test_labels,

    test_pred,

    labels=[0, 1]
)


tn = cm[0, 0]

fp = cm[0, 1]

fn = cm[1, 0]

tp = cm[1, 1]


specificity = (

    tn
    /
    (tn + fp + 1e-9)
)


balanced = (

    recall
    +
    specificity

) / 2.0


print()
print(
    "================================"
)

print(
    "RESULTADO INT8"
)

print(
    "================================"
)


print(
    f"Threshold         : "
    f"{best_threshold:.3f}"
)

print(
    f"ROC-AUC           : "
    f"{test_auc * 100:.2f}%"
)

print(
    f"Accuracy          : "
    f"{accuracy * 100:.2f}%"
)

print(
    f"Precision         : "
    f"{precision * 100:.2f}%"
)

print(
    f"Recall            : "
    f"{recall * 100:.2f}%"
)

print(
    f"Specificity       : "
    f"{specificity * 100:.2f}%"
)

print(
    f"Balanced Accuracy : "
    f"{balanced * 100:.2f}%"
)

print(
    f"F1                : "
    f"{f1 * 100:.2f}%"
)

print()
print(
    "Matriz:"
)

print(
    cm
)


# ============================================================
# GERAR model_data.cc
# ============================================================

MODEL_CC_PATH.parent.mkdir(
    parents=True,
    exist_ok=True
)


with open(
    MODEL_CC_PATH,
    "w"
) as f:

    f.write(
        '#include "model_data.h"\n\n'
    )

    f.write(
        "alignas(16) "
        "const unsigned char "
        "g_model[] = {\n"
    )


    for i, byte in enumerate(
        tflite_model
    ):

        if i % 12 == 0:

            f.write(
                "    "
            )


        f.write(
            f"0x{byte:02x}"
        )


        if (
            i
            !=
            len(tflite_model) - 1
        ):

            f.write(
                ", "
            )


        if i % 12 == 11:

            f.write(
                "\n"
            )


    f.write(
        "\n};\n\n"
    )


    f.write(

        "const unsigned int "
        "g_model_len = "
        "sizeof(g_model);\n"
    )


# ============================================================
# GERAR HEADER
# ============================================================

with open(
    MODEL_H_PATH,
    "w"
) as f:

    f.write(
        "#pragma once\n\n"
    )

    f.write(
        "extern const "
        "unsigned char "
        "g_model[];\n"
    )

    f.write(
        "extern const "
        "unsigned int "
        "g_model_len;\n"
    )


# ============================================================
# SALVAR CONFIG
# ============================================================

config = {

    "threshold":
        float(
            best_threshold
        ),

    "input_scale":
        float(
            input_scale
        ),

    "input_zero_point":
        int(
            input_zero_point
        ),

    "output_scale":
        float(
            output_scale
        ),

    "output_zero_point":
        int(
            output_zero_point
        ),

    "sample_rate":
        SAMPLE_RATE,

    "audio_samples":
        AUDIO_SAMPLES,

    "frames":
        49,

    "frame_length":
        FRAME_LENGTH,

    "frame_step":
        FRAME_STEP,

    "fft_length":
        FFT_LENGTH,

    "mel_bins":
        N_MELS,

    "lower_hz":
        LOWER_HZ,

    "upper_hz":
        UPPER_HZ,

    "model_bytes":
        size_bytes,
}


with open(
    CONFIG_PATH,
    "w"
) as f:

    json.dump(
        config,
        f,
        indent=4
    )


print()
print(
    "================================"
)

print(
    "EXPORT CONCLUÍDO"
)

print(
    "================================"
)


print(
    "TFLite:"
)

print(
    TFLITE_PATH
)


print()
print(
    "C++:"
)

print(
    MODEL_CC_PATH
)


print()
print(
    "Header:"
)

print(
    MODEL_H_PATH
)


print()
print(
    "Config:"
)

print(
    CONFIG_PATH
)
