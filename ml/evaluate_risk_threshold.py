from pathlib import Path
import random

import numpy as np
import tensorflow as tf

from sklearn.metrics import (
    precision_score,
    recall_score,
    accuracy_score,
    f1_score,
    confusion_matrix,
)


# ============================================================
# CONFIGURAÇÃO
# ============================================================

DATASET = Path(__file__).parent / "dataset/processed"

MODEL_PATH = (
    Path(__file__).parent
    / "build"
    / "cremad_model_v3.keras"
)

CLASSES = [
    "non_risk",
    "distress",
    "danger",
]

# Mapeamento das pastas antigas para as classes V3
SOURCE_MAP = {
    "normal": 0,
    "positive": 0,
    "distress": 1,
    "danger": 2,
}

SAMPLE_RATE = 16000
AUDIO_SAMPLES = 16000

FRAME_LENGTH = 480
FRAME_STEP = 320
FFT_LENGTH = 512

N_MELS = 32

LOWER_HZ = 80.0
UPPER_HZ = 7600.0

BATCH_SIZE = 64
SEED = 42


# ============================================================
# CARREGAR LISTA DE ARQUIVOS
# ============================================================

samples = []

for folder_name, label in SOURCE_MAP.items():

    folder = DATASET / folder_name

    if not folder.exists():
        raise RuntimeError(
            f"Pasta não encontrada: {folder}"
        )

    for path in folder.glob("*.wav"):

        # Exemplo:
        #
        # 1001__1001_DFA_ANG_XX__seg00.wav
        #
        # Primeiro campo é o speaker.
        speaker = path.name.split("__")[0]

        samples.append(
            {
                "path": str(path),
                "label": label,
                "speaker": speaker,
            }
        )


print()
print("================================")
print("RISK THRESHOLD EVALUATION")
print("================================")
print()

print(
    "Total de segmentos:",
    len(samples)
)


# ============================================================
# MESMO SPLIT POR SPEAKER USADO NA V3
# ============================================================

speakers = sorted(
    set(
        x["speaker"]
        for x in samples
    )
)

rng = random.Random(SEED)

rng.shuffle(speakers)

n = len(speakers)

n_train = int(
    n * 0.70
)

n_val = int(
    n * 0.15
)


train_speakers = set(
    speakers[:n_train]
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


val_samples = [
    x for x in samples
    if x["speaker"] in val_speakers
]

test_samples = [
    x for x in samples
    if x["speaker"] in test_speakers
]


print()
print("Speakers:")
print(
    "Train:",
    len(train_speakers)
)

print(
    "Validation:",
    len(val_speakers)
)

print(
    "Test:",
    len(test_speakers)
)


print()
print("Segmentos:")

print(
    "Validation:",
    len(val_samples)
)

print(
    "Test:",
    len(test_samples)
)


# ============================================================
# MEL FILTERBANK
# ============================================================

mel_matrix = tf.signal.linear_to_mel_weight_matrix(
    num_mel_bins=N_MELS,
    num_spectrogram_bins=FFT_LENGTH // 2 + 1,
    sample_rate=SAMPLE_RATE,
    lower_edge_hertz=LOWER_HZ,
    upper_edge_hertz=UPPER_HZ,
)


# ============================================================
# WAV -> LOG-MEL
# ============================================================

def process(path, label):

    binary = tf.io.read_file(
        path
    )

    audio, _ = tf.audio.decode_wav(
        binary,
        desired_channels=1,
        desired_samples=AUDIO_SAMPLES,
    )

    audio = tf.squeeze(
        audio,
        axis=-1
    )


    stft = tf.signal.stft(
        audio,
        frame_length=FRAME_LENGTH,
        frame_step=FRAME_STEP,
        fft_length=FFT_LENGTH,
        window_fn=tf.signal.hann_window,
        pad_end=False,
    )


    magnitude = tf.abs(
        stft
    )

    power = tf.square(
        magnitude
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


    return (
        logmel,
        label
    )


# ============================================================
# TF DATASET
# ============================================================

def make_dataset(data):

    paths = [
        x["path"]
        for x in data
    ]

    labels = [
        x["label"]
        for x in data
    ]


    ds = tf.data.Dataset.from_tensor_slices(
        (
            paths,
            labels
        )
    )


    ds = ds.map(
        process,
        num_parallel_calls=tf.data.AUTOTUNE
    )


    ds = ds.batch(
        BATCH_SIZE
    )


    ds = ds.prefetch(
        tf.data.AUTOTUNE
    )


    return ds


val_ds = make_dataset(
    val_samples
)

test_ds = make_dataset(
    test_samples
)


# ============================================================
# CARREGAR MODELO
# ============================================================

print()
print(
    "Carregando modelo:"
)

print(
    MODEL_PATH
)


model = tf.keras.models.load_model(
    MODEL_PATH
)


print(
    "Modelo carregado."
)


# ============================================================
# OBTER PREVISÕES
# ============================================================

def predict_dataset(ds):

    all_labels = []

    all_probs = []


    for features, labels in ds:

        probabilities = model(
            features,
            training=False
        ).numpy()


        all_probs.append(
            probabilities
        )


        all_labels.extend(
            labels.numpy()
        )


    return (
        np.array(
            all_labels
        ),

        np.concatenate(
            all_probs,
            axis=0
        )
    )


# ============================================================
# VALIDATION
# ============================================================

print()
print(
    "Calculando validation..."
)


val_labels, val_probs = predict_dataset(
    val_ds
)


# ============================================================
# CONVERTER 3 CLASSES -> RISCO / NÃO RISCO
# ============================================================

#
# Classe 0:
#
# NON_RISK
#
# Classes 1 e 2:
#
# DISTRESS
# DANGER
#
# são consideradas risco.
#

val_true_risk = (
    val_labels != 0
).astype(
    np.int32
)


# ============================================================
# RISK SCORE
# ============================================================

#
# Saída da rede:
#
# [non_risk, distress, danger]
#
# Então:
#
# risk_score =
# P(distress) + P(danger)
#

val_risk_score = (
    val_probs[:, 1]
    +
    val_probs[:, 2]
)


# ============================================================
# VARREDURA DE THRESHOLD
# ============================================================

print()
print(
    "Threshold | Accuracy | Precision | Recall | Specificity | F1"
)

print(
    "-------------------------------------------------------------"
)


best_threshold = None

best_precision = -1.0


for threshold in np.arange(
    0.20,
    0.81,
    0.05
):

    predicted_risk = (
        val_risk_score
        >=
        threshold
    ).astype(
        np.int32
    )


    accuracy = accuracy_score(
        val_true_risk,
        predicted_risk
    )


    precision = precision_score(
        val_true_risk,
        predicted_risk,
        zero_division=0
    )


    recall = recall_score(
        val_true_risk,
        predicted_risk,
        zero_division=0
    )


    f1 = f1_score(
        val_true_risk,
        predicted_risk,
        zero_division=0
    )


    cm = confusion_matrix(
        val_true_risk,
        predicted_risk,
        labels=[0, 1]
    )


    tn = cm[0, 0]
    fp = cm[0, 1]
    fn = cm[1, 0]
    tp = cm[1, 1]


    specificity = (
        tn /
        (tn + fp + 1e-9)
    )


    print(
        f"{threshold:8.2f} | "
        f"{accuracy * 100:8.2f}% | "
        f"{precision * 100:9.2f}% | "
        f"{recall * 100:6.2f}% | "
        f"{specificity * 100:11.2f}% | "
        f"{f1 * 100:6.2f}%"
    )


    # ========================================================
    # ESCOLHA DO THRESHOLD
    # ========================================================
    #
    # Para nosso sistema:
    #
    # Queremos pelo menos 90% de recall de risco.
    #
    # Entre os thresholds que cumprem isso,
    # escolhemos aquele com maior precision.
    #
    # Isso evita escolher 0.20 apenas porque
    # ele chama praticamente tudo de risco.
    #

    if recall >= 0.90:

        if precision > best_precision:

            best_precision = precision

            best_threshold = threshold


# ============================================================
# FALLBACK
# ============================================================

#
# Caso nenhum threshold consiga recall >= 90%,
# usamos 0.45 como valor inicial.
#

if best_threshold is None:

    print()
    print(
        "Nenhum threshold atingiu recall >= 90%."
    )

    print(
        "Usando threshold fallback = 0.45"
    )

    best_threshold = 0.45


# ============================================================
# THRESHOLD ESCOLHIDO
# ============================================================

print()
print(
    "============================"
)

print(
    "THRESHOLD ESCOLHIDO"
)

print(
    "============================"
)

print(
    f"{best_threshold:.2f}"
)


# ============================================================
# TEST FINAL
# ============================================================

print()
print(
    "Calculando TEST..."
)


test_labels, test_probs = predict_dataset(
    test_ds
)


test_true_risk = (
    test_labels != 0
).astype(
    np.int32
)


test_risk_score = (
    test_probs[:, 1]
    +
    test_probs[:, 2]
)


test_pred_risk = (
    test_risk_score
    >=
    best_threshold
).astype(
    np.int32
)


# ============================================================
# MÉTRICAS
# ============================================================

accuracy = accuracy_score(
    test_true_risk,
    test_pred_risk
)


precision = precision_score(
    test_true_risk,
    test_pred_risk,
    zero_division=0
)


recall = recall_score(
    test_true_risk,
    test_pred_risk,
    zero_division=0
)


f1 = f1_score(
    test_true_risk,
    test_pred_risk,
    zero_division=0
)


cm = confusion_matrix(
    test_true_risk,
    test_pred_risk,
    labels=[0, 1]
)


tn = cm[0, 0]
fp = cm[0, 1]
fn = cm[1, 0]
tp = cm[1, 1]


specificity = (
    tn /
    (tn + fp + 1e-9)
)


false_positive_rate = (
    fp /
    (fp + tn + 1e-9)
)


balanced_accuracy = (
    recall
    +
    specificity
) / 2.0


# ============================================================
# RESULTADO
# ============================================================

print()
print(
    "============================"
)

print(
    "RESULTADO TEST"
)

print(
    "============================"
)


print(
    f"Threshold          : "
    f"{best_threshold:.2f}"
)


print(
    f"Accuracy           : "
    f"{accuracy * 100:.2f}%"
)


print(
    f"Precision          : "
    f"{precision * 100:.2f}%"
)


print(
    f"Recall             : "
    f"{recall * 100:.2f}%"
)


print(
    f"Specificity        : "
    f"{specificity * 100:.2f}%"
)


print(
    f"False Positive Rate: "
    f"{false_positive_rate * 100:.2f}%"
)


print(
    f"Balanced Accuracy  : "
    f"{balanced_accuracy * 100:.2f}%"
)


print(
    f"F1                 : "
    f"{f1 * 100:.2f}%"
)


# ============================================================
# MATRIZ
# ============================================================

print()
print(
    "Matriz binária:"
)

print()

print(
    cm
)


print()
print(
    "Interpretação:"
)

print()

print(
    f"TN - NON_RISK correto : {tn}"
)

print(
    f"FP - falso risco      : {fp}"
)

print(
    f"FN - risco perdido    : {fn}"
)

print(
    f"TP - risco correto    : {tp}"
)


# ============================================================
# EXEMPLOS DE RISK SCORE
# ============================================================

print()
print(
    "============================"
)

print(
    "EXEMPLOS DE RISK SCORE"
)

print(
    "============================"
)


for i in range(
    min(
        20,
        len(test_probs)
    )
):

    non_risk = test_probs[i, 0]

    distress = test_probs[i, 1]

    danger = test_probs[i, 2]

    risk_score = (
        distress
        +
        danger
    )


    true_name = (
        "RISK"
        if test_true_risk[i]
        else
        "NON_RISK"
    )


    predicted_name = (
        "RISK"
        if risk_score >= best_threshold
        else
        "NON_RISK"
    )


    print(
        f"{i:02d} | "
        f"NR={non_risk:.3f} "
        f"DIS={distress:.3f} "
        f"DAN={danger:.3f} "
        f"RISK={risk_score:.3f} | "
        f"TRUE={true_name:8s} | "
        f"PRED={predicted_name}"
    )
