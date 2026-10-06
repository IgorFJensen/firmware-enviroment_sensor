from pathlib import Path
from collections import defaultdict
import random
import json

import numpy as np
import tensorflow as tf

from sklearn.metrics import (
    confusion_matrix,
    precision_score,
    recall_score,
    accuracy_score,
    f1_score,
)


# ============================================================
# CONFIGURAÇÃO
# ============================================================

DATASET = Path(__file__).parent / "dataset/processed"

BUILD = Path(__file__).parent / "build"
BUILD.mkdir(exist_ok=True)

MODEL_PATH = BUILD / "cremad_model_v4_binary.keras"
THRESHOLD_PATH = BUILD / "cremad_v4_threshold.json"


# ============================================================
# MAPEAMENTO
#
# 0 = NON_RISK
# 1 = RISK
# ============================================================

SOURCE_MAP = {
    "normal": 0,
    "positive": 0,

    "distress": 1,
    "danger": 1,
}

CLASSES = [
    "non_risk",
    "risk",
]


# ============================================================
# AUDIO
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
# TREINAMENTO
# ============================================================

BATCH_SIZE = 64
EPOCHS = 60

SEED = 42

random.seed(SEED)
np.random.seed(SEED)
tf.random.set_seed(SEED)


# ============================================================
# AUGMENTATION
# ============================================================

MAX_GAIN_DB = 6.0

MAX_SHIFT_SAMPLES = 1600

MIN_SNR_DB = 15.0
MAX_SNR_DB = 30.0

NOISE_PROBABILITY = 0.70


# ============================================================
# LER LISTA DE ARQUIVOS
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

        speaker = path.name.split("__")[0]

        samples.append(
            {
                "path": str(path),
                "label": label,
                "speaker": speaker,
                "source": folder_name,
            }
        )


print()
print("================================")
print("CREMA-D V4 - BINARY RISK")
print("================================")

print()
print(
    "Total de segmentos:",
    len(samples)
)


# ============================================================
# DISTRIBUIÇÃO
# ============================================================

def print_distribution(title, data):

    counts = defaultdict(int)

    for sample in data:
        counts[sample["label"]] += 1

    print()
    print(title)

    print(
        f"non_risk : {counts[0]}"
    )

    print(
        f"risk     : {counts[1]}"
    )


print_distribution(
    "DISTRIBUIÇÃO TOTAL",
    samples
)


# ============================================================
# SPLIT POR SPEAKER
#
# MESMO SEED / MESMA REGRA DAS VERSÕES ANTERIORES
# ============================================================

speakers = sorted(
    set(
        sample["speaker"]
        for sample in samples
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


train_samples = [
    sample
    for sample in samples
    if sample["speaker"] in train_speakers
]


val_samples = [
    sample
    for sample in samples
    if sample["speaker"] in val_speakers
]


test_samples = [
    sample
    for sample in samples
    if sample["speaker"] in test_speakers
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


print_distribution(
    "TRAIN ANTES DO BALANCEAMENTO",
    train_samples
)

print_distribution(
    "VALIDATION",
    val_samples
)

print_distribution(
    "TEST",
    test_samples
)


# ============================================================
# BALANCEAR SOMENTE TRAIN
# ============================================================

by_class = defaultdict(list)


for sample in train_samples:

    by_class[
        sample["label"]
    ].append(
        sample
    )


minimum_samples = min(
    len(by_class[0]),
    len(by_class[1])
)


balanced_train = []


for class_id in [0, 1]:

    class_samples = list(
        by_class[class_id]
    )

    rng.shuffle(
        class_samples
    )

    balanced_train.extend(
        class_samples[
            :minimum_samples
        ]
    )


rng.shuffle(
    balanced_train
)

train_samples = balanced_train


print()
print(
    "Amostras por classe após balanceamento:",
    minimum_samples
)


print_distribution(
    "TRAIN BALANCEADO",
    train_samples
)


# ============================================================
# MEL FILTER BANK
# ============================================================

mel_matrix = tf.signal.linear_to_mel_weight_matrix(
    num_mel_bins=N_MELS,

    num_spectrogram_bins=
        FFT_LENGTH // 2 + 1,

    sample_rate=SAMPLE_RATE,

    lower_edge_hertz=
        LOWER_HZ,

    upper_edge_hertz=
        UPPER_HZ,
)


# ============================================================
# CARREGAR WAV
# ============================================================

def load_audio(path):

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

    return audio


# ============================================================
# AUGMENTATION
# ============================================================

def random_gain(audio):

    gain_db = tf.random.uniform(
        [],

        minval=-MAX_GAIN_DB,

        maxval=MAX_GAIN_DB,
    )

    gain = tf.pow(
        10.0,

        gain_db / 20.0
    )

    return (
        audio * gain
    )


def random_time_shift(audio):

    shift = tf.random.uniform(
        [],

        minval=
            -MAX_SHIFT_SAMPLES,

        maxval=
            MAX_SHIFT_SAMPLES + 1,

        dtype=tf.int32,
    )


    def shift_right():

        result = tf.pad(
            audio,
            [[shift, 0]]
        )

        return result[
            :AUDIO_SAMPLES
        ]


    def shift_left():

        amount = -shift

        result = tf.pad(
            audio[amount:],

            [[0, amount]]
        )

        return result[
            :AUDIO_SAMPLES
        ]


    return tf.cond(
        shift >= 0,

        shift_right,

        shift_left
    )


def add_random_noise(audio):

    signal_rms = tf.sqrt(
        tf.reduce_mean(
            tf.square(audio)
        )
        +
        1e-8
    )


    snr_db = tf.random.uniform(
        [],

        minval=
            MIN_SNR_DB,

        maxval=
            MAX_SNR_DB,
    )


    snr_linear = tf.pow(
        10.0,

        snr_db / 20.0
    )


    noise_rms = (
        signal_rms
        /
        snr_linear
    )


    noise = tf.random.normal(
        tf.shape(audio),

        mean=0.0,

        stddev=noise_rms,
    )


    return (
        audio + noise
    )


def augment_audio(audio):

    audio = random_gain(
        audio
    )


    audio = random_time_shift(
        audio
    )


    add_noise = (
        tf.random.uniform([])
        <
        NOISE_PROBABILITY
    )


    audio = tf.cond(
        add_noise,

        lambda:
            add_random_noise(
                audio
            ),

        lambda:
            audio,
    )


    audio = tf.clip_by_value(
        audio,

        -1.0,

        1.0
    )


    return audio


# ============================================================
# AUDIO -> LOG-MEL
# ============================================================

def audio_to_logmel(audio):

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


    # Não normalizar cada áudio
    # individualmente.
    #
    # Queremos preservar energia,
    # dinâmica e intensidade.


    logmel = tf.expand_dims(
        logmel,

        axis=-1
    )


    return logmel


# ============================================================
# PIPELINE TRAIN
# ============================================================

def process_train(
    path,
    label
):

    audio = load_audio(
        path
    )


    audio = augment_audio(
        audio
    )


    feature = audio_to_logmel(
        audio
    )


    label = tf.cast(
        label,

        tf.float32
    )


    return (
        feature,
        label
    )


# ============================================================
# PIPELINE VALIDATION / TEST
# ============================================================

def process_eval(
    path,
    label
):

    audio = load_audio(
        path
    )


    feature = audio_to_logmel(
        audio
    )


    label = tf.cast(
        label,

        tf.float32
    )


    return (
        feature,
        label
    )


# ============================================================
# CRIAR TF.DATA
# ============================================================

def create_dataset(
    data,
    training=False
):

    paths = [
        sample["path"]
        for sample in data
    ]


    labels = [
        sample["label"]
        for sample in data
    ]


    ds = (
        tf.data.Dataset
        .from_tensor_slices(
            (
                paths,
                labels
            )
        )
    )


    if training:

        ds = ds.shuffle(
            len(data),

            seed=SEED,

            reshuffle_each_iteration=True,
        )


        ds = ds.map(
            process_train,

            num_parallel_calls=
                tf.data.AUTOTUNE
        )

    else:

        ds = ds.map(
            process_eval,

            num_parallel_calls=
                tf.data.AUTOTUNE
        )


    ds = ds.batch(
        BATCH_SIZE
    )


    ds = ds.prefetch(
        tf.data.AUTOTUNE
    )


    return ds


train_ds = create_dataset(
    train_samples,

    training=True
)


val_ds = create_dataset(
    val_samples,

    training=False
)


test_ds = create_dataset(
    test_samples,

    training=False
)


# ============================================================
# CONFIRMAR DIMENSÃO
# ============================================================

for features, labels in train_ds.take(1):

    print()
    print(
        "Feature shape:",
        features.shape
    )

    print(
        "Esperado:"
        " (batch, 49, 32, 1)"
    )


# ============================================================
# MODELO
# ============================================================

inputs = tf.keras.Input(
    shape=(49, 32, 1),

    name="logmel"
)


# ============================================================
# CONV INICIAL
# ============================================================

x = tf.keras.layers.Conv2D(
    16,

    kernel_size=3,

    padding="same",

    use_bias=False,
)(inputs)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ============================================================
# DEPTHWISE BLOCK 1
# ============================================================

x = tf.keras.layers.DepthwiseConv2D(
    kernel_size=3,

    strides=(2, 2),

    padding="same",

    use_bias=False,
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


x = tf.keras.layers.Conv2D(
    24,

    kernel_size=1,

    padding="same",

    use_bias=False,
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ============================================================
# DEPTHWISE BLOCK 2
# ============================================================

x = tf.keras.layers.DepthwiseConv2D(
    kernel_size=3,

    strides=(2, 2),

    padding="same",

    use_bias=False,
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


x = tf.keras.layers.Conv2D(
    32,

    kernel_size=1,

    padding="same",

    use_bias=False,
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ============================================================
# DEPTHWISE BLOCK 3
# ============================================================

x = tf.keras.layers.DepthwiseConv2D(
    kernel_size=3,

    padding="same",

    use_bias=False,
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


x = tf.keras.layers.Conv2D(
    48,

    kernel_size=1,

    padding="same",

    use_bias=False,
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ============================================================
# CLASSIFICADOR BINÁRIO
# ============================================================

x = tf.keras.layers.GlobalAveragePooling2D()(x)


x = tf.keras.layers.Dropout(
    0.20
)(x)


outputs = tf.keras.layers.Dense(
    1,

    activation="sigmoid",

    name="risk_probability"
)(x)


model = tf.keras.Model(
    inputs,

    outputs
)


# ============================================================
# COMPILAR
# ============================================================

model.compile(

    optimizer=
        tf.keras.optimizers.Adam(
            learning_rate=3e-4
        ),

    loss=
        "binary_crossentropy",

    metrics=[
        tf.keras.metrics.BinaryAccuracy(
            name="accuracy"
        ),

        tf.keras.metrics.Precision(
            name="precision"
        ),

        tf.keras.metrics.Recall(
            name="recall"
        ),
    ]
)


model.summary()


# ============================================================
# CALLBACKS
# ============================================================

callbacks = [

    tf.keras.callbacks.EarlyStopping(

        monitor="val_loss",

        patience=10,

        restore_best_weights=True,

        verbose=1,
    ),


    tf.keras.callbacks.ReduceLROnPlateau(

        monitor="val_loss",

        patience=4,

        factor=0.5,

        min_lr=1e-6,

        verbose=1,
    ),


    tf.keras.callbacks.ModelCheckpoint(

        filepath=str(
            BUILD
            /
            "cremad_model_v4_binary_best.keras"
        ),

        monitor="val_loss",

        save_best_only=True,

        verbose=1,
    ),
]


# ============================================================
# TREINAR
# ============================================================

print()
print("================================")
print("TREINANDO V4 BINÁRIA")
print("================================")
print()


history = model.fit(

    train_ds,

    validation_data=
        val_ds,

    epochs=
        EPOCHS,

    callbacks=
        callbacks,
)


# ============================================================
# SALVAR MODELO
# ============================================================

model.save(
    MODEL_PATH
)


print()
print(
    "Modelo salvo em:",
    MODEL_PATH
)


# ============================================================
# FUNÇÃO DE PREVISÃO
# ============================================================

def predict_dataset(ds):

    labels = []
    probabilities = []


    for features, batch_labels in ds:

        probs = model(
            features,

            training=False
        ).numpy()


        probs = probs.reshape(
            -1
        )


        probabilities.extend(
            probs
        )


        labels.extend(
            batch_labels.numpy()
        )


    return (
        np.array(labels)
            .astype(np.int32),

        np.array(
            probabilities
        )
    )


# ============================================================
# VALIDATION
# ============================================================

print()
print(
    "Calculando probabilidades de VALIDATION..."
)


val_labels, val_probs = predict_dataset(
    val_ds
)


# ============================================================
# ESCOLHER THRESHOLD SOMENTE NA VALIDATION
# ============================================================

print()
print(
    "Threshold | Accuracy | Precision | Recall | Specificity | Balanced | F1"
)

print(
    "----------------------------------------------------------------------"
)


best_threshold = None
best_balanced_accuracy = -1.0


for threshold in np.arange(
    0.10,
    0.91,
    0.025
):

    pred = (
        val_probs >= threshold
    ).astype(
        np.int32
    )


    accuracy = accuracy_score(
        val_labels,
        pred
    )


    precision = precision_score(
        val_labels,
        pred,
        zero_division=0
    )


    recall = recall_score(
        val_labels,
        pred,
        zero_division=0
    )


    f1 = f1_score(
        val_labels,
        pred,
        zero_division=0
    )


    cm = confusion_matrix(
        val_labels,
        pred,
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


    balanced_accuracy = (
        recall
        +
        specificity
    ) / 2.0


    print(
        f"{threshold:8.3f} | "
        f"{accuracy * 100:8.2f}% | "
        f"{precision * 100:9.2f}% | "
        f"{recall * 100:6.2f}% | "
        f"{specificity * 100:11.2f}% | "
        f"{balanced_accuracy * 100:8.2f}% | "
        f"{f1 * 100:6.2f}%"
    )


    # Queremos proteger o recall,
    # mas sem aceitar especificidade
    # praticamente nula.
    #
    # Só entram candidatos com:
    #
    # recall >= 85%

    if recall >= 0.85:

        if (
            balanced_accuracy
            >
            best_balanced_accuracy
        ):

            best_balanced_accuracy = (
                balanced_accuracy
            )

            best_threshold = (
                threshold
            )


# ============================================================
# FALLBACK
# ============================================================

if best_threshold is None:

    print()
    print(
        "Nenhum threshold atingiu recall >= 85%."
    )

    print(
        "Escolhendo maior Balanced Accuracy."
    )


    for threshold in np.arange(
        0.10,
        0.91,
        0.025
    ):

        pred = (
            val_probs
            >= threshold
        ).astype(
            np.int32
        )


        cm = confusion_matrix(
            val_labels,
            pred,
            labels=[0, 1]
        )


        tn = cm[0, 0]
        fp = cm[0, 1]

        fn = cm[1, 0]
        tp = cm[1, 1]


        recall = (
            tp /
            (tp + fn + 1e-9)
        )


        specificity = (
            tn /
            (tn + fp + 1e-9)
        )


        balanced_accuracy = (
            recall
            +
            specificity
        ) / 2.0


        if (
            balanced_accuracy
            >
            best_balanced_accuracy
        ):

            best_balanced_accuracy = (
                balanced_accuracy
            )

            best_threshold = (
                threshold
            )


print()
print("============================")
print("THRESHOLD V4")
print("============================")

print(
    f"{best_threshold:.3f}"
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


test_pred = (
    test_probs
    >=
    best_threshold
).astype(
    np.int32
)


# ============================================================
# MÉTRICAS TEST
# ============================================================

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
# RESULTADO FINAL
# ============================================================

print()
print("============================")
print("RESULTADO V4 - TEST")
print("============================")


print(
    f"Threshold          : "
    f"{best_threshold:.3f}"
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


print()
print("Matriz binária:")
print()

print(
    cm
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
# SALVAR THRESHOLD
# ============================================================

threshold_config = {
    "threshold":
        float(best_threshold),

    "sample_rate":
        SAMPLE_RATE,

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
}


with open(
    THRESHOLD_PATH,
    "w"
) as file:

    json.dump(
        threshold_config,

        file,

        indent=4
    )


print()
print(
    "Threshold/config salvo em:"
)

print(
    THRESHOLD_PATH
)

