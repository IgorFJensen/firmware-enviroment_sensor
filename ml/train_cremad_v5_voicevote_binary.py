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
    roc_auc_score,
)


# ============================================================
# CONFIGURAÇÃO
# ============================================================

DATASET = (
    Path(__file__).parent
    / "dataset"
    / "voicevote"
)

BUILD = (
    Path(__file__).parent
    / "build"
)

BUILD.mkdir(
    exist_ok=True
)


MODEL_PATH = (
    BUILD
    / "cremad_model_v5_voicevote_binary.keras"
)

THRESHOLD_PATH = (
    BUILD
    / "cremad_v5_threshold.json"
)


# ============================================================
# CLASSES
#
# 0 = NON_RISK
# 1 = RISK
# ============================================================

SOURCE_MAP = {

    "non_risk": 0,

    "distress": 1,
    "danger": 1,
}


CLASSES = [
    "non_risk",
    "risk",
]


# ============================================================
# AUDIO / DSP
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
# TRAIN CONFIG
# ============================================================

BATCH_SIZE = 64

EPOCHS = 60

SEED = 42


random.seed(SEED)

np.random.seed(SEED)

tf.random.set_seed(SEED)


# ============================================================
# AUGMENTATION
#
# V5 um pouco mais leve que V4.
# ============================================================

MAX_GAIN_DB = 4.0

MAX_SHIFT_SAMPLES = 1280
# 80 ms

MIN_SNR_DB = 20.0

MAX_SNR_DB = 35.0

NOISE_PROBABILITY = 0.40


# ============================================================
# CARREGAR SEGMENTOS
# ============================================================

samples = []


for folder_name, label in SOURCE_MAP.items():

    folder = (
        DATASET
        / folder_name
    )

    if not folder.exists():

        raise RuntimeError(
            f"Pasta não encontrada: {folder}"
        )


    for path in folder.glob("*.wav"):

        stem = path.stem


        # Ex:
        #
        # 1001__1001_DFA_ANG_XX__seg00
        #
        # speaker:
        #
        # 1001

        speaker = (
            stem.split("__")[0]
        )


        # ====================================================
        # IDENTIFICADOR DO CLIP ORIGINAL
        #
        # Remove:
        #
        # __seg00
        # __seg01
        #
        # Isso permite balancear por gravação original,
        # e não simplesmente pelo número de janelas.
        # ====================================================

        if "__seg" in stem:

            original_id = (
                stem.rsplit(
                    "__seg",
                    1
                )[0]
            )

        else:

            original_id = stem


        samples.append(
            {
                "path": str(path),

                "label": label,

                "speaker": speaker,

                "original_id": original_id,

                "source": folder_name,
            }
        )


print()

print(
    "================================"
)

print(
    "CREMA-D V5 VOICEVOTE BINARY"
)

print(
    "================================"
)

print()

print(
    "Segmentos encontrados:",
    len(samples)
)


# ============================================================
# ESTATÍSTICAS
# ============================================================

def print_distribution(
    title,
    data
):

    segment_counts = defaultdict(int)

    originals = defaultdict(set)


    for sample in data:

        label = sample["label"]

        segment_counts[
            label
        ] += 1


        originals[
            label
        ].add(
            sample["original_id"]
        )


    print()

    print(title)

    print()


    for class_id, class_name in enumerate(CLASSES):

        print(
            f"{class_name:10s} "
            f"| clips={len(originals[class_id]):5d} "
            f"| segments={segment_counts[class_id]:5d}"
        )


print_distribution(
    "DISTRIBUIÇÃO TOTAL",
    samples
)


# ============================================================
# SPLIT POR SPEAKER
# ============================================================

speakers = sorted(
    set(
        sample["speaker"]
        for sample in samples
    )
)


rng = random.Random(
    SEED
)


rng.shuffle(
    speakers
)


n_speakers = len(
    speakers
)


n_train = int(
    n_speakers * 0.70
)


n_val = int(
    n_speakers * 0.15
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
    sample
    for sample in samples
    if sample["speaker"]
    in train_speakers
]


val_samples = [
    sample
    for sample in samples
    if sample["speaker"]
    in val_speakers
]


test_samples = [
    sample
    for sample in samples
    if sample["speaker"]
    in test_speakers
]


print()

print(
    "Speakers:"
)

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
# BALANCEAMENTO POR CLIP ORIGINAL
#
# Muito importante:
#
# Não vamos balancear simplesmente por segmento.
#
# Selecionamos o mesmo número de GRAVAÇÕES ORIGINAIS
# para cada classe.
# ============================================================

originals_by_class = defaultdict(
    lambda: defaultdict(list)
)


for sample in train_samples:

    class_id = sample["label"]

    original_id = sample[
        "original_id"
    ]


    originals_by_class[
        class_id
    ][
        original_id
    ].append(
        sample
    )


non_risk_originals = list(
    originals_by_class[0].keys()
)


risk_originals = list(
    originals_by_class[1].keys()
)


minimum_originals = min(
    len(non_risk_originals),
    len(risk_originals)
)


print()

print(
    "Balanceando usando",
    minimum_originals,
    "clips originais por classe."
)


rng.shuffle(
    non_risk_originals
)


rng.shuffle(
    risk_originals
)


selected_originals = {

    0: set(
        non_risk_originals[
            :minimum_originals
        ]
    ),

    1: set(
        risk_originals[
            :minimum_originals
        ]
    )
}


balanced_train = []


for class_id in [0, 1]:

    for original_id in selected_originals[
        class_id
    ]:

        balanced_train.extend(
            originals_by_class[
                class_id
            ][
                original_id
            ]
        )


rng.shuffle(
    balanced_train
)


train_samples = (
    balanced_train
)


print_distribution(
    "TRAIN BALANCEADO POR CLIP ORIGINAL",
    train_samples
)


# ============================================================
# MEL FILTERBANK
# ============================================================

mel_matrix = tf.signal.linear_to_mel_weight_matrix(

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


# ============================================================
# LOAD AUDIO
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

        minval=
            -MAX_GAIN_DB,

        maxval=
            MAX_GAIN_DB,
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

        dtype=tf.int32
    )


    def right():

        x = tf.pad(

            audio,

            [[shift, 0]]
        )


        return x[
            :AUDIO_SAMPLES
        ]


    def left():

        amount = (
            -shift
        )


        x = tf.pad(

            audio[
                amount:
            ],

            [[0, amount]]
        )


        return x[
            :AUDIO_SAMPLES
        ]


    return tf.cond(

        shift >= 0,

        right,

        left
    )


def add_random_noise(audio):

    signal_rms = tf.sqrt(

        tf.reduce_mean(
            tf.square(audio)
        )

        + 1e-8
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

        stddev=
            noise_rms
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


    use_noise = (

        tf.random.uniform([])
        <
        NOISE_PROBABILITY
    )


    audio = tf.cond(

        use_noise,

        lambda:
            add_random_noise(
                audio
            ),

        lambda:
            audio
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

        mel
        +
        1e-6
    )


    logmel = tf.expand_dims(

        logmel,

        axis=-1
    )


    return logmel


# ============================================================
# TRAIN PIPELINE
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


    features = audio_to_logmel(
        audio
    )


    label = tf.cast(
        label,
        tf.float32
    )


    return (
        features,
        label
    )


# ============================================================
# EVAL PIPELINE
# ============================================================

def process_eval(
    path,
    label
):

    audio = load_audio(
        path
    )


    features = audio_to_logmel(
        audio
    )


    label = tf.cast(
        label,
        tf.float32
    )


    return (
        features,
        label
    )


# ============================================================
# DATASET
# ============================================================

def create_dataset(
    data,
    training=False
):

    paths = [
        x["path"]
        for x in data
    ]


    labels = [
        x["label"]
        for x in data
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

            reshuffle_each_iteration=True
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
# VERIFICAR FEATURE
# ============================================================

for features, labels in train_ds.take(1):

    print()

    print(
        "Feature shape:",
        features.shape
    )


# ============================================================
# MODELO DS-CNN
# ============================================================

inputs = tf.keras.Input(

    shape=(49, 32, 1),

    name="logmel"
)


# ------------------------------------------------------------
# CONV INICIAL
# ------------------------------------------------------------

x = tf.keras.layers.Conv2D(

    16,

    kernel_size=3,

    padding="same",

    use_bias=False
)(inputs)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ------------------------------------------------------------
# DS BLOCK 1
# ------------------------------------------------------------

x = tf.keras.layers.DepthwiseConv2D(

    kernel_size=3,

    strides=(2, 2),

    padding="same",

    use_bias=False
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


x = tf.keras.layers.Conv2D(

    24,

    kernel_size=1,

    use_bias=False
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ------------------------------------------------------------
# DS BLOCK 2
# ------------------------------------------------------------

x = tf.keras.layers.DepthwiseConv2D(

    kernel_size=3,

    strides=(2, 2),

    padding="same",

    use_bias=False
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


x = tf.keras.layers.Conv2D(

    32,

    kernel_size=1,

    use_bias=False
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ------------------------------------------------------------
# DS BLOCK 3
# ------------------------------------------------------------

x = tf.keras.layers.DepthwiseConv2D(

    kernel_size=3,

    padding="same",

    use_bias=False
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


x = tf.keras.layers.Conv2D(

    48,

    kernel_size=1,

    use_bias=False
)(x)


x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ------------------------------------------------------------
# CLASSIFIER
# ------------------------------------------------------------

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
# COMPILE
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

        tf.keras.metrics.AUC(
            name="auc"
        ),
    ]
)


model.summary()


# ============================================================
# CALLBACKS
# ============================================================

callbacks = [

    tf.keras.callbacks.EarlyStopping(

        monitor=
            "val_loss",

        patience=10,

        restore_best_weights=True,

        verbose=1
    ),


    tf.keras.callbacks.ReduceLROnPlateau(

        monitor=
            "val_loss",

        patience=4,

        factor=0.5,

        min_lr=1e-6,

        verbose=1
    ),


    tf.keras.callbacks.ModelCheckpoint(

        filepath=str(
            BUILD
            /
            "cremad_model_v5_voicevote_best.keras"
        ),

        monitor=
            "val_loss",

        save_best_only=True,

        verbose=1
    ),
]


# ============================================================
# TRAIN
# ============================================================

print()

print(
    "================================"
)

print(
    "TREINANDO V5 VOICEVOTE"
)

print(
    "================================"
)


history = model.fit(

    train_ds,

    validation_data=
        val_ds,

    epochs=
        EPOCHS,

    callbacks=
        callbacks
)


# ============================================================
# SALVAR
# ============================================================

model.save(
    MODEL_PATH
)


print()

print(
    "Modelo salvo em:"
)

print(
    MODEL_PATH
)


# ============================================================
# PREDICT
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

        np.array(
            labels
        ).astype(
            np.int32
        ),

        np.array(
            probabilities
        )
    )


# ============================================================
# VALIDATION
# ============================================================

print()

print(
    "Calculando VALIDATION..."
)


val_labels, val_probs = predict_dataset(
    val_ds
)


val_auc = roc_auc_score(

    val_labels,

    val_probs
)


print()

print(
    f"Validation ROC-AUC: "
    f"{val_auc * 100:.2f}%"
)


# ============================================================
# THRESHOLD SEARCH
# ============================================================

print()

print(
    "Threshold | Accuracy | Precision | Recall | Specificity | Balanced | F1"
)

print(
    "----------------------------------------------------------------------"
)


best_threshold = None

best_balanced = -1.0


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

        f"{accuracy * 100:8.2f}% | "

        f"{precision * 100:9.2f}% | "

        f"{recall * 100:6.2f}% | "

        f"{specificity * 100:11.2f}% | "

        f"{balanced * 100:8.2f}% | "

        f"{f1 * 100:6.2f}%"
    )


    # Queremos recall razoavelmente alto,
    # mas sem destruir a especificidade.

    if recall >= 0.80:

        if balanced > best_balanced:

            best_balanced = balanced

            best_threshold = threshold


# ============================================================
# FALLBACK
# ============================================================

if best_threshold is None:

    print()

    print(
        "Nenhum threshold atingiu recall >= 80%."
    )

    print(
        "Selecionando maior Balanced Accuracy."
    )


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


        if balanced > best_balanced:

            best_balanced = balanced

            best_threshold = threshold


print()

print(
    "============================"
)

print(
    "THRESHOLD V5"
)

print(
    "============================"
)


print(
    f"{best_threshold:.3f}"
)


# ============================================================
# TEST
# ============================================================

print()

print(
    "Calculando TEST..."
)


test_labels, test_probs = predict_dataset(
    test_ds
)


test_auc = roc_auc_score(

    test_labels,

    test_probs
)


test_pred = (

    test_probs
    >=
    best_threshold

).astype(
    np.int32
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


false_positive_rate = (

    fp
    /
    (fp + tn + 1e-9)
)


balanced_accuracy = (

    recall
    +
    specificity

) / 2.0


# ============================================================
# FINAL
# ============================================================

print()

print(
    "============================"
)

print(
    "RESULTADO V5 - TEST"
)

print(
    "============================"
)


print(

    f"Threshold          : "
    f"{best_threshold:.3f}"
)


print(

    f"ROC-AUC            : "
    f"{test_auc * 100:.2f}%"
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

print(
    "Matriz:"
)

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
# SAVE CONFIG
# ============================================================

config = {

    "threshold":
        float(
            best_threshold
        ),

    "sample_rate":
        SAMPLE_RATE,

    "audio_samples":
        AUDIO_SAMPLES,

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

) as f:

    json.dump(

        config,

        f,

        indent=4
    )


print()

print(
    "Config salvo em:"
)

print(
    THRESHOLD_PATH
)
