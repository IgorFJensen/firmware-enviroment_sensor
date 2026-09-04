from pathlib import Path
from collections import defaultdict
import random

import numpy as np
import tensorflow as tf

from sklearn.metrics import (
    classification_report,
    confusion_matrix,
)

# ============================================================
# CONFIG
# ============================================================

DATASET = Path(__file__).parent / "dataset/processed"
BUILD = Path(__file__).parent / "build"
BUILD.mkdir(exist_ok=True)

CLASSES = [
    "normal",
    "positive",
    "distress",
    "danger",
]

SAMPLE_RATE = 16000
AUDIO_SAMPLES = 16000

FRAME_LENGTH = 480
FRAME_STEP = 320
FFT_LENGTH = 512

N_MELS = 32
LOWER_HZ = 80.0
UPPER_HZ = 7600.0

BATCH_SIZE = 64
EPOCHS = 50

SEED = 42

random.seed(SEED)
np.random.seed(SEED)
tf.random.set_seed(SEED)

# ============================================================
# CARREGAR LISTA DOS ARQUIVOS
# ============================================================

samples = []

for label, class_name in enumerate(CLASSES):

    folder = DATASET / class_name

    for path in folder.glob("*.wav"):

        # Ex:
        # 1001__1001_DFA_ANG_XX__seg00.wav

        speaker = path.name.split("__")[0]

        samples.append(
            {
                "path": str(path),
                "label": label,
                "speaker": speaker,
            }
        )

print()
print("Total:", len(samples))

# ============================================================
# SPLIT POR SPEAKER
# ============================================================

speakers = sorted(
    set(x["speaker"] for x in samples)
)

rng = random.Random(SEED)
rng.shuffle(speakers)

n = len(speakers)

n_train = int(n * 0.70)
n_val = int(n * 0.15)

train_speakers = set(
    speakers[:n_train]
)

val_speakers = set(
    speakers[n_train:n_train + n_val]
)

test_speakers = set(
    speakers[n_train + n_val:]
)

train_samples = [
    x for x in samples
    if x["speaker"] in train_speakers
]

val_samples = [
    x for x in samples
    if x["speaker"] in val_speakers
]

test_samples = [
    x for x in samples
    if x["speaker"] in test_speakers
]

print()
print("Speakers")
print("Train:", len(train_speakers))
print("Val:  ", len(val_speakers))
print("Test: ", len(test_speakers))

# ============================================================
# MOSTRAR DISTRIBUIÇÃO
# ============================================================

def print_distribution(name, data):

    counts = defaultdict(int)

    for x in data:
        counts[x["label"]] += 1

    print()
    print(name)

    for i, cls in enumerate(CLASSES):
        print(
            f"{cls:10s}: {counts[i]}"
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
# BALANCEAR APENAS TRAIN
# ============================================================

by_class = defaultdict(list)

for sample in train_samples:
    by_class[sample["label"]].append(sample)

minimum = min(
    len(by_class[i])
    for i in range(len(CLASSES))
)

balanced_train = []

for class_id in range(len(CLASSES)):

    class_samples = by_class[class_id]

    rng.shuffle(class_samples)

    balanced_train.extend(
        class_samples[:minimum]
    )

rng.shuffle(balanced_train)

train_samples = balanced_train

print()
print(
    "Amostras por classe após balanceamento:",
    minimum
)

print_distribution(
    "TRAIN BALANCEADO",
    train_samples
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
# AUDIO -> LOG MEL
# ============================================================

def load_audio(path):

    data = tf.io.read_file(path)

    audio, _ = tf.audio.decode_wav(
        data,
        desired_channels=1,
        desired_samples=AUDIO_SAMPLES,
    )

    audio = tf.squeeze(
        audio,
        axis=-1
    )

    return audio


def audio_to_logmel(audio):

    stft = tf.signal.stft(
        audio,
        frame_length=FRAME_LENGTH,
        frame_step=FRAME_STEP,
        fft_length=FFT_LENGTH,
        window_fn=tf.signal.hann_window,
        pad_end=False,
    )

    magnitude = tf.abs(stft)

    power = tf.square(
        magnitude
    )

    mel = tf.matmul(
        power,
        mel_matrix
    )

    # IMPORTANTE:
    # NÃO normalizamos individualmente
    # cada áudio nesta versão.

    logmel = tf.math.log(
        mel + 1e-6
    )

    logmel = tf.expand_dims(
        logmel,
        axis=-1
    )

    return logmel


def process_sample(path, label):

    audio = load_audio(path)

    features = audio_to_logmel(
        audio
    )

    return features, label

# ============================================================
# TF DATASET
# ============================================================

def create_dataset(
    sample_list,
    training=False
):

    paths = [
        x["path"]
        for x in sample_list
    ]

    labels = [
        x["label"]
        for x in sample_list
    ]

    ds = tf.data.Dataset.from_tensor_slices(
        (paths, labels)
    )

    if training:

        ds = ds.shuffle(
            len(sample_list),
            seed=SEED,
            reshuffle_each_iteration=True,
        )

    ds = ds.map(
        process_sample,
        num_parallel_calls=tf.data.AUTOTUNE,
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
    val_samples
)

test_ds = create_dataset(
    test_samples
)

# ============================================================
# MODELO
# ============================================================

inputs = tf.keras.Input(
    shape=(49, 32, 1)
)

# ------------------------------------------------------------
# BLOCO INICIAL
# ------------------------------------------------------------

x = tf.keras.layers.Conv2D(
    16,
    kernel_size=3,
    padding="same",
    use_bias=False,
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
    use_bias=False,
)(x)

x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)

x = tf.keras.layers.Conv2D(
    24,
    kernel_size=1,
    use_bias=False,
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
    use_bias=False,
)(x)

x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)

x = tf.keras.layers.Conv2D(
    32,
    kernel_size=1,
    use_bias=False,
)(x)

x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)

# ------------------------------------------------------------
# DS BLOCK 3
# ------------------------------------------------------------

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
    use_bias=False,
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
    len(CLASSES),
    activation="softmax",
)(x)

model = tf.keras.Model(
    inputs,
    outputs
)

model.compile(
    optimizer=tf.keras.optimizers.Adam(
        learning_rate=3e-4
    ),
    loss="sparse_categorical_crossentropy",
    metrics=["accuracy"],
)

model.summary()

# ============================================================
# CALLBACKS
# ============================================================

callbacks = [

    tf.keras.callbacks.EarlyStopping(
        monitor="val_loss",
        patience=8,
        restore_best_weights=True,
        verbose=1,
    ),

    tf.keras.callbacks.ReduceLROnPlateau(
        monitor="val_loss",
        patience=3,
        factor=0.5,
        min_lr=1e-6,
        verbose=1,
    ),
]

# ============================================================
# TRAIN
# ============================================================

history = model.fit(
    train_ds,
    validation_data=val_ds,
    epochs=EPOCHS,
    callbacks=callbacks,
)

# ============================================================
# AVALIAÇÃO
# ============================================================

print()
print("============================")
print("TRAIN")
print("============================")

train_loss, train_acc = model.evaluate(
    train_ds,
    verbose=0
)

print(
    f"Train accuracy: {train_acc * 100:.2f}%"
)

print()
print("============================")
print("VALIDATION")
print("============================")

val_loss, val_acc = model.evaluate(
    val_ds,
    verbose=0
)

print(
    f"Validation accuracy: {val_acc * 100:.2f}%"
)

print()
print("============================")
print("TEST")
print("============================")

test_loss, test_acc = model.evaluate(
    test_ds,
    verbose=0
)

print(
    f"Test accuracy: {test_acc * 100:.2f}%"
)

# ============================================================
# CONFUSION MATRIX
# ============================================================

y_true = []
y_pred = []

for features, labels in test_ds:

    predictions = model(
        features,
        training=False
    ).numpy()

    predictions = np.argmax(
        predictions,
        axis=1
    )

    y_true.extend(
        labels.numpy()
    )

    y_pred.extend(
        predictions
    )

print()
print(
    classification_report(
        y_true,
        y_pred,
        target_names=CLASSES,
        zero_division=0,
    )
)

cm = confusion_matrix(
    y_true,
    y_pred
)

print()
print("Matriz de confusão:")
print(cm)

print()
print("Distribuição das previsões:")

pred_counts = np.bincount(
    y_pred,
    minlength=len(CLASSES)
)

for i, cls in enumerate(CLASSES):

    print(
        f"{cls:10s}: {pred_counts[i]}"
    )

# ============================================================
# SALVAR
# ============================================================

output = BUILD / "cremad_model_v2.keras"

model.save(output)

print()
print(
    "Modelo salvo em:",
    output
)

