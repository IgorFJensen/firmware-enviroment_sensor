from pathlib import Path
from collections import defaultdict
import random
import json

import numpy as np
import tensorflow as tf

from sklearn.metrics import (
    classification_report,
    confusion_matrix,
)


# ============================================================
# CONFIGURAÇÃO
# ============================================================

DATASET = Path(__file__).parent / "dataset/processed"
BUILD = Path(__file__).parent / "build"
BUILD.mkdir(exist_ok=True)

CLASSES = [
    "non_risk",
    "distress",
    "danger",
]

# Pastas do dataset V2 -> classes V3
SOURCE_MAP = {
    "normal": 0,
    "positive": 0,
    "distress": 1,
    "danger": 2,
}

SAMPLE_RATE = 16000
AUDIO_SAMPLES = 16000

FRAME_LENGTH = 480       # 30 ms
FRAME_STEP = 320         # 20 ms
FFT_LENGTH = 512

N_MELS = 32
LOWER_HZ = 80.0
UPPER_HZ = 7600.0

BATCH_SIZE = 64
EPOCHS = 60

SEED = 42

random.seed(SEED)
np.random.seed(SEED)
tf.random.set_seed(SEED)


# ============================================================
# AUGMENTATION
# ============================================================

# ±6 dB
MAX_GAIN_DB = 6.0

# ±100 ms
MAX_SHIFT_SAMPLES = 1600

# Ruído branco equivalente aproximadamente a:
MIN_SNR_DB = 15.0
MAX_SNR_DB = 30.0

NOISE_PROBABILITY = 0.70


# ============================================================
# CARREGAR ARQUIVOS
# ============================================================

samples = []

for folder_name, class_id in SOURCE_MAP.items():

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
        # Primeiro campo = speaker
        speaker = path.name.split("__")[0]

        samples.append(
            {
                "path": str(path),
                "label": class_id,
                "speaker": speaker,
                "source": folder_name,
            }
        )


print()
print("================================")
print("CREMA-D V3")
print("================================")
print("Total de segmentos:", len(samples))


# ============================================================
# DISTRIBUIÇÃO GERAL
# ============================================================

def print_distribution(title, data):

    counts = defaultdict(int)

    for x in data:
        counts[x["label"]] += 1

    print()
    print(title)

    for i, class_name in enumerate(CLASSES):

        print(
            f"{class_name:12s}: {counts[i]}"
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
        x["speaker"]
        for x in samples
    )
)

rng = random.Random(SEED)
rng.shuffle(speakers)

n_speakers = len(speakers)

n_train = int(n_speakers * 0.70)
n_val = int(n_speakers * 0.15)

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
print("Speakers:")
print("Train:", len(train_speakers))
print("Val:  ", len(val_speakers))
print("Test: ", len(test_speakers))


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


minimum_samples = min(
    len(by_class[i])
    for i in range(len(CLASSES))
)


balanced_train = []

for class_id in range(len(CLASSES)):

    class_samples = list(
        by_class[class_id]
    )

    rng.shuffle(class_samples)

    balanced_train.extend(
        class_samples[:minimum_samples]
    )


rng.shuffle(balanced_train)

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
# LEITURA DE WAV
# ============================================================

def load_audio(path):

    binary = tf.io.read_file(path)

    audio, sample_rate = tf.audio.decode_wav(
        binary,
        desired_channels=1,
        desired_samples=AUDIO_SAMPLES,
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

    return audio * gain


def random_time_shift(audio):

    shift = tf.random.uniform(
        [],
        minval=-MAX_SHIFT_SAMPLES,
        maxval=MAX_SHIFT_SAMPLES + 1,
        dtype=tf.int32,
    )

    def shift_right():

        shifted = tf.pad(
            audio,
            [[shift, 0]]
        )

        return shifted[:AUDIO_SAMPLES]

    def shift_left():

        amount = -shift

        shifted = tf.pad(
            audio[amount:],
            [[0, amount]]
        )

        return shifted[:AUDIO_SAMPLES]

    return tf.cond(
        shift >= 0,
        shift_right,
        shift_left,
    )


def add_random_noise(audio):

    signal_rms = tf.sqrt(
        tf.reduce_mean(
            tf.square(audio)
        ) + 1e-8
    )

    snr_db = tf.random.uniform(
        [],
        minval=MIN_SNR_DB,
        maxval=MAX_SNR_DB,
    )

    snr_linear = tf.pow(
        10.0,
        snr_db / 20.0
    )

    noise_rms = (
        signal_rms /
        snr_linear
    )

    noise = tf.random.normal(
        tf.shape(audio),
        mean=0.0,
        stddev=noise_rms,
    )

    return audio + noise


def augment_audio(audio):

    # ganho
    audio = random_gain(audio)

    # deslocamento temporal
    audio = random_time_shift(audio)

    # ruído com 70% de probabilidade
    use_noise = (
        tf.random.uniform([]) <
        NOISE_PROBABILITY
    )

    audio = tf.cond(
        use_noise,
        lambda: add_random_noise(audio),
        lambda: audio,
    )

    # PCM normalizado do TensorFlow deve
    # permanecer no intervalo [-1, +1]
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

    logmel = tf.math.log(
        mel + 1e-6
    )

    # IMPORTANTE:
    #
    # NÃO fazemos normalização
    # individual por áudio.
    #
    # Queremos preservar diferenças
    # de energia/intensidade.

    logmel = tf.expand_dims(
        logmel,
        axis=-1
    )

    return logmel


# ============================================================
# PROCESSAMENTO TRAIN / TEST
# ============================================================

def process_train(path, label):

    audio = load_audio(path)

    audio = augment_audio(
        audio
    )

    features = audio_to_logmel(
        audio
    )

    return features, label


def process_eval(path, label):

    audio = load_audio(path)

    features = audio_to_logmel(
        audio
    )

    return features, label


# ============================================================
# DATASET TF
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
            process_train,
            num_parallel_calls=tf.data.AUTOTUNE,
        )

    else:

        ds = ds.map(
            process_eval,
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
    val_samples,
    training=False
)

test_ds = create_dataset(
    test_samples,
    training=False
)


# ============================================================
# VERIFICAR FORMATO DA FEATURE
# ============================================================

for features, labels in train_ds.take(1):

    print()
    print(
        "Shape das features:",
        features.shape
    )

    print(
        "Shape esperado: (batch, 49, 32, 1)"
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
    padding="same",
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
    padding="same",
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
    padding="same",
    use_bias=False,
)(x)

x = tf.keras.layers.BatchNormalization()(x)

x = tf.keras.layers.ReLU()(x)


# ------------------------------------------------------------
# CLASSIFICADOR
# ------------------------------------------------------------

x = tf.keras.layers.GlobalAveragePooling2D()(x)

x = tf.keras.layers.Dropout(
    0.20
)(x)

outputs = tf.keras.layers.Dense(
    len(CLASSES),
    activation="softmax",
    name="output"
)(x)


model = tf.keras.Model(
    inputs,
    outputs
)


# ============================================================
# COMPILAÇÃO
# ============================================================

model.compile(

    optimizer=tf.keras.optimizers.Adam(
        learning_rate=3e-4
    ),

    loss="sparse_categorical_crossentropy",

    metrics=[
        "accuracy"
    ],
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
            BUILD /
            "cremad_model_v3_best.keras"
        ),

        monitor="val_loss",

        save_best_only=True,

        verbose=1,
    ),
]


# ============================================================
# TREINAMENTO
# ============================================================

print()
print("================================")
print("INICIANDO TREINAMENTO V3")
print("================================")
print()


history = model.fit(

    train_ds,

    validation_data=val_ds,

    epochs=EPOCHS,

    callbacks=callbacks,
)


# ============================================================
# RESULTADOS
# ============================================================

print()
print("============================")
print("TRAIN")
print("============================")

train_loss, train_accuracy = model.evaluate(
    train_ds,
    verbose=0
)

print(
    f"Train accuracy: "
    f"{train_accuracy * 100:.2f}%"
)


print()
print("============================")
print("VALIDATION")
print("============================")

val_loss, val_accuracy = model.evaluate(
    val_ds,
    verbose=0
)

print(
    f"Validation accuracy: "
    f"{val_accuracy * 100:.2f}%"
)


print()
print("============================")
print("TEST")
print("============================")

test_loss, test_accuracy = model.evaluate(
    test_ds,
    verbose=0
)

print(
    f"Test accuracy: "
    f"{test_accuracy * 100:.2f}%"
)


# ============================================================
# PREVISÕES
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


# ============================================================
# CLASSIFICATION REPORT
# ============================================================

print()

print(
    classification_report(
        y_true,
        y_pred,
        target_names=CLASSES,
        zero_division=0,
    )
)


# ============================================================
# CONFUSION MATRIX
# ============================================================

cm = confusion_matrix(
    y_true,
    y_pred
)

print()
print("Matriz de confusão:")
print()

print(cm)


# ============================================================
# DISTRIBUIÇÃO DAS PREVISÕES
# ============================================================

print()
print("Distribuição das previsões:")

prediction_counts = np.bincount(
    y_pred,
    minlength=len(CLASSES)
)

for i, class_name in enumerate(CLASSES):

    print(
        f"{class_name:12s}: "
        f"{prediction_counts[i]}"
    )


# ============================================================
# MÉTRICA ESPECIAL: RISCO
# ============================================================

#
# Para a aplicação:
#
# NON_RISK = 0
#
# DISTRESS + DANGER = RISCO
#

true_risk = np.array(y_true) != 0
pred_risk = np.array(y_pred) != 0


tp = np.sum(
    true_risk & pred_risk
)

tn = np.sum(
    ~true_risk & ~pred_risk
)

fp = np.sum(
    ~true_risk & pred_risk
)

fn = np.sum(
    true_risk & ~pred_risk
)


risk_accuracy = (
    (tp + tn) /
    len(y_true)
)

risk_recall = (
    tp /
    (tp + fn + 1e-9)
)

risk_precision = (
    tp /
    (tp + fp + 1e-9)
)


print()
print("============================")
print("MÉTRICA BINÁRIA DE RISCO")
print("============================")

print(
    f"Risk accuracy : "
    f"{risk_accuracy * 100:.2f}%"
)

print(
    f"Risk recall   : "
    f"{risk_recall * 100:.2f}%"
)

print(
    f"Risk precision: "
    f"{risk_precision * 100:.2f}%"
)

print()

print("TP:", tp)
print("TN:", tn)
print("FP:", fp)
print("FN:", fn)


# ============================================================
# SALVAR MODELO
# ============================================================

output_model = (
    BUILD /
    "cremad_model_v3.keras"
)

model.save(
    output_model
)


# ============================================================
# SALVAR LABELS
# ============================================================

labels_file = (
    BUILD /
    "cremad_v3_labels.json"
)

with open(
    labels_file,
    "w"
) as f:

    json.dump(
        CLASSES,
        f,
        indent=4
    )


print()
print(
    "Modelo salvo em:",
    output_model
)

print(
    "Labels salvos em:",
    labels_file
)
