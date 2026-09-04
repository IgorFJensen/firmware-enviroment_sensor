from pathlib import Path
import random
import numpy as np
import tensorflow as tf

from sklearn.metrics import classification_report, confusion_matrix
from sklearn.utils.class_weight import compute_class_weight


# ============================================================
# CONFIGURAÇÃO
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

FRAME_LENGTH = 480       # 30 ms
FRAME_STEP = 320         # 20 ms
FFT_LENGTH = 512

N_MELS = 32
LOWER_HZ = 80.0
UPPER_HZ = 7600.0

BATCH_SIZE = 64
EPOCHS = 40
SEED = 42

random.seed(SEED)
np.random.seed(SEED)
tf.random.set_seed(SEED)


# ============================================================
# BUSCA DOS ARQUIVOS
# ============================================================

samples = []

for class_id, class_name in enumerate(CLASSES):

    folder = DATASET / class_name

    for path in folder.glob("*.wav"):

        # Nome:
        # 1001__1001_DFA_ANG_XX__seg00.wav
        speaker = path.name.split("__")[0]

        samples.append(
            {
                "path": str(path),
                "label": class_id,
                "speaker": speaker,
            }
        )


print()
print("Total de segmentos:", len(samples))


# ============================================================
# SPLIT POR SPEAKER
# ============================================================

speakers = sorted(set(x["speaker"] for x in samples))

random.shuffle(speakers)

n = len(speakers)

n_train = int(n * 0.70)
n_val = int(n * 0.15)

train_speakers = set(speakers[:n_train])

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

print()
print("Segmentos:")
print("Train:", len(train_samples))
print("Val:  ", len(val_samples))
print("Test: ", len(test_samples))


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

def load_audio(path):

    audio_binary = tf.io.read_file(path)

    audio, sr = tf.audio.decode_wav(
        audio_binary,
        desired_channels=1,
        desired_samples=AUDIO_SAMPLES,
    )

    audio = tf.squeeze(audio, axis=-1)

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

    spectrogram = tf.abs(stft)

    power = tf.square(spectrogram)

    mel = tf.matmul(
        power,
        mel_matrix,
    )

    logmel = tf.math.log(
        mel + 1e-6
    )

    # Normalização por janela.
    mean = tf.reduce_mean(logmel)
    std = tf.math.reduce_std(logmel)

    logmel = (
        logmel - mean
    ) / (std + 1e-6)

    logmel = tf.expand_dims(
        logmel,
        axis=-1
    )

    return logmel


def process_sample(path, label):

    audio = load_audio(path)

    features = audio_to_logmel(audio)

    return features, label


# ============================================================
# DATASET TENSORFLOW
# ============================================================

def create_dataset(sample_list, training=False):

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
            seed=SEED
        )

    ds = ds.map(
        process_sample,
        num_parallel_calls=tf.data.AUTOTUNE
    )

    ds = ds.batch(BATCH_SIZE)

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
# CLASS WEIGHTS
# ============================================================

train_labels = np.array(
    [
        x["label"]
        for x in train_samples
    ]
)

class_weights_array = compute_class_weight(
    class_weight="balanced",
    classes=np.arange(len(CLASSES)),
    y=train_labels,
)

class_weights = {
    i: float(w)
    for i, w in enumerate(class_weights_array)
}

print()
print("Class weights:")

for i, w in class_weights.items():
    print(
        f"{CLASSES[i]:10s}: {w:.3f}"
    )


# ============================================================
# MODELO DS-CNN
# ============================================================

input_shape = (
    49,
    N_MELS,
    1
)

inputs = tf.keras.Input(
    shape=input_shape
)

x = tf.keras.layers.Conv2D(
    8,
    kernel_size=3,
    strides=(2, 2),
    padding="same",
    activation="relu",
)(inputs)


x = tf.keras.layers.DepthwiseConv2D(
    kernel_size=3,
    padding="same",
    activation="relu",
)(x)

x = tf.keras.layers.Conv2D(
    16,
    kernel_size=1,
    activation="relu",
)(x)


x = tf.keras.layers.DepthwiseConv2D(
    kernel_size=3,
    strides=(2, 2),
    padding="same",
    activation="relu",
)(x)

x = tf.keras.layers.Conv2D(
    24,
    kernel_size=1,
    activation="relu",
)(x)


x = tf.keras.layers.DepthwiseConv2D(
    kernel_size=3,
    padding="same",
    activation="relu",
)(x)

x = tf.keras.layers.Conv2D(
    32,
    kernel_size=1,
    activation="relu",
)(x)


x = tf.keras.layers.GlobalAveragePooling2D()(x)

outputs = tf.keras.layers.Dense(
    len(CLASSES),
    activation="softmax"
)(x)


model = tf.keras.Model(
    inputs,
    outputs
)


model.compile(
    optimizer=tf.keras.optimizers.Adam(
        learning_rate=0.001
    ),
    loss="sparse_categorical_crossentropy",
    metrics=["accuracy"],
)


model.summary()


# ============================================================
# TREINAMENTO
# ============================================================

callbacks = [

    tf.keras.callbacks.EarlyStopping(
        monitor="val_loss",
        patience=7,
        restore_best_weights=True,
    ),

    tf.keras.callbacks.ReduceLROnPlateau(
        monitor="val_loss",
        patience=3,
        factor=0.5,
        min_lr=1e-6,
    ),

]


history = model.fit(
    train_ds,
    validation_data=val_ds,
    epochs=EPOCHS,
    class_weight=class_weights,
    callbacks=callbacks,
)


# ============================================================
# TESTE
# ============================================================

loss, accuracy = model.evaluate(
    test_ds
)

print()
print("============================")
print("RESULTADO")
print("============================")
print(
    f"Test accuracy: {accuracy * 100:.2f}%"
)


y_true = []
y_pred = []

for features, labels in test_ds:

    predictions = model.predict(
        features,
        verbose=0
    )

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
        target_names=CLASSES
    )
)

print("Matriz de confusão:")

print(
    confusion_matrix(
        y_true,
        y_pred
    )
)


# ============================================================
# SALVAR
# ============================================================

model.save(
    BUILD / "cremad_model.keras"
)

print()
print(
    "Modelo salvo em:",
    BUILD / "cremad_model.keras"
)
