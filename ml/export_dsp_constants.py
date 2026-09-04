from pathlib import Path

import tensorflow as tf


ROOT = Path(__file__).parent

OUTPUT = (
    ROOT.parent
    / "components"
    / "tinyml_runtime"
    / "dsp_constants.h"
)


# ============================================================
# CONFIGURAÇÃO
# ============================================================

SAMPLE_RATE = 16000

FRAME_LENGTH = 480

FFT_LENGTH = 512

N_MELS = 32

LOWER_HZ = 80.0

UPPER_HZ = 7600.0


# ============================================================
# GARANTIR QUE A PASTA EXISTE
# ============================================================

OUTPUT.parent.mkdir(
    parents=True,
    exist_ok=True
)


# ============================================================
# HANN WINDOW
#
# Mesmo tf.signal.hann_window usado no treinamento.
# periodic=True é o padrão do TensorFlow.
# ============================================================

hann = tf.signal.hann_window(
    FRAME_LENGTH,
    periodic=True,
    dtype=tf.float32,
).numpy()


# ============================================================
# MEL FILTERBANK
#
# Mesmo método usado na V5.
#
# FFT 512
# bins positivos = 257
# 32 filtros Mel
# 80 Hz até 7600 Hz
# ============================================================

mel = tf.signal.linear_to_mel_weight_matrix(
    num_mel_bins=N_MELS,

    num_spectrogram_bins=
        FFT_LENGTH // 2 + 1,

    sample_rate=SAMPLE_RATE,

    lower_edge_hertz=
        LOWER_HZ,

    upper_edge_hertz=
        UPPER_HZ,

    dtype=tf.float32,
).numpy()


print()
print("Hann shape:", hann.shape)
print("Mel shape :", mel.shape)


# ============================================================
# EXPORTAR HEADER
# ============================================================

with open(
    OUTPUT,
    "w"
) as f:

    f.write(
        "#pragma once\n\n"
    )

    f.write(
        "#include <stdint.h>\n\n"
    )

    f.write(
        "static constexpr int "
        "TINYML_FRAME_LENGTH = 480;\n"
    )

    f.write(
        "static constexpr int "
        "TINYML_FFT_LENGTH = 512;\n"
    )

    f.write(
        "static constexpr int "
        "TINYML_FFT_BINS = 257;\n"
    )

    f.write(
        "static constexpr int "
        "TINYML_MEL_BINS = 32;\n\n"
    )


    # ========================================================
    # HANN
    # ========================================================

    f.write(
        "alignas(16) "
        "static const float "
        "kTinymlHann[480] = {\n"
    )

    for i, value in enumerate(hann):

        f.write(
            f"{value:.9e}f"
        )

        if i != len(hann) - 1:
            f.write(",")

        if i % 6 == 5:
            f.write("\n")
        else:
            f.write(" ")

    f.write(
        "\n};\n\n"
    )


    # ========================================================
    # MEL MATRIX
    #
    # [257 FFT bins][32 Mel bins]
    # ========================================================

    f.write(
        "alignas(16) "
        "static const float "
        "kTinymlMel[257][32] = {\n"
    )

    for fft_bin in range(
        mel.shape[0]
    ):

        f.write(
            "    {"
        )

        for mel_bin in range(
            mel.shape[1]
        ):

            value = mel[
                fft_bin,
                mel_bin
            ]

            f.write(
                f"{value:.9e}f"
            )

            if (
                mel_bin
                !=
                mel.shape[1] - 1
            ):
                f.write(", ")

        f.write(
            "}"
        )

        if (
            fft_bin
            !=
            mel.shape[0] - 1
        ):
            f.write(",")

        f.write(
            "\n"
        )

    f.write(
        "};\n"
    )


print()
print("Arquivo gerado:")
print(OUTPUT)
