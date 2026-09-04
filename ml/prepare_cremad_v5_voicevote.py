from pathlib import Path
import csv
import wave

import numpy as np


# ============================================================
# PATHS
# ============================================================

CREMAD_ROOT = (
    Path.home()
    / "Documents"
    / "datasets"
    / "CREMA-D"
)

AUDIO_DIR = (
    CREMAD_ROOT
    / "AudioWAV"
)

SUMMARY_CSV = (
    CREMAD_ROOT
    / "processedResults"
    / "summaryTable.csv"
)

DEST = (
    Path(__file__).parent
    / "dataset"
    / "voicevote"
)


# ============================================================
# AUDIO CONFIG
# ============================================================

SAMPLE_RATE = 16000

WINDOW_SAMPLES = 16000

# 0.5 segundo de overlap
HOP_SAMPLES = 8000

# Último pedaço precisa ter ao menos 0.75 s
MIN_LAST_SAMPLES = 12000


# ============================================================
# VOICEVOTE -> NOSSA CLASSE
# ============================================================

#
# N = Neutral
# H = Happy
#
# S = Sad
#
# A = Anger
# F = Fear
#
# D = Disgust -> não usamos inicialmente
#

VOICE_MAP = {

    "N": "non_risk",
    "H": "non_risk",

    "S": "distress",

    "A": "danger",
    "F": "danger",
}


# ============================================================
# WAV
# ============================================================

def read_wav(path):

    with wave.open(
        str(path),
        "rb"
    ) as wf:

        channels = wf.getnchannels()

        sample_width = wf.getsampwidth()

        sample_rate = wf.getframerate()

        frames = wf.readframes(
            wf.getnframes()
        )

    if channels != 1:

        raise ValueError(
            f"{path.name}: "
            f"esperado mono, "
            f"recebido {channels}"
        )

    if sample_width != 2:

        raise ValueError(
            f"{path.name}: "
            f"esperado PCM16"
        )

    if sample_rate != SAMPLE_RATE:

        raise ValueError(
            f"{path.name}: "
            f"esperado 16000 Hz, "
            f"recebido {sample_rate}"
        )

    return np.frombuffer(
        frames,
        dtype=np.int16
    )


def save_wav(
    path,
    samples
):

    path.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    with wave.open(
        str(path),
        "wb"
    ) as wf:

        wf.setnchannels(1)

        wf.setsampwidth(2)

        wf.setframerate(
            SAMPLE_RATE
        )

        wf.writeframes(
            samples
            .astype(np.int16)
            .tobytes()
        )


def normalize_length(samples):

    if len(samples) >= WINDOW_SAMPLES:

        return samples[
            :WINDOW_SAMPLES
        ]

    output = np.zeros(
        WINDOW_SAMPLES,
        dtype=np.int16
    )

    output[
        :len(samples)
    ] = samples

    return output


# ============================================================
# LER SUMMARY TABLE
# ============================================================

def load_voice_votes():

    votes = {}

    with open(
        SUMMARY_CSV,
        newline="",
        encoding="utf-8-sig"
    ) as f:

        reader = csv.DictReader(f)

        print(
            "Colunas encontradas:"
        )

        print(
            reader.fieldnames
        )

        for row in reader:

            filename = (
                row["FileName"]
                .strip()
            )

            vote = (
                row["VoiceVote"]
                .strip()
            )

            # remove extensão se houver
            filename = (
                Path(filename)
                .stem
            )

            votes[
                filename
            ] = vote

    return votes


# ============================================================
# MAIN
# ============================================================

def main():

    if not AUDIO_DIR.exists():

        raise RuntimeError(
            f"AudioWAV não encontrado:\n"
            f"{AUDIO_DIR}"
        )

    if not SUMMARY_CSV.exists():

        raise RuntimeError(
            f"summaryTable.csv "
            f"não encontrado:\n"
            f"{SUMMARY_CSV}"
        )


    votes = load_voice_votes()


    print()
    print(
        "VoiceVotes carregados:",
        len(votes)
    )


    counts_original = {
        "non_risk": 0,
        "distress": 0,
        "danger": 0,
    }


    counts_segments = {
        "non_risk": 0,
        "distress": 0,
        "danger": 0,
    }


    skipped_tie = 0

    skipped_disgust = 0

    skipped_missing = 0

    total_audio = 0


    # ========================================================
    # PROCESSAR
    # ========================================================

    for wav_path in sorted(
        AUDIO_DIR.glob("*.wav")
    ):

        total_audio += 1

        stem = wav_path.stem


        # ================================================
        # procurar VoiceVote
        # ================================================

        if stem not in votes:

            skipped_missing += 1

            continue


        vote = votes[stem]


        # ================================================
        # Empate
        #
        # Ex:
        #
        # A:F
        #
        # Não queremos label ambíguo.
        # ================================================

        if ":" in vote:

            skipped_tie += 1

            continue


        # ================================================
        # Disgust ou qualquer categoria não usada
        # ================================================

        if vote not in VOICE_MAP:

            skipped_disgust += 1

            continue


        target_class = (
            VOICE_MAP[vote]
        )


        # ================================================
        # AUDIO
        # ================================================

        try:

            audio = read_wav(
                wav_path
            )

        except Exception as e:

            print(
                "ERRO:",
                e
            )

            continue


        counts_original[
            target_class
        ] += 1


        # Speaker ID
        speaker = (
            stem
            .split("_")[0]
        )


        segment_index = 0


        for start in range(
            0,
            len(audio),
            HOP_SAMPLES
        ):

            segment = audio[
                start:
                start + WINDOW_SAMPLES
            ]


            if (
                len(segment)
                <
                MIN_LAST_SAMPLES
            ):

                break


            segment = normalize_length(
                segment
            )


            output_name = (

                f"{speaker}"
                f"__{stem}"
                f"__seg"
                f"{segment_index:02d}"
                f".wav"
            )


            output_path = (

                DEST
                /
                target_class
                /
                output_name
            )


            save_wav(
                output_path,
                segment
            )


            counts_segments[
                target_class
            ] += 1


            segment_index += 1


    # ========================================================
    # REPORT
    # ========================================================

    print()
    print(
        "================================"
    )

    print(
        "CREMA-D V5 VOICEVOTE"
    )

    print(
        "================================"
    )

    print()

    print(
        "AudioWAV encontrados:",
        total_audio
    )

    print()

    print(
        "Arquivos originais utilizados:"
    )


    for name in [
        "non_risk",
        "distress",
        "danger"
    ]:

        print(
            f"{name:12s}: "
            f"{counts_original[name]}"
        )


    print()

    print(
        "Segmentos de 1 segundo:"
    )


    for name in [
        "non_risk",
        "distress",
        "danger"
    ]:

        print(
            f"{name:12s}: "
            f"{counts_segments[name]}"
        )


    print()

    print(
        "Ignorados:"
    )

    print(
        "Empate VoiceVote :",
        skipped_tie
    )

    print(
        "Disgust/outros    :",
        skipped_disgust
    )

    print(
        "Sem VoiceVote     :",
        skipped_missing
    )

    print()

    print(
        "Destino:"
    )

    print(
        DEST.resolve()
    )


if __name__ == "__main__":

    main()
