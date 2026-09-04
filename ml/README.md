# TinyML acústico — primeira etapa

Objetivo inicial: classificar janelas de 1 segundo em 5 classes observáveis:

1. `background`
2. `normal_speech`
3. `scream_panic`
4. `cry_groan`
5. `laugh_positive`

O treino ocorre no PC. O ESP32-C6 receberá apenas o modelo INT8 e o pipeline DSP equivalente.

## 1. Ambiente Python

```bash
cd ml
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 2. Dataset

Coloque WAVs em:

```text
dataset/raw/
├── background/
├── normal_speech/
├── scream_panic/
├── cry_groan/
└── laugh_positive/
```

Use nomes iniciados pelo ID do falante:

```text
spk01__grito_001.wav
spk01__fala_001.wav
spk02__choro_001.wav
```

Isso permite split `speaker-disjoint`, evitando que a mesma voz apareça em treino e teste.

Para uma coleta rápida usando o microfone do PC:

```bash
python collect_pc.py normal_speech --speaker spk01 --count 20
```

Para o dataset final, prefira capturar com o mesmo caminho acústico do produto/ICS-43434.

## 3. Padronização

```bash
python prepare_dataset.py
```

O script converte para mono, 16 kHz, PCM16 e janelas de 1 segundo sem normalização de pico por clip.

## 4. Treino + quantização INT8

```bash
python train_acoustic.py
```

Arquivos principais gerados:

```text
build/acoustic_best.keras
build/acoustic_int8.tflite
build/model_metadata.json
```

O script também imprime tamanho do `.tflite`, matriz de confusão e accuracy do modelo INT8.

## 5. Exportar para o firmware

```bash
python export_model.py
```

Isso substitui:

```text
../components/tinyml_model/model_data.cc
```

**Importante:** nesta versão starter o firmware ainda não instancia o `MicroInterpreter`. Depois do primeiro modelo, a próxima etapa é implementar Log-Mel no C6 com os mesmos parâmetros de `config.py` e então conectar `esp-tflite-micro`.

## Parâmetros congelados do DSP

```text
Sample rate:   16000 Hz
Janela:        1.0 s
Frame:         480 amostras (30 ms)
Hop:           320 amostras (20 ms)
FFT:           512
Mel bins:      32
Faixa Mel:     80 .. 7600 Hz
Entrada CNN:   49 x 32 x 1
```

Qualquer mudança aqui precisa ser reproduzida exatamente no firmware.
