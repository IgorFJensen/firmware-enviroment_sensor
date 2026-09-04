# Integração do classificador de risco V6.1

Este pacote contém o modelo V6.1 INT8 incorporado e o runtime para ESP32-C6.

## Configuração congelada

- Áudio: PCM16 mono, 16 kHz, janela de 16.000 amostras.
- STFT: frame 480, hop 320, FFT 512, Hann periódica.
- Log-Mel: 49 x 32, faixa de 80 a 7.600 Hz.
- Entrada INT8: escala 0.09267431497573853, zero-point 21.
- Saída INT8: escala 0.00390625, zero-point -128.
- Ativação: probabilidade >= 0.450 em pelo menos 3 das últimas 5 janelas.
- Desativação: menos de 2 das 5 janelas acima de 0.350.

O runtime usa os parâmetros de quantização lidos do próprio modelo e interrompe
a inicialização caso o tipo ou o shape não correspondam ao esperado.

## Compilar e gravar

Na raiz do projeto:

```bash
idf.py fullclean
idf.py reconfigure
idf.py build
idf.py -p SUA_PORTA flash monitor
```

O log esperado começa com `TINYML_V61: V6.1 pronto`. Depois, cada janela mostra
a probabilidade, média, votos e estado. Um risco só é confirmado após votação.

## Trocar o modelo futuramente

Coloque o novo arquivo em:

```text
ml/build/cremad_model_v61_voicevote_int8.tflite
```

Execute:

```bash
python3 ml/export_model.py
idf.py build
```

Não altere o DSP do firmware sem aplicar exatamente a mesma mudança no treino.

