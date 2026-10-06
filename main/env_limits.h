/*
 * env_limits.h - Limites de conforto/qualidade do ar e calibracao
 * Made by Igor Jensen - UFES - LAEEC
 *
 * TODOS os limites usados pelo firmware (terminal E MQTT) vem daqui.
 * Antes, o terminal usava 20-29 C / 45-85 % e o MQTT usava 20-26 C / 40-65 %,
 * por isso o terminal dizia "OK" em casos que o MQTT marcava como fora da faixa.
 */
#pragma once

/* ------------------------------------------------------------------
 * Norma de referencia: ABNT NBR 17037:2023
 *   Qualidade do ar interior em ambientes nao residenciais climatizados
 *   artificialmente. Substituiu a ANVISA RE n. 9/2003 em 25/07/2024.
 *   Adotada como referencia para a residencia (nao ha norma residencial).
 *   T: 21-26 C   UR: 35-65 %
 * ------------------------------------------------------------------ */
#define ENV_NORM_NAME          "ABNT NBR 17037:2023"
#define FLAG_TEMP_MIN_C        21.0f
#define FLAG_TEMP_MAX_C        26.0f
#define FLAG_HUMID_MIN_PCT     35.0f
#define FLAG_HUMID_MAX_PCT     65.0f

#define FLAG_DARK_LUX              10.0f    /* empirico: luz do comodo apagada        */
#define FLAG_VOC_ELEVATED_INDEX    150.0f   /* Sensirion VOC Index (100 = media 24 h)  */
#define FLAG_NOX_ELEVATED_INDEX    20.0f    /* Sensirion NOx Index (1 = linha de base) */

/* Variacao brusca de temperatura: desvio padrao entre bursts consecutivos
 * (ultimos BURST_CIRCULAR_SIZE bursts = ~1 min). 1.0 C de desvio ja e muito
 * para um ambiente interno. Antes era variancia > 5 C^2 dentro de 10 s, que
 * na pratica nunca disparava. */
#define FLAG_TEMP_STDDEV_LIMIT_C   1.0f

/* Gas Index: a Sensirion recomenda ignorar os indices no inicio
 * (blackout de ~45 s do algoritmo + aprendizado). */
#define GAS_INDEX_WARMUP_S         60

/* ------------------------------------------------------------------
 * Calibracao de temperatura
 *
 * A placa de sensores fica logo acima da placa principal (ESP32-C6 com
 * radio Thread + TinyML rodando sem parar, IP5306 e LDO) e ao lado do
 * aquecedor do SGP41. Isso aquece o SHT40 alguns decimos ate alguns graus.
 *
 * Como calibrar:
 *   1. Deixe a placa ligada >= 30 min no local de uso, ja dentro da case.
 *   2. Coloque um termometro/higrometro de referencia ao lado por 15 min.
 *   3. TEMP_OFFSET_C = media(T_placa) - media(T_referencia)   (ex.: 1.8)
 * A umidade e recalculada automaticamente para a temperatura corrigida
 * (o ar perto do sensor mais quente tem UR menor que a do ambiente).
 * Se ligado no USB carregando a bateria, o offset fica maior (IP5306 esquenta).
 * ------------------------------------------------------------------ */
#ifndef TEMP_OFFSET_C
#define TEMP_OFFSET_C              0.0f
#endif

/* ------------------------------------------------------------------
 * Terminal
 * ------------------------------------------------------------------ */
#ifndef TERM_USE_COLOR
#define TERM_USE_COLOR             1   /* cores ANSI (idf.py monitor / VS Code); 0 = texto puro */
#endif
#ifndef TERM_PRINT_EACH_SAMPLE
#define TERM_PRINT_EACH_SAMPLE     1   /* 1 = imprime as 20 amostras de cada burst */
#endif
#ifndef TERM_PRINT_SPECTRAL_SAMPLE
#define TERM_PRINT_SPECTRAL_SAMPLE 0   /* 1 = inclui F1..F8 em cada amostra      */
#endif
