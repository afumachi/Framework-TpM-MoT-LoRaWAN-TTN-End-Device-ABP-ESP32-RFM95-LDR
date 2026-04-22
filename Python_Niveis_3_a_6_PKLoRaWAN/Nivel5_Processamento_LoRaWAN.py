# =============================================================================
# Framework TpM - N5 - PROCESSAMENTO DE DADOS IoT LoRa  (v3 — TTN/CSV)
# =============================================================================
#
# Lê em tempo real o arquivo gerado pelo N3 (coleta TTN via MQTT):
#   - Nivel4.csv  →  timestamp,uplink_counter,ldr_value,intensity,
#                    rssi,snr,gateway_id,MY_ID,bateria,temperatura,
#                    umidade,latitude,longitude
#
# Calcula médias acumuladas e salva UMA vez por ciclo:
#   - medidas_aplicacao_media.txt  → lido pelo N6
#   - medidas_gerencia_media.txt   → lido pelo N6
#
# Média de RSSI (dBm) e SNR (dB) via domínio linear (mW):
#   - RSSI: converte dBm → mW, acumula, calcula média, reconverte para dBm
#   - SNR:  converte dB  → razão linear, acumula, calcula média, reconverte para dB
#   Isso evita a distorção que a média aritmética direta em dB produz.
#
# Formato medidas_aplicacao_media.txt (escrito aqui, lido pelo N6):
#   timestamp;lum;lum_media;umi;umi_media;temp;temp_media;lat;lon;qtd_amostras
#
# Formato medidas_gerencia_media.txt (escrito aqui, lido pelo N6):
#   timestamp;rssi;rssi_media_dbm;snr;snr_media_db;bateria;qtd_amostras
# =============================================================================

import time
import os
import math
from datetime import datetime

# ---------- Configurações ----------
ARQUIVO_CSV_IN  = "Nivel4.csv"          # Gerado pelo N3 (coleta TTN)
ARQUIVO_APP_OUT = "medidas_aplicacao_media.txt"
ARQUIVO_GER_OUT = "medidas_gerencia_media.txt"

INTERVALO_LEITURA = 2.0   # segundos entre ciclos
MAX_AMOSTRAS      = 100   # limite das listas (evita crescimento infinito)

# ---------- Rastreamento de posição (tail no CSV) ----------
pos_csv = 0

# ---------- Acumuladores — Aplicação ----------
lum_lista  = []
lat_ultimo = 0.0
lon_ultimo = 0.0
alt_ultimo = 0.0

# ---------- Acumuladores — Gerência ----------
# RSSI e SNR armazenados em mW/linear para média correta
rssi_mw_lista  = []   # RSSI em mW
snr_lin_lista  = []   # SNR como razão linear
rssi_dbm_ultimo = 0.0
snr_db_ultimo   = 0.0
bat_ultimo      = 0.0


# =============================================================================
# Conversões dBm ↔ mW  /  dB ↔ linear
# =============================================================================

def dbm_para_mw(dbm: float) -> float:
    """Converte dBm para mW. Seguro para valores negativos (ex: -120 dBm)."""
    return 10.0 ** (dbm / 10.0)


def mw_para_dbm(mw: float) -> float:
    """Converte mW para dBm. Retorna -200 se mw <= 0 (valor sentinela)."""
    if mw <= 0:
        return -200.0
    return 10.0 * math.log10(mw)


def db_para_linear(db: float) -> float:
    """Converte dB para razão linear."""
    return 10.0 ** (db / 10.0)


def linear_para_db(lin: float) -> float:
    """Converte razão linear para dB."""
    if lin <= 0:
        return -200.0
    return 10.0 * math.log10(lin)


# =============================================================================
# Utilitários
# =============================================================================

def media(lista: list) -> float:
    return sum(lista) / len(lista) if lista else 0.0


def limita(lista: list):
    if len(lista) > MAX_AMOSTRAS:
        del lista[:-MAX_AMOSTRAS]


def aguarda_arquivo(nome: str, timeout: int = 120):
    print(f"  Aguardando '{nome}' ...")
    inicio = time.time()
    while not os.path.exists(nome):
        if time.time() - inicio > timeout:
            raise TimeoutError(f"'{nome}' não encontrado após {timeout}s.")
        time.sleep(1)
    print(f"  '{nome}' encontrado.")


def inicializa_saida(nome: str, cabecalho: str):
    with open(nome, "w", encoding="utf-8") as f:
        f.write(cabecalho + "\n")


# =============================================================================
# Leitura com detecção de recriação de arquivo (tail no CSV)
# =============================================================================

def le_novas_linhas_csv(caminho: str, pos_atual: int):
    """
    Lê apenas linhas novas a partir de pos_atual no CSV.
    Se o arquivo for menor que pos_atual (N3 recriou), reseta para 0.
    Retorna (lista_de_strings, nova_posicao).
    """
    novas = []
    if not os.path.exists(caminho):
        return novas, pos_atual
    try:
        tamanho = os.path.getsize(caminho)
        if tamanho < pos_atual:
            print(f"  [AVISO] '{caminho}' foi recriado. Reiniciando leitura.")
            pos_atual = 0

        with open(caminho, "r", encoding="utf-8") as f:
            f.seek(pos_atual)
            for linha in f:
                s = linha.strip()
                if s:
                    novas.append(s)
            nova_pos = f.tell()

        return novas, nova_pos

    except Exception as e:
        print(f"  [ERRO] Leitura '{caminho}': {e}")
        return novas, pos_atual


# =============================================================================
# Parse do CSV do N3
#
# Cabeçalho:
#   timestamp,uplink_counter,ldr_value,intensity,rssi,snr,
#   gateway_id,MY_ID,bateria,temperatura,umidade,latitude,longitude
# Índices:
#   0=timestamp  1=uplink_counter  2=ldr_value  3=intensity
#   4=rssi       5=snr             6=gateway_id  7=MY_ID
#   8=bateria    9=temperatura     10=umidade
#   11=latitude  12=longitude
# =============================================================================

def parseia_csv(linhas: list) -> list:
    """
    Retorna lista de dicts com os campos necessários.
    Ignora cabeçalho e linhas malformadas.
    """
    resultado = []
    for linha in linhas:
        # Pula cabeçalho
        if linha.startswith("timestamp"):
            continue
        partes = linha.split(",")
        if len(partes) < 11:
            continue
        try:
            rssi_str = partes[4].strip()
            snr_str  = partes[5].strip()
            # Campos podem vir como "None" ou vazio se gateway não reportou
            rssi = float(rssi_str) if rssi_str not in ("", "None") else None
            snr  = float(snr_str)  if snr_str  not in ("", "None") else None

            resultado.append({
                "ldr_value"  : float(partes[2].strip()),
                "rssi"       : rssi,
                "snr"        : snr,
                "bateria"    : float(partes[8].strip()),
                "latitude"   : float(partes[9].strip()),
                "longitude"  : float(partes[10].strip()),
                "altitude"  : float(partes[11].strip()),
            })
        except (ValueError, IndexError):
            pass
    return resultado


# =============================================================================
# Gravação — UMA vez por ciclo
# =============================================================================

def salva_app():
    if not lum_lista:
        return
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    linha = (
        f"{ts};"
        f"{lum_lista[-1]:.1f};{media(lum_lista):.2f};"
        f"{lat_ultimo:.6f};{lon_ultimo:.6f};{alt_ultimo:.1f};"
        f"{len(lum_lista)}\n"
    )
    with open(ARQUIVO_APP_OUT, "a", encoding="utf-8") as f:
        f.write(linha)


def salva_ger():
    if not rssi_mw_lista:
        return
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Média em mW → reconverte para dBm
    rssi_media_dbm = mw_para_dbm(media(rssi_mw_lista))
    # Média linear → reconverte para dB
    snr_media_db   = linear_para_db(media(snr_lin_lista))

    linha = (
        f"{ts};"
        f"{rssi_dbm_ultimo:.2f};{rssi_media_dbm:.2f};"
        f"{snr_db_ultimo:.2f};{snr_media_db:.2f};"
        f"{bat_ultimo:.2f};"
        f"{len(rssi_mw_lista)}\n"
    )
    with open(ARQUIVO_GER_OUT, "a", encoding="utf-8") as f:
        f.write(linha)


# =============================================================================
# MAIN
# =============================================================================

print("=" * 65)
print("  N5 - Processamento de Dados IoT LoRa  (v3 — entrada: CSV TTN)")
print("=" * 65)
print(f"  Entrada CSV       : {ARQUIVO_CSV_IN}")
print(f"  Saida aplicacao   : {ARQUIVO_APP_OUT}")
print(f"  Saida gerencia    : {ARQUIVO_GER_OUT}")
print(f"  Intervalo : {INTERVALO_LEITURA}s | Max amostras: {MAX_AMOSTRAS}")
print(f"  Média RSSI/SNR    : domínio linear (mW / razão) → resultado em dBm/dB")
print("-" * 65)

aguarda_arquivo(ARQUIVO_CSV_IN)

inicializa_saida(
    ARQUIVO_APP_OUT,
    "timestamp;lum;lum_media;lat;lon;alt;qtd_amostras"
)
inicializa_saida(
    ARQUIVO_GER_OUT,
    "timestamp;rssi;rssi_media_dbm;snr;snr_media_db;bateria;qtd_amostras"
)

print(f"\nMonitorando '{ARQUIVO_CSV_IN}' a cada {INTERVALO_LEITURA}s ... Ctrl+C para encerrar.\n")

try:
    while True:

        linhas_novas, pos_csv = le_novas_linhas_csv(ARQUIVO_CSV_IN, pos_csv)
        registros = parseia_csv(linhas_novas)

        novos_app = 0
        novos_ger = 0

        for r in registros:
            # ----------------------------------------------------------------
            # Aplicação
            # ----------------------------------------------------------------
            lum_lista.append(r["ldr_value"])
            lat_ultimo = r["latitude"]
            lon_ultimo = r["longitude"]
            alt_ultimo = r["altitude"]
            novos_app += 1

            for lst in [lum_lista]:
                limita(lst)

            # ----------------------------------------------------------------
            # Gerência — RSSI e SNR: converte para linear antes de acumular
            # ----------------------------------------------------------------
            if r["rssi"] is not None and r["snr"] is not None:
                rssi_mw_lista.append(dbm_para_mw(r["rssi"]))
                snr_lin_lista.append(db_para_linear(r["snr"]))
                # Valores reais do último pacote (em dBm/dB para exibição)
                globals()["rssi_dbm_ultimo"] = r["rssi"]
                globals()["snr_db_ultimo"]   = r["snr"]
                bat_ultimo_local = r["bateria"]
                globals()["bat_ultimo"] = bat_ultimo_local
                novos_ger += 1

                for lst in [rssi_mw_lista, snr_lin_lista]:
                    limita(lst)

        # Grava UMA vez por ciclo, após processar todas as novas linhas
        if novos_app > 0:
            salva_app()
            ts = datetime.now().strftime("%H:%M:%S")
            print(
                f"[{ts}] APP +{novos_app:2d} | "
                f"Lum={lum_lista[-1]:.0f}(med={media(lum_lista):.1f}) | "
                f"Lat={lat_ultimo:.6f} Lon={lon_ultimo:.6f} | Alt={alt_ultimo:.0f} | "
                f"N={len(lum_lista)}"
            )

        if novos_ger > 0:
            rssi_media_dbm = mw_para_dbm(media(rssi_mw_lista))
            snr_media_db   = linear_para_db(media(snr_lin_lista))
            salva_ger()
            ts = datetime.now().strftime("%H:%M:%S")
            print(
                f"[{ts}] GER +{novos_ger:2d} | "
                f"RSSI={rssi_dbm_ultimo:.1f}dBm(med={rssi_media_dbm:.2f}dBm) | "
                f"SNR={snr_db_ultimo:.1f}dB(med={snr_media_db:.2f}dB) | "
                f"Bat={bat_ultimo:.2f} | "
                f"N={len(rssi_mw_lista)}"
            )

        time.sleep(INTERVALO_LEITURA)

except KeyboardInterrupt:
    print("\n\nCtrl+C — Encerrando N5...")

# ---------- Resumo final ----------
rssi_media_dbm = mw_para_dbm(media(rssi_mw_lista))
snr_media_db   = linear_para_db(media(snr_lin_lista))

print("\nResumo final:")
print(f"  Amostras aplicacao : {len(lum_lista)}")
print(f"  Amostras gerencia  : {len(rssi_mw_lista)}")
if lum_lista:
    print(f"  Media Luminosidade : {media(lum_lista):.2f}")
if rssi_mw_lista:
    print(f"  Media RSSI UL      : {rssi_media_dbm:.2f} dBm  (via média linear)")
    print(f"  Media SNR  UL      : {snr_media_db:.2f}  dB   (via média linear)")
print("N5 encerrado.")
