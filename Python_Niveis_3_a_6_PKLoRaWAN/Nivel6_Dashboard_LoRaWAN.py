# =============================================================================
# Framework TpM - N6 - DASHBOARD DE VISUALIZAÇÃO IoT LoRa  (v3 — TTN/CSV)
# =============================================================================
#
# Lê em tempo real os arquivos gerados pelo N5 (v3):
#   - medidas_aplicacao_media.txt
#       timestamp;lum;lum_media;umi;umi_media;temp;temp_media;lat;lon;qtd_amostras
#   - medidas_gerencia_media.txt
#       timestamp;rssi;rssi_media_dbm;snr;snr_media_db;bateria;qtd_amostras
#
# Nota: rssi_media e snr_media já chegam em dBm/dB (média calculada em mW/linear pelo N5).
#
# Dependências:
#   pip install matplotlib
#   (tkinter já vem com Python no Windows; no Linux: sudo apt install python3-tk)
#
# Arquivo de saída compartilhado (Toggle LED Amarelo):
#   cmd_led_amarelo.txt  → "0" desligado | "1" ligado
# =============================================================================

import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
import os
import webbrowser

# ---------- Configurações ----------
ARQUIVO_APP  = "medidas_aplicacao_media.txt"
ARQUIVO_GER  = "medidas_gerencia_media.txt"
# CMD_LED_FILE = "cmd_led_amarelo.txt" # Para LoRaWAN está comentado, sem recepção de downlinks

INTERVALO_ATUALIZA = 2000    # ms entre atualizações dos gráficos
MAX_PONTOS = 100             # máximo de pontos exibidos nos gráficos

COR_DADO      = "#1f77b4"
COR_MEDIA     = "#ff7f0e"
COR_BG        = "#1e1e2e"
COR_FG        = "#cdd6f4"
COR_GRADE     = "#313244"
COR_BOTAO_ON  = "#a6e3a1"
COR_BOTAO_OFF = "#f38ba8"

# ---------- Estado de leitura (tail) ----------
pos_app = 0
pos_ger = 0

# ---------- Dados acumulados ----------
dados_app = {
    "ts": [], "lum": [], "lum_med": [],
    "lat": 0.0, "lon": 0.0, "alt": 0.0, "qtd": 0
}
dados_ger = {
    "ts": [], "rssi": [], "rssi_med": [],
    "snr": [], "snr_med": [],
    "bat": 0.0, "qtd": 0
}

estado_led = [0]   # lista para ser mutável entre funções


# =============================================================================
# Leitura de arquivo (tail)
# =============================================================================

def le_novas_linhas(caminho: str, pos_ref: int):
    """Lê apenas linhas novas a partir de pos_ref. Retorna (linhas, nova_pos)."""
    novas = []
    nova_pos = pos_ref
    if not os.path.exists(caminho):
        return novas, nova_pos
    try:
        tamanho = os.path.getsize(caminho)
        if tamanho < pos_ref:
            nova_pos = 0   # arquivo foi recriado
        with open(caminho, "r", encoding="utf-8") as f:
            f.seek(nova_pos)
            for linha in f:
                novas.append(linha.strip())
            nova_pos = f.tell()
    except Exception:
        pass
    return novas, nova_pos


def _limita(lista: list):
    if len(lista) > MAX_PONTOS:
        del lista[:-MAX_PONTOS]


# =============================================================================
# Parse e acumulação
# =============================================================================

def atualiza_dados_app():
    global pos_app
    linhas, pos_app = le_novas_linhas(ARQUIVO_APP, pos_app)
    for linha in linhas:
        if not linha or linha.startswith("timestamp"):
            continue
        p = linha.split(";")
        if len(p) < 6:
            continue
        try:
            dados_app["ts"].append(p[0][11:19])          # HH:MM:SS
            dados_app["lum"].append(float(p[1]))
            dados_app["lum_med"].append(float(p[2]))
            dados_app["lat"] = float(p[3])
            dados_app["lon"] = float(p[4])
            dados_app["alt"] = float(p[5])
            dados_app["qtd"] = int(p[6])
            for chave in ["ts", "lum", "lum_med"]:
                _limita(dados_app[chave])
        except ValueError:
            pass


def atualiza_dados_ger():
    """
    Formato do N5 v3:
      timestamp;rssi;rssi_media_dbm;snr;snr_media_db;bateria;qtd_amostras
    rssi_media já vem em dBm (N5 fez a média em mW e reconverte).
    """
    global pos_ger
    linhas, pos_ger = le_novas_linhas(ARQUIVO_GER, pos_ger)
    for linha in linhas:
        if not linha or linha.startswith("timestamp"):
            continue
        p = linha.split(";")
        if len(p) < 7:
            continue
        try:
            dados_ger["ts"].append(p[0][11:19])
            dados_ger["rssi"].append(float(p[1]))       # RSSI último pacote (dBm)
            dados_ger["rssi_med"].append(float(p[2]))   # Média RSSI em dBm (via mW)
            dados_ger["snr"].append(float(p[3]))        # SNR último pacote (dB)
            dados_ger["snr_med"].append(float(p[4]))    # Média SNR em dB (via linear)
            dados_ger["bat"] = float(p[5])
            dados_ger["qtd"] = int(p[6])
            for chave in ["ts", "rssi", "rssi_med", "snr", "snr_med"]:
                _limita(dados_ger[chave])
        except ValueError:
            pass


# =============================================================================
# Helpers de gráfico
# =============================================================================

def estilo_axes(ax, titulo: str, ylabel: str):
    ax.set_facecolor(COR_BG)
    ax.set_title(titulo, color=COR_FG, fontsize=9, pad=4)
    ax.set_ylabel(ylabel, color=COR_FG, fontsize=8)
    ax.tick_params(colors=COR_FG, labelsize=7)
    ax.grid(True, color=COR_GRADE, linewidth=0.5)
    for spine in ax.spines.values():
        spine.set_edgecolor(COR_GRADE)


def plota_grafico(ax, xs, ys, ys_med, titulo, ylabel,
                  cor_dado=COR_DADO, cor_med=COR_MEDIA):
    ax.clear()
    estilo_axes(ax, titulo, ylabel)
    if xs:
        step = max(1, len(xs) // 8)
        ticks_idx = list(range(0, len(xs), step))
        ax.set_xticks(ticks_idx)
        ax.set_xticklabels([xs[i] for i in ticks_idx],
                           rotation=30, ha="right", fontsize=6)
        idx = list(range(len(xs)))
        ax.plot(idx, ys,     color=cor_dado, linewidth=1.2,
                label="Leitura", marker="o", markersize=2)
        ax.plot(idx, ys_med, color=cor_med,  linewidth=1.5,
                label="Média (lin.)", linestyle="--")
        ax.legend(fontsize=7, facecolor=COR_BG, labelcolor=COR_FG, loc="upper left")


# =============================================================================
# Classe principal da janela
# =============================================================================

class DashboardLoRa(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Dashboard IoT PKLoRaWAN MoT — TpM Framework UNICAMP (V3 TTN/CSV)")
        self.configure(bg=COR_BG)
        self.geometry("1280x800")
        self.resizable(True, True)

        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TNotebook",     background=COR_BG, borderwidth=0)
        style.configure("TNotebook.Tab", background="#313244", foreground=COR_FG,
                         padding=[12, 4], font=("Segoe UI", 9, "bold"))
        style.map("TNotebook.Tab",
                  background=[("selected", "#45475a")],
                  foreground=[("selected", "#cba6f7")])

        self._constroi_notebook()
        self._atualiza_loop()

    # ------------------------------------------------------------------
    def _constroi_notebook(self):
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=6, pady=6)

        self.aba_app = tk.Frame(nb, bg=COR_BG)
        self.aba_ger = tk.Frame(nb, bg=COR_BG)
        self.aba_map = tk.Frame(nb, bg=COR_BG)

        nb.add(self.aba_app, text="  📡  Aplicação  ")
        nb.add(self.aba_ger, text="  🛠  Gerência   ")
        nb.add(self.aba_map, text="  🗺  GPS / Mapa  ")

        self._constroi_aba_app()
        self._constroi_aba_ger()
        self._constroi_aba_map()

    # ------------------------------------------------------------------
    # ABA APLICAÇÃO
    # ------------------------------------------------------------------
    def _constroi_aba_app(self):
        frame = self.aba_app

        top = tk.Frame(frame, bg=COR_BG)
        top.pack(fill="x", padx=10, pady=(8, 2))

        tk.Label(top, text="End Device LoRaWAN - Luminosidade (LDR)",
                 bg=COR_BG, fg="#cba6f7",
                 font=("Segoe UI", 11, "bold")).pack(side="left")

        # Para LoRaWAN está comentado, sem recepção de downlinks
        """
        self.btn_led = tk.Button(
            top, text="💡 LED AMARELO: OFF",
            bg=COR_BOTAO_OFF, fg="#1e1e2e",
            font=("Segoe UI", 9, "bold"),
            relief="flat", padx=10, pady=4,
            command=self._toggle_led
        )
        self.btn_led.pack(side="right", padx=6)
        """

        gps_frame = tk.Frame(frame, bg="#313244", bd=0)
        gps_frame.pack(fill="x", padx=10, pady=(2, 6))

        tk.Label(gps_frame, text="  Latitude:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self.lbl_lat = tk.Label(gps_frame, text="---", bg="#313244", fg=COR_FG,
                                font=("Courier New", 9))
        self.lbl_lat.pack(side="left", padx=(0, 20))


        tk.Label(gps_frame, text="Longitude:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self.lbl_lon = tk.Label(gps_frame, text="---", bg="#313244", fg=COR_FG,
                                font=("Courier New", 9))
        self.lbl_lon.pack(side="left", padx=(0, 20))


        tk.Label(gps_frame, text="Altitude:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self.lbl_alt = tk.Label(gps_frame, text="---", bg="#313244", fg=COR_FG,
                                font=("Courier New", 9))
        self.lbl_alt.pack(side="left", padx=(0, 20))


        tk.Label(gps_frame, text="Amostras:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self.lbl_qtd_app = tk.Label(gps_frame, text="0", bg="#313244", fg=COR_FG,
                                    font=("Courier New", 9))
        self.lbl_qtd_app.pack(side="left", padx=(0, 20))


        self.fig_app = Figure(figsize=(12, 6), dpi=90, facecolor=COR_BG) 
        self.fig_app.subplots_adjust(hspace=0.45, wspace=0.3, left=0.06,
                                     right=0.98, top=0.93, bottom=0.12)
        self.ax_lum  = self.fig_app.add_subplot(1, 1, 1)

        for ax in [self.ax_lum]:
            estilo_axes(ax, "", "")

        self.canvas_app = FigureCanvasTkAgg(self.fig_app, master=frame)
        self.canvas_app.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=4)

    # ------------------------------------------------------------------
    # ABA GERÊNCIA
    # ------------------------------------------------------------------
    def _constroi_aba_ger(self):
        frame = self.aba_ger

        top = tk.Frame(frame, bg=COR_BG)
        top.pack(fill="x", padx=10, pady=(8, 2))

        tk.Label(top,
                 text="Gerência de Rede LoRaWAN — RSSI / SNR / Bateria End Device "
                      "[Média via domínio linear mW/razão]",
                 bg=COR_BG, fg="#cba6f7",
                 font=("Segoe UI", 11, "bold")).pack(side="left")

        bat_frame = tk.Frame(frame, bg="#313244")
        bat_frame.pack(fill="x", padx=10, pady=(2, 6))

        tk.Label(bat_frame, text="  Bateria do Nó Sensor:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self.lbl_bat = tk.Label(bat_frame, text="---", bg="#313244", fg="#a6e3a1",
                                font=("Courier New", 10, "bold"))
        self.lbl_bat.pack(side="left", padx=(0, 20))

        tk.Label(bat_frame, text="Amostras:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self.lbl_qtd_ger = tk.Label(bat_frame, text="0", bg="#313244", fg=COR_FG,
                                    font=("Courier New", 9))
        self.lbl_qtd_ger.pack(side="left")

        self.fig_ger = Figure(figsize=(12, 5.5), dpi=90, facecolor=COR_BG)
        self.fig_ger.subplots_adjust(hspace=0.4, wspace=0.35, left=0.08,
                                     right=0.97, top=0.92, bottom=0.14)
        self.ax_rssi = self.fig_ger.add_subplot(1, 2, 1)
        self.ax_snr  = self.fig_ger.add_subplot(1, 2, 2)

        for ax in [self.ax_rssi, self.ax_snr]:
            estilo_axes(ax, "", "")

        self.canvas_ger = FigureCanvasTkAgg(self.fig_ger, master=frame)
        self.canvas_ger.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=4)

    # ------------------------------------------------------------------
    # ABA GPS / MAPA
    # ------------------------------------------------------------------
    def _constroi_aba_map(self):
        frame = self.aba_map

        tk.Label(frame, text="📍  Localização do End Device LoRaWAN - GPS",
                 bg=COR_BG, fg="#cba6f7",
                 font=("Segoe UI", 12, "bold")).pack(pady=(20, 10))

        info = tk.Frame(frame, bg="#313244", pady=14)
        info.pack(fill="x", padx=40)

        tk.Label(info, text="Latitude:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 10, "bold")).grid(row=0, column=0, padx=16, sticky="w")
        self.lbl_lat2 = tk.Label(info, text="---", bg="#313244", fg=COR_FG,
                                 font=("Courier New", 11))
        self.lbl_lat2.grid(row=0, column=1, padx=8, sticky="w")


        tk.Label(info, text="Longitude:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 10, "bold")).grid(row=0, column=2, padx=16, sticky="w")
        self.lbl_lon2 = tk.Label(info, text="---", bg="#313244", fg=COR_FG,
                                 font=("Courier New", 11))
        self.lbl_lon2.grid(row=0, column=3, padx=8, sticky="w")


        tk.Label(info, text="Altitude:", bg="#313244", fg="#89dceb",
                 font=("Segoe UI", 10, "bold")).grid(row=0, column=4, padx=16, sticky="w")
        self.lbl_alt2 = tk.Label(info, text="---", bg="#313244", fg=COR_FG,
                                 font=("Courier New", 11))
        self.lbl_alt2.grid(row=0, column=5, padx=8, sticky="w")


        self.lbl_endereco = tk.Label(
            frame, text="Endereço: aguardando dados GPS...",
            bg=COR_BG, fg="#fab387", font=("Segoe UI", 10), wraplength=800
        )
        self.lbl_endereco.pack(pady=8)

        self.btn_mapa = tk.Button(
            frame, text="🗺  Abrir no Google Maps",
            bg="#74c7ec", fg="#1e1e2e",
            font=("Segoe UI", 10, "bold"),
            relief="flat", padx=16, pady=8,
            command=self._abrir_mapa
        )
        self.btn_mapa.pack(pady=6)

        self.btn_osm = tk.Button(
            frame, text="🌐  Abrir no OpenStreetMap",
            bg="#a6e3a1", fg="#1e1e2e",
            font=("Segoe UI", 10, "bold"),
            relief="flat", padx=16, pady=8,
            command=self._abrir_osm
        )
        self.btn_osm.pack(pady=4)

        self.lbl_dica_mapa = tk.Label(
            frame,
            text="💡 Dica: clique nos botões acima para ver a localização no mapa online.",
            bg=COR_BG, fg="#6c7086", font=("Segoe UI", 9, "italic")
        )
        self.lbl_dica_mapa.pack(pady=(20, 4))

        tk.Label(frame, text="Histórico de posições (últimas 10):",
                 bg=COR_BG, fg="#a6adc8",
                 font=("Segoe UI", 9, "bold")).pack(pady=(16, 2))

        self.txt_hist = tk.Text(
            frame, height=8, width=50,
            bg="#313244", fg=COR_FG,
            font=("Courier New", 9),
            relief="flat", state="disabled"
        )
        self.txt_hist.pack()
        self._historico_gps = []


    # Para LoRaWAN está comentado, sem recepção de downlinks
    """
    # ------------------------------------------------------------------
    # Toggle LED Amarelo
    # ------------------------------------------------------------------
    def _toggle_led(self):
        estado_led[0] = 1 - estado_led[0]
        try:
            with open(CMD_LED_FILE, "w") as f:
                f.write(str(estado_led[0]))
        except Exception as e:
            print(f"[ERRO] Não foi possível escrever {CMD_LED_FILE}: {e}")
        self._atualiza_botao_led()

    def _atualiza_botao_led(self):
        if estado_led[0] == 1:
            self.btn_led.config(text="💡 LED AMARELO: ON",  bg=COR_BOTAO_ON)
        else:
            self.btn_led.config(text="💡 LED AMARELO: OFF", bg=COR_BOTAO_OFF)

    """
    
    # ------------------------------------------------------------------
    # Abrir mapa no navegador
    # ------------------------------------------------------------------
    def _abrir_mapa(self):
        lat, lon = dados_app["lat"], dados_app["lon"]
        if lat == 0.0 and lon == 0.0:
            return
        webbrowser.open(f"https://www.google.com/maps?q={lat},{lon}")

    def _abrir_osm(self):
        lat, lon = dados_app["lat"], dados_app["lon"]
        if lat == 0.0 and lon == 0.0:
            return
        webbrowser.open(
            f"https://www.openstreetmap.org/?mlat={lat}&mlon={lon}#map=16/{lat}/{lon}"
        )

    # ------------------------------------------------------------------
    # Loop de atualização
    # ------------------------------------------------------------------
    def _atualiza_loop(self):
        threading.Thread(target=self._tarefa_leitura, daemon=True).start()
        self.after(INTERVALO_ATUALIZA, self._atualiza_graficos)

    def _tarefa_leitura(self):
        """Thread em background: lê arquivos continuamente."""
        while True:
            atualiza_dados_app()
            atualiza_dados_ger()
            time.sleep(INTERVALO_ATUALIZA / 1000.0)

    def _atualiza_graficos(self):
        """Chamado pelo loop principal do Tk para redesenhar."""
        # --- Aplicação ---
        d = dados_app
        if d["ts"]:
            plota_grafico(self.ax_lum,  d["ts"], d["lum"],  d["lum_med"],
                          "Luminosidade (LDR)", "Valor ADC")
            self.canvas_app.draw_idle()

            lat_str = f"{d['lat']:.6f}"
            lon_str = f"{d['lon']:.6f}"
            alt_str = f"{d['alt']:.0f}"

            self.lbl_lat.config(text=lat_str)
            self.lbl_lon.config(text=lon_str)
            self.lbl_alt.config(text=alt_str)

            self.lbl_lat2.config(text=lat_str)
            self.lbl_lon2.config(text=lon_str)
            self.lbl_alt2.config(text=alt_str)
            
            self.lbl_qtd_app.config(text=str(d["qtd"]))

            # Histórico GPS
            if d["lat"] != 0.0 or d["lon"] != 0.0:
                entrada = (
                    f"{d['ts'][-1] if d['ts'] else '?'}  "
                    f"{d['lat']:.6f}, {d['lon']:.6f}, {d['alt']:.0f}"
                )
                if not self._historico_gps or self._historico_gps[-1] != entrada:
                    self._historico_gps.append(entrada)
                    self._historico_gps = self._historico_gps[-10:]
                    self.txt_hist.config(state="normal")
                    self.txt_hist.delete("1.0", "end")
                    self.txt_hist.insert("end", "\n".join(self._historico_gps))
                    self.txt_hist.config(state="disabled")

        # --- Gerência ---
        g = dados_ger
        if g["ts"]:
            plota_grafico(
                self.ax_rssi, g["ts"], g["rssi"], g["rssi_med"],
                "RSSI Uplink (dBm)", "dBm",
                "#74c7ec", "#f38ba8"
            )
            plota_grafico(
                self.ax_snr,  g["ts"], g["snr"],  g["snr_med"],
                "SNR Uplink (dB) — Média linear", "dB",
                "#a6e3a1", "#fab387"
            )
            self.canvas_ger.draw_idle()

            self.lbl_bat.config(text=f"{g['bat']:.2f}  (Volts DC)")
            self.lbl_qtd_ger.config(text=str(g["qtd"]))

        # Para LoRaWAN está comentado, sem recepção de downlinks
        """
        # Sincroniza estado do LED com o arquivo
        try:
            with open(CMD_LED_FILE, "r") as f:
                v = f.read().strip()
                estado_led[0] = int(v) if v in ("0", "1") else 0
            self._atualiza_botao_led()
        except Exception:
            pass

        """
        
        self.after(INTERVALO_ATUALIZA, self._atualiza_graficos)


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("  N6 - Dashboard IoT LoRa  (v3 — entrada: arquivos N5 via CSV TTN)")
    print("  Iniciando interface gráfica...")
    print("  Aguardando dados do N5...")
    print("=" * 60)
    app = DashboardLoRa()
    app.mainloop()
    print("N6 encerrado.")
