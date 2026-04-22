/*
  End Device Tipo A - LoRaWAN — ABP (Activation By Personalization)
  Hardware: ESP32 Dev Module 30 pinos + RFM95
  Frequência: 916.8 MHz (AU915 canal 8, fixo)
  Modo: ABP, Classe A (Somente Uplinks, sem confirmação)

  Bibliotecas requeridas:
    - MCCI LoRaWAN LMIC library (arduino-lmic by mcci-catena)

  Chaves ABP fornecidos pelo Servidor de Rede - Network Server (TTN, Chirpstack, etc.)
  Sem procedimento de join — End device transmite imediatamente após o boot.
  Firmware desenvolvido para Gateways Monocanal LoRaWAN

  NVS Wear Leveling:
    O frame counter é salvo em 100 slots rotativos no NVS.
    A cada salvamento, usa-se o próximo slot no ciclo circular.
    Na leitura, percorre todos os slots e retorna o maior valor.
    Isso distribui as escritas e multiplica a vida útil da flash por ~100x.
    Com flash típica de 10.000 ciclos/célula e FCNT_SAVE_EVERY=10:
      - Sem wear leveling: ~100.000 uplinks por vida útil
      - Com wear leveling: ~10.000.000 uplinks por vida útil
*/

// Bibliotecas requeridas
#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Preferences.h>    // ESP32 NVS — Mantém o contador de Uplinks mesmo após reinicializações.
                            // Necessário para Servidor TTN não identificar o End device como impostor/ghost

// -----------------------------------------------------------------
//  MAPEAMENTO PINAGEM - RFM95 - ESP32 Dev Module Kit Vs.1
// -----------------------------------------------------------------
#define SCK_PIN   5
#define MISO_PIN  19
#define MOSI_PIN  27
#define RST_PIN   14
#define NSS_PIN   18
#define DIO0_PIN  26
#define DIO1_PIN  35
#define DIO2_PIN  34

// Sensor LDR
#define LDR_PIN   36   // ADC1_CH0 — sensor LDR

// Medido da Bateria
#define BAT_PIN   32  // ADC1_CH4 - Bateria End Device (APP)  - NÃO IMPLEMENTADO NO HARDWARE PKLORA

// Pinos de Entrada Digitais
#define BOTAO_PIN   39   // Pino do Botão - PIN VN

// Pinos de Saída Digitais
#define LED_VERMELHO_PIN  4
#define LED_AMARELO_PIN  2
#define LED_VERDE_PIN  15

// --- 3. Variáveis Globais e de Configuração ---
const int MY_ID = 3; // Identificação (ID de rede) deste Nó Sensor
const int GATEWAY_ID = 0; // Identificação (ID de rede) do Destino Gateway deste Pacote

// Tipos dos Sensores
const int TIPO_SENSOR_LDR = 44; //SENSOR LDR
const int TIPO_SENSOR_BAT = 11; // SENSOR BATERIA - NÃO IMPLEMENTADO NO HARDWARE PKLORA
const int TIPO_SENSOR_GPS = 10; // TIPO DO SENSOR GPS - VIA SOFTWARE


// -----------------------------------------------------------------
//  CREDENCIAIS LoRaWAN - tipo ABP
//  Gere na TTN - Network Server - durante o registro do End device.
//
//  IMPORTANTE - ordem dos bytes para Biblioteca LMIC:
//    DevAddr  — 4 bytes, MSB primeiro (conforme mostrado no registro)
//    NwkSKey  — 16 bytes, MSB primeiro
//    AppSKey  — 16 bytes, MSB primeiro
// -----------------------------------------------------------------

// Endereço do End Device - Device Address — 4 bytes
static const u4_t DEVADDR = 0x260DACBD;
// ECE334FFFF48ABB0 => DevEUI
// Chave de Network Session Key — 16 bytes, MSB primeiro
static const u1_t PROGMEM NWKSKEY[16] = {
    0x50,0x82,0x01,0xA5,0x99,0x72,0x64,0xA3,0xD3,0xA3,0x9D,0xC1,0xAF,0x11,0xF5,0x9D
    // 508201A5997264A3D3A39DC1AF11F59D // Obtido na TTN durante o registro
};

// Cheave de Application Session Key — 16 bytes, MSB primeiro
static const u1_t PROGMEM APPSKEY[16] = {
    0x3A,0x98,0x0F,0xE6,0x83,0x26,0xB7,0x58,0x84,0x61,0x2C,0xDE,0x6D,0x6E,0xE2,0x1C
    // 3A980FE68326B75884612CDE6D6EE21C // Obtido na TTN durante o registro
};

// -----------------------------------------------------------------
//  LMIC - Mapeamento da pinagem para Biblioteca LMIC
// -----------------------------------------------------------------
const lmic_pinmap lmic_pins = {
    .nss  = NSS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = RST_PIN,
    .dio  = { DIO0_PIN, DIO1_PIN, DIO2_PIN },
};

// -----------------------------------------------------------------
//  INTERVALOS de tempo entre UPLINK
// -----------------------------------------------------------------
#define TX_INTERVAL_MS  120000UL   // 120 seconds // 2 minutos Padrão TTN Free

// -----------------------------------------------------------------
//  TAMANHO DO PAYLOAD (BYTES ÚTEIS)
#define TAMANHO_PAYLOAD 36   // MoT => 36 bytes
#define NUM_LEITURA 8 // Média de 8 Leituras LDR - evitar oscilações

// --- Variáveis de Medição ---
float rssi_dl_real_dbm;
// Armazena o RSSI de Uplink medido (ex: -80.5)
byte rssi_dl_convertido;
// Armazena o RSSI convertido para 1 byte (ex: 170)

float snr_dl_real_dbm;
// Armazena o SNR de Uplink medido (ex: 9.5)
byte snr_dl_convertido;
// Armazena o SNR convertido para 1 byte, inteiro com shift de 10 (ex: ((-9.5*10)+120) = 95+120 = 215)

// Contador UL
uint PKTu = 0;

// -----------------------------------------------------------------
//  CONTADOR DE UPLINKS PERSISTENTE — NVS com Wear Leveling
//
//  Estratégia: NVS_SLOT_COUNT slots rotativos no NVS.
//  A cada salvamento, usa o próximo slot circular.
//  Na leitura, percorre todos os slots e retorna o maior valor.
//  Isso distribui as escritas e multiplica a vida útil da flash
//  por NVS_SLOT_COUNT vezes.
//
//  Layout NVS (namespace "lorawan"):
//    "fcnt_00" … "fcnt_99"  → valores do frame counter (uint32)
//    "fcnt_idx"             → índice do último slot gravado (uint8)
//
//  End Device ABP DEVE salvar o frame counter persistentemente.
//  Se o contador for reiniciado abaixo do último valor visto pelo
//  servidor, o servidor irá descartar os pacotes silenciosamente
//  (proteção contra ataques de repetição).
// -----------------------------------------------------------------
Preferences prefs;
#define NVS_NAMESPACE     "lorawan"
#define NVS_SLOT_COUNT    100          // Número de slots rotativos
#define NVS_IDX_KEY       "fcnt_idx"   // Chave do índice atual
#define FCNT_SAVE_EVERY   10           // Salva a cada N uplinks (reduz desgaste)

static uint32_t fcntUp        = 0;
static uint32_t fcntSinceSave = 0;
static uint8_t  fcntSlotIdx   = 0;    // Slot atual na rotação (0 … NVS_SLOT_COUNT-1)

// Gera a chave NVS de um slot: "fcnt_00" … "fcnt_99"
static void slotKey(uint8_t idx, char* buf) {
    snprintf(buf, 12, "fcnt_%02u", (unsigned)(idx % NVS_SLOT_COUNT));
}

/*
 * loadFrameCounter()
 * Percorre todos os NVS_SLOT_COUNT slots e retorna o maior fcntUp
 * encontrado — valor mais recente mesmo após crash ou reset.
 * Também recupera fcntSlotIdx para retomar a rotação corretamente.
 */
void loadFrameCounter() {
    prefs.begin(NVS_NAMESPACE, false);

    uint32_t maxVal  = 0;
    uint8_t  maxSlot = 0;
    char     key[12];

    for (uint8_t i = 0; i < NVS_SLOT_COUNT; i++) {
        slotKey(i, key);
        uint32_t val = prefs.getUInt(key, 0);
        if (val > maxVal) {
            maxVal  = val;
            maxSlot = i;
        }
    }

    fcntUp = maxVal;

    // Tenta usar o índice persistido para desempate e continuidade
    uint8_t savedIdx = prefs.getUChar(NVS_IDX_KEY, (maxSlot + 1) % NVS_SLOT_COUNT);
    char    savedKey[12];
    slotKey(savedIdx, savedKey);

    // Usa savedIdx somente se o slot apontado contém o valor máximo
    if (prefs.getUInt(savedKey, 0) >= maxVal) {
        fcntSlotIdx = (savedIdx + 1) % NVS_SLOT_COUNT;
    } else {
        // Retoma a partir do slot seguinte ao que continha o máximo
        fcntSlotIdx = (maxSlot + 1) % NVS_SLOT_COUNT;
    }

    prefs.end();

    Serial.printf("[NVS] FCntUp restaurado: %u  |  Próximo slot: %u/%u\n",
                  fcntUp, fcntSlotIdx, NVS_SLOT_COUNT);
}

/*
 * saveFrameCounter(fcnt)
 * Grava fcnt no slot atual da rotação, persiste o índice e avança
 * para o próximo slot. Cada chamada usa uma célula de flash diferente.
 *
 * ATENÇÃO: chame sempre com (LMIC.seqnoUp + FCNT_SAVE_EVERY) para
 * criar uma margem de segurança em caso de crash antes do próximo save.
 */
void saveFrameCounter(uint32_t fcnt) {
    char key[12];
    slotKey(fcntSlotIdx, key);

    prefs.begin(NVS_NAMESPACE, false);
    prefs.putUInt(key,      fcnt);
    prefs.putUChar(NVS_IDX_KEY, fcntSlotIdx);
    prefs.end();

    Serial.printf("[NVS] FCnt %u → slot %u (%s)  |  Próximo: %u\n",
                  fcnt, fcntSlotIdx, key,
                  (fcntSlotIdx + 1) % NVS_SLOT_COUNT);

    fcntSlotIdx = (fcntSlotIdx + 1) % NVS_SLOT_COUNT;
}

// -----------------------------------------------------------------
//  OTAA stubs — requeridos pela LMIC mesmo quando usado ABP
// -----------------------------------------------------------------
void os_getArtEui(u1_t* buf) { memset(buf, 0, 8); }
void os_getDevEui(u1_t* buf) { memset(buf, 0, 8); }
void os_getDevKey(u1_t* buf) { memset(buf, 0, 16); }

// -----------------------------------------------------------------
//  GLOBAL
// -----------------------------------------------------------------
static osjob_t sendjob;

// -----------------------------------------------------------------
//  LÊ SENSOR — LDR (ADC 12-bits, Média de 8 amostras)
// -----------------------------------------------------------------
uint16_t readLDR() {
    uint32_t soma = 0;
    for (int i = 0; i < 8; i++) {
        soma += analogRead(LDR_PIN);
        delay(2);
    }
    return (uint16_t)(soma / 8);
}

// -----------------------------------------------------------------
//  Monta o PAYLOAD e Envia
// -----------------------------------------------------------------
void do_send(osjob_t* j) {
//    float temperature, humidity;
    
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("[TX] Pendente — pulando este ciclo"));
        return;
    }

    digitalWrite(LED_VERMELHO_PIN, HIGH); // Turn the LED off
    // Lê o sensor LDR
    uint16_t ldrValue = readLDR();

    float    voltage  = (ldrValue / 4095.0f) * 3.3f;
    Serial.printf("[SENSOR] LDR raw: %u  |  Voltage: %.3fV  |  FCnt: %u\n",
                  ldrValue, voltage, LMIC.seqnoUp);
    
    // Nivel de Bateria do Módulo Nó Sensor
    uint16_t bateria = analogRead(BAT_PIN); // PIN D32
    // voltage_bat  = (bateria / 4095.0f) * 3.3f;
    float    voltage_bat  = (bateria * 3.3f ) / 4095.0f;
    //uint16_t voltBatInt = (uint16_t)(voltage_bat * 100.0f);
    uint16_t voltBatInt = (uint16_t)(voltage_bat * 100);
    Serial.print(F("voltBatInt="));
    Serial.println(voltBatInt);

    uint8_t botao = 0;
    uint8_t estadoBotao = digitalRead(BOTAO_PIN);
    if (estadoBotao == HIGH) {
        // Botão pressionado
        botao = 1;
    }
    else{
        botao = 0;
    }

    
    // payload
    uint8_t payload[TAMANHO_PAYLOAD];

    // Incrementa Contador Pacote UL
    PKTu = PKTu + 1;

    // BYTES CAMADA PHY
    payload[0] = 0;     // RSSId; // RSSId RX Rádio LoRa
    payload[1] = 0;     // SNRd; // SNRu RX Rádio LoRa
    payload[2] = 0;     // RSSIu TX Rádio LoRa - Será mensurado pelo Transceptor LoRa Gateway
    payload[3] = 0;     // SNRu TX Rádio LoRa - Será mensurado pelo Transceptor LoRa Gateway

    // BYTES CAMADA MAC
    payload[4] = 0;   // POT_LORA; // Potência TX Rádio LoRa
    payload[5] = 0;   // SF_LORA;  // Spreading Factor Rádio LoRa
    payload[6] = 0;   // BW_LORA;  // Bandwidth Rádio LoRa
    payload[7] = 0;   // CR_LORA;  // Cording Rate Rádio LoRa    

    // BYTES CAMADA DE REDE - NET
    
    // Byte Destino
    payload[8] = GATEWAY_ID;
    payload[9] = 0;
    
    // Byte Origem
    payload[10] = MY_ID;
    payload[11] = 0;

    // BYTES CAMADA TRANSPORTE
    payload[12] = 0;
    payload[13] = 0;
    
    // Contador Pacote UL
    payload[14] = (PKTu >> 8) & 0xFF;    
    payload[15] =  PKTu       & 0xFF;

    // BYTES CAMADA DE APLICAÇÂO
    
    // LDR - Luminosidade
    payload[16] = TIPO_SENSOR_LDR;
    payload[17] = (ldrValue >> 8) & 0xFF;
    payload[18] =  ldrValue       & 0xFF;


    // Tensão da Bateria
    payload[19] = TIPO_SENSOR_BAT;
    payload[20] = (voltBatInt >> 8) & 0xFF;
    payload[21] =  voltBatInt       & 0xFF;

    // Estado do Botão do End Device
    payload[22] = botao;

    // Posição GPS do End Device PKLoRa - Buscar no Maps manualmente
    // Latitude
    float raw_lat = -22.999782; 
    int32_t lat = (int32_t)(raw_lat * 1000000);
    payload[23] = TIPO_SENSOR_GPS;
    payload[24]  = (lat >> 24) & 0xFF;
    payload[25]  = (lat >> 16) & 0xFF;
    payload[26] = (lat >> 8)  & 0xFF;
    payload[27] =  lat        & 0xFF;

    // Longitude
    float raw_lon = -46.819428; 
    int32_t lon = (int32_t)(raw_lon * 1000000);
    payload[28] = (lon >> 24) & 0xFF;
    payload[29] = (lon >> 16) & 0xFF;
    payload[30] = (lon >> 8)  & 0xFF;
    payload[31] =  lon        & 0xFF;

    // Altitude (em metros)
    int16_t alt = 785;// (int16_t)(raw_alt * 100); // Result: 45075
    payload[32] = (alt >> 8) & 0xFF;
    payload[33] = alt & 0xFF;
    payload[34] = 0;     // Reservado
    payload[35] = 0;     // Reservado

    // Uplink não confirmado na porta 1
    LMIC_setTxData2(1, payload, sizeof(payload), 0);

    Serial.println(F("[TX] Pacote Enviado"));

    digitalWrite(LED_VERMELHO_PIN, LOW); // Turn the LED OFF

}

// -----------------------------------------------------------------
//  LMIC - CALLBACK DE EVENTOS
// -----------------------------------------------------------------
void onEvent(ev_t ev) {
    switch (ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("[LMIC] EV_TXCOMPLETE"));

            // Atualiza fcntUp com o valor atual do LMIC
            fcntUp = LMIC.seqnoUp;
            fcntSinceSave++;

            // Persiste a cada FCNT_SAVE_EVERY uplinks
            // Salva com margem (+FCNT_SAVE_EVERY) para cobrir possível crash
            // antes do próximo ciclo de salvamento
            if (fcntSinceSave >= FCNT_SAVE_EVERY) {
                saveFrameCounter(fcntUp + FCNT_SAVE_EVERY);
                fcntSinceSave = 0;
            }

            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("  ACK recebido"));
            if (LMIC.dataLen)
                Serial.printf("  Downlink %d byte(s) na porta %d\n",
                              LMIC.dataLen, LMIC.frame[LMIC.dataBeg - 1]);

            // Agenda próximo uplink
            os_setTimedCallback(&sendjob,
                os_getTime() + ms2osticks(TX_INTERVAL_MS), do_send);
            break;

        case EV_TXSTART:
            Serial.println(F("[LMIC] EV_TXSTART"));
            break;

        case EV_RXSTART:
            // Dispara frequentemente — suprimido para manter serial limpo
            break;

        case EV_JOIN_FAILED:
        case EV_REJOIN_FAILED:
            // Não deve ocorrer em ABP — ignorado
            break;

        case EV_RESET:
            Serial.println(F("[LMIC] EV_RESET"));
            break;

        case EV_LINK_DEAD:
            Serial.println(F("[LMIC] EV_LINK_DEAD — verifique FCnt no servidor"));
            break;

        default:
            Serial.printf("[LMIC] Evento: %u\n", (unsigned)ev);
            break;
    }
}

// -----------------------------------------------------------------
//  CONFIGURAÇÃO DO CANAL — 916.8 MHz somente
//  AU915 canal 8 conforme Gateway Monocanal
// -----------------------------------------------------------------
void configureChannel() {
    // Desabilita todos os 72 canais AU915
    for (int ch = 0; ch < 72; ch++) {
        LMIC_disableChannel(ch);
    }
    // Reabilita somente canal 8 = 916.8 MHz
    LMIC_enableChannel(8);

    // SF7 / 125 kHz, 14 dBm — ajustar SF se necessário
    LMIC_setDrTxpow(DR_SF7, 14);

    // Desabilita ADR — DR gerenciado manualmente
    LMIC_setAdrMode(0);

    // Desabilita link check (requer downlinks para funcionar)
    LMIC_setLinkCheckMode(0);
}

// -----------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);

    while (!Serial && millis() < 3000);
    Serial.println(F("\nEnd Device LoRaWAN ABP — Wear Leveling NVS"));
    Serial.printf("    Slots NVS: %u  |  Salva a cada: %u uplinks\n",
                  NVS_SLOT_COUNT, FCNT_SAVE_EVERY);
    Serial.printf("    Vida útil estimada: ~%lu uplinks\n\n",
                  (unsigned long)NVS_SLOT_COUNT * 10000UL * FCNT_SAVE_EVERY);
   
    // Inicializa o Botao como Entrada Digital do ESP32
    pinMode(BOTAO_PIN, INPUT); 

    // Inicializa os LEDs como Saídas Digitais do ESP32
    pinMode(LED_VERMELHO_PIN, OUTPUT);
    pinMode(LED_AMARELO_PIN, OUTPUT);
    pinMode(LED_VERDE_PIN, OUTPUT);    
       
    //  --- Atua Led vermelho  --- 
    digitalWrite(LED_VERMELHO_PIN, HIGH); // LIGA LED VERMELHO - INDIFERENTE PARA O BOOT

    //  --- Atua Led amarelo  --- 
    digitalWrite(LED_AMARELO_PIN, HIGH); // LIGA O LED AMARELO - DEVE SER HIGH DURANTE BOOT

    //  --- Atua Led verde  --- 
    digitalWrite(LED_VERDE_PIN, LOW);  // DESLIGA O LED VERDE - DEVE SER LOW DURANTE BOOT

    // Pinagem SPI para RFM95
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN);
    
    delay(200);
    // Configuração ADC para o LDR
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(LDR_PIN, INPUT);

    // Carrega frame counter persistente (wear leveling)
    loadFrameCounter();

    // Inicializa LMIC
    os_init();
    LMIC_reset();

    // Aplica configuração de canal único
    configureChannel();

    // Configura sessão ABP
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);

    // Restaura o frame counter do NVS no LMIC
    LMIC.seqnoUp = fcntUp;
    Serial.printf("[ABP] Sessão configurada. DevAddr: 0x%08X  |  FCntUp: %u\n",
                  DEVADDR, LMIC.seqnoUp);

    // Salva imediatamente com margem de segurança
    saveFrameCounter(LMIC.seqnoUp + FCNT_SAVE_EVERY);

    Serial.println(F("[ABP] Iniciando envio de uplinks imediatamente...\n"));

    // Limpa o Display
//    display.clearDisplay();
    delay(200);
    //  --- Atua Led vermelho  --- 
    digitalWrite(LED_VERMELHO_PIN, LOW); // DESLIGA LED VERMELHO

    //  --- Atua Led amarelo  --- 
    digitalWrite(LED_AMARELO_PIN, LOW); // DESLIGA O LED AMARELO

    //  --- Atua Led verde  --- 
    digitalWrite(LED_VERDE_PIN, LOW);  // DESLIGA O LED VERDE

    // Primeiro uplink imediato
    os_setCallback(&sendjob, do_send);

}

// -----------------------------------------------------------------
//  LOOP — delega ao scheduler do LMIC
// -----------------------------------------------------------------
void loop() {
    os_runloop_once();
}
