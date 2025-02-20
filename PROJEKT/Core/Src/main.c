/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h> // do sprawdzania cyfr
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//do czujnika
volatile uint32_t echo_start = 0;
volatile uint32_t echo_end   = 0;
volatile uint8_t  echo_captured = 0; // 0 - czekam na rising, 1 - mam rising, czekam na falling, 2 - pomiar gotowy


typedef enum {
    WAITING_FOR_FLAG = 0,
    READING_FRAME,
    ESCAPING_BYTE
} HDLC_RX_State;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HDLC_FLAG        0x7E
#define HDLC_ESCAPE      0x7D
#define HDLC_ESCAPE_7E   0x5E
#define HDLC_ESCAPE_7D   0x5D


#define HDLC_MIN_FRAME_SIZE 6 // min długość ramki (bez znaków początku i końca oraz puste dane)
#define HDLC_MAX_FRAME_SIZE 3506 //max długość całlkowitej ramki
#define MAX_DATA_LEN 3500 // Max długość danych w ramce

#define BUFFER_SIZE 500 // rozmiar do mojego bufora na dane z czujnika




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//do czujnika
#define TRIG_Pin GPIO_PIN_7
#define TRIG_GPIO_Port GPIOA

#define SERVO_MIN_TICKS 3278   // 1ms
#define SERVO_MAX_TICKS 6556   // 2ms


//USART

#define USART_TXBUF_LEN 7014
#define USART_RXBUF_LEN 1428
uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];

#define MAX_PWM_VALUES 25  // Maksymalna liczba wartości PWM

__IO int USART_TX_Empty=0;
__IO int USART_TX_Busy=0;
__IO int USART_RX_Empty=0;
__IO int USART_RX_Busy=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t USART_kbhit(void);
int16_t USART_getchar(void);
uint8_t USART_getline(char *buf);
void    USART_fsend(char* format,...);

uint8_t  computeCRC8(const uint8_t *data, uint16_t length);
void     escapeByte(uint8_t byte, uint8_t *outBuf, uint16_t *outLen, uint16_t outMax);
void HDLC_SendFrame(uint8_t addrSrc, uint8_t addrDst, const uint8_t* data, uint16_t dataLen);
void HDLC_ParseFrame(const uint8_t* frame, uint16_t length);
int8_t HDLC_ProcessInput(void);
static HDLC_RX_State  hdlcRxState = WAITING_FOR_FLAG;
static uint8_t        hdlcInBuf[HDLC_MAX_FRAME_SIZE];
static uint16_t       hdlcInPos = 0;

int32_t pwm_val = 0;


//zmienne globalne dla tych tamtych co mają to

uint8_t addrSrc = 0;
uint8_t addrDst = 0;



typedef struct {
    uint16_t pwm_values[MAX_PWM_VALUES];
    uint16_t current_index;
    uint16_t total_values;
    uint8_t is_running;
    UART_HandleTypeDef* huart;
    TIM_HandleTypeDef* htim;
} PWMHandlerTypeDef;

PWMHandlerTypeDef pwm_handler = {0};

void PWM_DMA_Init(UART_HandleTypeDef* huart, TIM_HandleTypeDef* htim) {
    pwm_handler.huart = huart;
    pwm_handler.htim = htim;
    pwm_handler.current_index = 0;
    pwm_handler.total_values = 0;
    pwm_handler.is_running = 0;


    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    htim->hdma[TIM_DMA_ID_CC1]->Init.Mode = DMA_CIRCULAR;
    HAL_DMA_Init(htim->hdma[TIM_DMA_ID_CC1]);
}

HAL_StatusTypeDef PWM_DMA_AddValue(uint16_t angle) {
    if (angle > 180 || pwm_handler.total_values >= MAX_PWM_VALUES) {
        return HAL_ERROR;
    }

    // zamiana kąta na ticki
    uint32_t pulse_ticks = SERVO_MIN_TICKS +
        ((uint32_t)angle * (SERVO_MAX_TICKS - SERVO_MIN_TICKS) / 180);

    // sprawdzenie czy jest w min max rrange
    if (pulse_ticks < SERVO_MIN_TICKS || pulse_ticks > SERVO_MAX_TICKS) {
        return HAL_ERROR;
    }
    //jest to zabezpieczenie aby uniknąć konfliktu jak DMA chodzi i chcemy dodać wartość
    //stopuje przed dodaniem wartości. Dzięki temu DMA nie będzie jednocześnie odczytywać lub modyfikować bufora, gdy jest dodawana nowa wartość
    if (pwm_handler.is_running = 1){
    	HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
    	pwm_handler.pwm_values[pwm_handler.total_values++] = (uint16_t)pulse_ticks;
        HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    }
    else{pwm_handler.pwm_values[pwm_handler.total_values++] = (uint16_t)pulse_ticks;}
    return HAL_OK;
}

void PWM_DMA_Start(void) {
    if (pwm_handler.total_values == 0 || pwm_handler.is_running) {
        return;
    }

    // Resetuje index i ustawia flagę że chodzi DMA
    pwm_handler.current_index = 0;
    pwm_handler.is_running = 1;

    // Start PWM z DMA
    HAL_TIM_PWM_Start_DMA(pwm_handler.htim, TIM_CHANNEL_1, (uint32_t*)pwm_handler.pwm_values, pwm_handler.total_values);
}

void PWM_DMA_Stop(void) {
    if (!pwm_handler.is_running) {
        return;
    }

    HAL_TIM_PWM_Stop_DMA(pwm_handler.htim, TIM_CHANNEL_1);
    pwm_handler.is_running = 0;
}

// DMA callback pokazuje który indeks czyli jaka wartość jest teraz używana
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim != pwm_handler.htim || !pwm_handler.is_running) {
        return;
    }

    pwm_handler.current_index = (pwm_handler.current_index + 1) % pwm_handler.total_values;
}

typedef struct {
    uint8_t buffer[BUFFER_SIZE]; // Tablica na dane
    uint16_t head;              // Wskaźnik na miejsce wstawienia nowego elementu
    uint16_t tail;              // Wskaźnik na miejsce odczytu elementu
    uint16_t count;             // Liczba elementów w buforze
} CircularBuffer;

//dodawanie do bufora kołowego wyników
CircularBuffer cb;

// Inicjalizacja bufora
void CircularBuffer_Init(CircularBuffer* cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

// Dodanie elementu do bufora
bool CircularBuffer_Put(CircularBuffer* cb, float data) {
    if (cb->count >= BUFFER_SIZE) {
        // full, zacznij od początku
        cb->buffer[cb->head] = data;
        cb->head = (cb->head + 1) % BUFFER_SIZE;
        cb->tail = cb->head; // zawijam tail zeby był za głową
    } else {
        cb->buffer[cb->head] = data;
        cb->head = (cb->head + 1) % BUFFER_SIZE;
        cb->count++;
    }
    return true;
}

// Odczytanie elementu z bufora
bool CircularBuffer_Get(CircularBuffer* cb, float* data) {
    if (cb->count == 0) {
        // Bufor jest pusty
        return false;
    }
    *data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFFER_SIZE;
    cb->count--;
    return true;
}

// Sprawdzenie, czy bufor jest pusty
uint8_t CircularBuffer_IsEmpty(CircularBuffer* cb) {
    return (cb->count == 0) ? 1 : 0;  // 1 gdy pusty 0 gdy nie
}

// Zwrócenie liczby elementów w buforze
uint16_t CircularBuffer_Size(CircularBuffer* cb) {
    return cb->count;
}

bool CircularBuffer_Peek(const CircularBuffer* cb, uint16_t pos, float* value) {
    if (pos >= cb->count) return false;

    uint16_t actual_pos = (cb->tail + pos) % BUFFER_SIZE;
    *value = cb->buffer[actual_pos];
    return true;
}


void escapeByte(uint8_t byte, uint8_t *outBuf, uint16_t *outLen, uint16_t outMax)
{
if (*outLen >= outMax) return; // zabezpieczenie

if (byte == HDLC_FLAG) //sprawdzam czy bajt to 7E
{
    // 0x7E -> 0x7D 0x5E
    if (*outLen + 2 <= outMax) // jak tak to dodaj 2 bajty dla escapeów
    {
        outBuf[(*outLen)++] = HDLC_ESCAPE;
        outBuf[(*outLen)++] = HDLC_ESCAPE_7E;
    }
}
else if (byte == HDLC_ESCAPE) //7D jest używany do oznaczani ze następny bajt w danych jest zakodowany
{
    // 0x7D -> 0x7D 0x5D
    if (*outLen + 2 <= outMax) //tez dodaje 2 bajty
    {
        outBuf[(*outLen)++] = HDLC_ESCAPE;
        outBuf[(*outLen)++] = HDLC_ESCAPE_7D;
    }
}
else
{
    // normalny bajt
    outBuf[(*outLen)++] = byte;
}
}





uint8_t computeCRC8(const uint8_t *data, uint16_t length) {
    uint8_t crc = 0x00; // zaczynaj od 0x00

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i]; // xor obecny bit

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) { // sprawdz najważniejszy bit, używamy maski 0x80 (binarnie: 10000000)
                crc = (crc << 1) ^ 0x07; //Przesuń w lewo i xor z polinomial(0x07)
            } else {
                crc <<= 1; // Przesuń w lewo
            }
        }
    }

    return crc;
}



uint8_t USART_kbhit(){
		if(USART_RX_Empty==USART_RX_Busy){
			return 0;
		}else{
			return 1;
		}
	} //Jeśli są równe, oznacza to, że bufor odbiorczy jest pusty i funkcja zwraca 0
	 //Jeśli są różne, oznacza to, że w buforze są dane i funkcja zwraca 1

	int16_t USART_getchar(){
	int16_t tmp;

		if(USART_RX_Empty!=USART_RX_Busy){ //sprawdza czy są dane do odczytu
			 tmp=USART_RxBuf[USART_RX_Busy]; // jeżeli tak to przechowaj bajt w zmiennej tmp
			 USART_RX_Busy++; // przejdź dalej
			 if(USART_RX_Busy >= USART_RXBUF_LEN)USART_RX_Busy=0; //zresetuj, jeżeli osiągnie max rozmiar USART_RXBUF_LEN(bufor do odczytu) czyli nastąpi zapętlenie
			 return tmp;
		}else return -1; //zwraca -1 jeżeli bufor jest pusty
	}

	uint8_t USART_getline(char *buf){ //pobierająca wskaźnik do danego miejsca bufora
	static uint8_t bf[128];
	static uint8_t idx=0; //statyczny index pokazująca aktualną pozycję w buforze bf, przez to,
							//że jest statyczny to nie jest resetowany między wywołaniami funckji
	int i;
	uint8_t ret; //zmienna do przechowywania liczby znaków odebranych
	 while(USART_kbhit()){ //najpierw sprawdza czy jest coś do odczytu
		  bf[idx]=USART_getchar(); //Odczytuje jeden znak z bufora odbiorczego USART za pomocą funkcji USART_getchar() i zapisuje go do bufora bf na pozycji idx

		  if(((bf[idx]==10)||(bf[idx]==13))){ //Jeśli odczytany znak to kod ASCII 10 (newline) lub 13 (carriage return), oznacza to koniec linii
			  	  	  	  	  	  	  	  	  //Ostatni element bufora bf jest ustawiany na 0 (znak końca ciągu) w celu zakończenia łańcucha znaków
			  bf[idx]=0;
			  for(i=0;i<=idx;i++){
				 buf[i]=bf[i];//przekopiuj do bufora
			  }
			  ret=idx;	//Zmienna ret jest ustawiana na wartość idx, aby oznaczać długość odebranej linii
			  idx=0; //Indeks idx jest resetowany do 0, aby przygotować bufor bf do odbioru nowej linii
			  return ret;//odebrano linie
		  }else{//jesli tekst dluzszy to ustawiamy index na początek bufora (zapętlenie)
			  idx++;
			  if(idx>=128)idx=0;
		  }
	  }
	  return 0;
	}//USART_getline

	void USART_fsend(char* format,...){ // służy do wysyłania sformatowanych danych przez interfejs USART z wykorzystaniem bufora kołowego i przerwań
	char tmp_rs[2007]; //Tymczasowy bufor na sformatowane dane do wysłania
	int i; //do iteracji
	__IO int idx; //zmienna wskaźnika indeksu w buforze nadawczym (USART_TXBUFF)
	va_list arglist; //typ do obsługi listy  argumentów
	  va_start(arglist,format); // inicjalizuje listę argumentów zmiennej liczby dla formatowania danych
	  vsprintf(tmp_rs,format,arglist); //formatuje dane wejściowe zgodnie z przekazanym łańcuchem format i argumentami i zapisuje wynik w buforze tymczasowym tmp_rs
	  va_end(arglist); //kończy obsługę argumentów zmiennej liczby
	  idx=USART_TX_Empty; // wskaźnik idx jest ustawiany na aktualną pozycję USART_TX_Empty (wskaźnik indeksu pustego miejsca w buforze nadawczym)
	  for(i=0;i<strlen(tmp_rs);i++){ //iteruje przez całą długość sformatowanych danych
		  USART_TxBuf[idx]=tmp_rs[i]; //zapisuje je do bufora nadawczego
		  idx++; //przesuwa wskaźnik po buforze
		  if(idx >= USART_TXBUF_LEN)idx=0; //jeżeli dojdzie do końca to zresetuj bufor (pozycja początkowa)
	  }
	  __disable_irq(); //wyłącza przerwania, aby zabezpieczyć krytyczną sekcję kodu.
	  if((USART_TX_Empty==USART_TX_Busy)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){//sprawdzic dodatkowo zajetosc bufora nadajnika
		  USART_TX_Empty=idx;
		  uint8_t tmp=USART_TxBuf[USART_TX_Busy];
		  USART_TX_Busy++;
		  if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
		  HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	  }else{
		  USART_TX_Empty=idx;
	  }//    Jeśli wskaźnik USART_TX_Empty jest równy USART_TX_Busy (bufor nadawczy nie jest aktualnie zajęty) i flaga UART_FLAG_TXE jest ustawiona (USART jest gotowy do transmisji), to zaczyna się proces transmisji.
	    //Zmienna USART_TX_Empty jest aktualizowana do nowej pozycji idx.
	    //Pobierany jest pierwszy bajt z bufora USART_TxBuf w pozycji USART_TX_Busy i zapisywany do zmiennej tmp.
	    //USART_TX_Busy jest zwiększany o 1, a jeśli osiągnie USART_TXBUF_LEN, zostaje zresetowany do 0.
	    //HAL_UART_Transmit_IT(&huart2, &tmp, 1); wywołuje funkcję HAL do rozpoczęcia transmisji pierwszego bajtu w trybie przerwań.
	  	  //Jeśli warunek nie jest spełniony, USART_TX_Empty jest aktualizowany do nowego idx.
	  __enable_irq(); //włącza przerwania po zakończeniu krytycznej sekcji
	}//fsend

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	   if(huart==&huart2){
		   if(USART_TX_Empty!=USART_TX_Busy){ // Sprawdza, czy są jeszcze dane w buforze do wysłania. Jeśli wskaźnik USART_TX_Empty różni się od USART_TX_Busy, oznacza to, że bufor nadawczy nie jest pusty.
			   uint8_t tmp=USART_TxBuf[USART_TX_Busy]; // Pobiera kolejny bajt danych do wysłania z bufora USART_TxBuf w pozycji USART_TX_Busy
			   USART_TX_Busy++; //Przeskok do kolejnego bajtu
			   if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0; // Jeżeli dojdzie do końca resetuj do pozycji początkowej (powrót do 0)
			   HAL_UART_Transmit_IT(&huart2, &tmp, 1); //Rozpoczyna transmisję kolejnego bajtu danych w trybie przerwań
		   }
	   }
	}
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		 if(huart==&huart2){ //Sprawdza, czy przerwanie dotyczy danego interfejsu UART huart2
			 USART_RX_Empty++; //Przesuwa wskaźnik wskazując na następne miejsce w buforze odbiorczym
			 if(USART_RX_Empty>=USART_RXBUF_LEN)USART_RX_Empty=0;// Jeżeli dojdzie do końca resetuj do pozycji początkowej (powrót do 0)
			 HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1); //Ustawia przerwanie odbioru kolejnego bajtu do bufora USART_RxBuf w pozycji USART_RX_Empty

		 }
	}





// Struktura pakietu
typedef struct {
    uint8_t cmd_type;    // Typ komendy
    uint8_t data_len;    // Długość danych
    uint8_t data[16];    // Dane
} CommandPacket;


//tymczasowy bufor na odp komend później do przesłania HDLC_SendFrame tak jest
uint8_t tempbufanswer[MAX_DATA_LEN];
size_t bufIndex = 0;

// Funkcja do dodawania odpowiedzi do bufora
void addToResponse(uint8_t* data, uint8_t len) {
    if (bufIndex + len <= MAX_DATA_LEN) {
        memcpy(&tempbufanswer[bufIndex], data, len);
        bufIndex += len;
    }
}
// funkcja sprawdzająca, czy bufor odpowiedzi zawiera wyłącznie komunikaty "ERROR "
uint8_t allResponsesAreError(void) {
    if (bufIndex == 0) return 0; // Bufor pusty - nie traktujemy jako same błędy

    // długość nie jest wielokrotnością 6, nie są same bloki "ERROR "
    if (bufIndex % 6 != 0) return 0;

    uint16_t numBlocks = bufIndex / 6;

    for (uint16_t i = 0; i < numBlocks; i++) {
        //ERROR plus spacja
        if (memcmp(&tempbufanswer[i * 6], "ERROR ", 6) != 0)
            return 0; // Znaleziono blok inny niż "ERROR "
    }

    return 1; // Wszystkie bloki to "ERROR "
}

// ---------------- Handler dla SET1 i SET2 ----------------
  static void SET1or2(uint8_t *cmdData, uint16_t length) {
       if (length != 9 || cmdData[length - 1] != ']') {
           uint8_t resp[] = "ERROR";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       uint16_t value = 0;
       for (uint16_t i = 5; i < 8; i++) {
           if (cmdData[i] < '0' || cmdData[i] > '9') {
               if (cmdData[3] == '1') {
                   uint8_t resp[] = "S1 INVALID FORMAT ";
                   addToResponse(resp, sizeof(resp) - 1);
               } else {
                   uint8_t resp[] = "S2 INVALID FORMAT ";
                   addToResponse(resp, sizeof(resp) - 1);
               }
               return;
           }
           value = value * 10 + (cmdData[i] - '0');
       }
       if (value > 180) {
           if (cmdData[3] == '1') {
               uint8_t resp[] = "S1 INVALID FORMAT ";
               addToResponse(resp, sizeof(resp) - 1);
           } else {
               uint8_t resp[] = "S2 INVALID FORMAT ";
               addToResponse(resp, sizeof(resp) - 1);
           }
           return;
       }
       uint16_t pulse_ticks = SERVO_MIN_TICKS + (value * (SERVO_MAX_TICKS - SERVO_MIN_TICKS) / 180);
       if (cmdData[3] == '1') {
           __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_ticks);
           uint8_t resp[] = "S1 SET ";
           addToResponse(resp, sizeof(resp) - 1);
       } else {
           __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_ticks);
           uint8_t resp[] = "S2 SET ";
           addToResponse(resp, sizeof(resp) - 1);
       }
   }

   // ---------------- Handler dla ADDDMA ----------------
  static void ADDDMA(uint8_t* cmdData, uint16_t length) {
       if (length != 11 || cmdData[length - 1] != ']') {
           uint8_t resp[] = "DMA INVALID FORMAT";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       uint16_t value = 0;
       for (uint16_t i = 7; i < 10; i++) {
           if (cmdData[i] < '0' || cmdData[i] > '9') {
               uint8_t resp[] = "DMA INVALID FORMAT";
               addToResponse(resp, sizeof(resp) - 1);
               return;
           }
           value = value * 10 + (cmdData[i] - '0');
       }
       if (value > 180) {
           uint8_t resp[] = "DMA INVALID FORMAT";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       HAL_StatusTypeDef status = PWM_DMA_AddValue(value);
       if (status == HAL_OK) {
           uint8_t resp[] = "DMA ADDED";
           addToResponse(resp, sizeof(resp) - 1);
       } else {
           uint8_t resp[] = "DMA INVALID FORMAT";
           addToResponse(resp, sizeof(resp) - 1);
       }
   }

   // ---------------- Handler dla SAUTO ----------------
  static void SAUTO(uint8_t *cmdData, uint16_t length) {
       if (length != 8 || cmdData[length - 1] != ']') {
           uint8_t resp[] = "SAUTO INVALID FORMAT";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       uint8_t param = cmdData[6];
       if (param == '1') {
           PWM_DMA_Start();
           uint8_t resp[] = "SAUTO ON";
           addToResponse(resp, sizeof(resp) - 1);
       } else if (param == '0') {
           PWM_DMA_Stop();
           uint8_t resp[] = "SAUTO OFF";
           addToResponse(resp, sizeof(resp) - 1);
       }
   }

   // ---------------- Handler dla UA? ----------------
  static void UA(uint8_t* cmdData, uint16_t length) {
       uint16_t ms = htim6.Init.Period + 1;
       // Format: "UA " + 4-cyfrowa liczba, bez znaku '\0'
       uint8_t resp[7];
       resp[0] = 'U';
       resp[1] = 'A';
       resp[2] = ' ';
       resp[3] = '0' + (ms / 1000);         // tysiące
       resp[4] = '0' + ((ms % 1000) / 100);   // setki
       resp[5] = '0' + ((ms % 100) / 10);     // dziesiątki
       resp[6] = '0' + (ms % 10);             // jedności
       addToResponse(resp, 7);
   }

   // ---------------- Handler dla UA[xxx] ----------------
  static void UAxxx(uint8_t *cmdData, uint16_t length) {
       if (length != 8 || cmdData[length - 1] != ']') {
           uint8_t resp[] = "ERROR";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       uint16_t value = 0;
       for (uint16_t i = 3; i < 7; i++) {
           if (cmdData[i] < '0' || cmdData[i] > '9') {
               uint8_t resp[] = "ERROR";
               addToResponse(resp, sizeof(resp) - 1);
               return;
           }
           value = value * 10 + (cmdData[i] - '0');
       }
       if (value >= 10 && value <= 1000) {
           SetUltrasonicInterval(value);
           uint8_t resp[] = "UA SET";
           addToResponse(resp, sizeof(resp) - 1);
       }
   }

   // ---------------- Handler dla BUF ----------------
  static void BUF(uint8_t *cmdData, uint16_t length) {
       if (length != 3) {
           uint8_t resp[] = "ERROR";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       if (CircularBuffer_IsEmpty(&cb)) {
           uint8_t resp[] = "NO DATA";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       float value;
       for (int i = 0; i < 20; i++) {
           if (CircularBuffer_Get(&cb, &value)) {
               char temp[16];
               int len = snprintf(temp, sizeof(temp), "%.2f ", value);
               addToResponse((uint8_t*)temp, (uint8_t)len);
           }
       }
   }

   // ---------------- Handler dla BUFALL ----------------
  static void BUFALL(uint8_t *cmdData, uint16_t length) {
       if (length != 6) {
           uint8_t resp[] = "ERROR";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       if (CircularBuffer_IsEmpty(&cb)) {
           uint8_t resp[] = "NO DATA";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       float value;
       while (CircularBuffer_Get(&cb, &value)) {
           char temp[16];
           int len = snprintf(temp, sizeof(temp), "%.2f ", value);
           addToResponse((uint8_t*)temp, (uint8_t)len);
       }
   }

   // ---------------- Handler dla BUFN[xxx,xxx] ----------------
  static void BUFN(uint8_t *cmdData, uint16_t length) {
       // Oczekujemy dokładnie 13 bajtów: "BUFN[xxx,xxx]"
       if (length != 13 || cmdData[length - 1] != ']') {
           uint8_t resp[] = "BUFN INVALID FORMAT";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       if (cmdData[8] != ',') {
           uint8_t resp[] = "BUFN INVALID FORMAT";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       uint16_t start = 0, end = 0;
       for (uint8_t i = 5; i < 8; i++) {
           if (!isdigit(cmdData[i])) {
               uint8_t resp[] = "BUFN INVALID START";
               addToResponse(resp, sizeof(resp) - 1);
               return;
           }
           start = start * 10 + (cmdData[i] - '0');
       }
       for (uint8_t i = 9; i < 12; i++) {
           if (!isdigit(cmdData[i])) {
               uint8_t resp[] = "BUFN INVALID END";
               addToResponse(resp, sizeof(resp) - 1);
               return;
           }
           end = end * 10 + (cmdData[i] - '0');
       }
       if (start > 500 || end > 500 || start > end) {
           uint8_t resp[] = "BUFN INVALID RANGE";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       uint16_t total = CircularBuffer_Size(&cb);
       if (start >= total || end >= total) {
           uint8_t resp[] = "BUFN INVALID RANGE";
           addToResponse(resp, sizeof(resp) - 1);
           return;
       }
       char outStr[32];
       for (uint16_t i = start; i <= end; i++) {
           float value;
           if (CircularBuffer_Peek(&cb, i, &value)) {
               int len = snprintf(outStr, sizeof(outStr), "%.2f ", value);
               addToResponse((uint8_t*)outStr, (uint8_t)len);
           }
       }
   }

void processCommand(uint8_t *cmdData, uint16_t length) {

    if (length >= 5 && ((memcmp(cmdData, "SET1[", 5) == 0) || (memcmp(cmdData, "SET2[", 5) == 0))) {
        SET1or2(cmdData, length);
    } else if (length >= 7 && (memcmp(cmdData, "ADDDMA[", 7) == 0)) {
        ADDDMA(cmdData, length);
    } else if (length >= 6 && (memcmp(cmdData, "SAUTO[", 6) == 0)) {
        SAUTO(cmdData, length);
    } else if (length >= 3 && (memcmp(cmdData, "UA?", 3) == 0)) {
        UA(cmdData, length);
    } else if (length >= 3 && (memcmp(cmdData, "UA[", 3) == 0)) {
        UAxxx(cmdData, length);
    } else if (length >= 6 && (memcmp(cmdData, "BUFALL", 6) == 0)) {
        BUFALL(cmdData, length);
    } else if (length >= 5 && (memcmp(cmdData, "BUFN[", 5) == 0)) {
        BUFN(cmdData, length);
    } else if (length >= 3 && (memcmp(cmdData, "BUF", 3) == 0)) {
        BUF(cmdData, length);
    } else {
        uint8_t resp[] = "ERROR ";
        addToResponse(resp, sizeof(resp) - 1);
    }
}
void processMultipleCommands(uint8_t *cmdData, uint16_t totalLength) {
    uint16_t pos = 0;
    while (pos < totalLength) {
        // Pomijam bajty separatora (0x3B to ';' w ASCII)
        while (pos < totalLength && cmdData[pos] == 0x3B) {
            pos++;
        }
        if (pos >= totalLength)
            break;

        uint16_t start = pos;
        // Szuka separatora lub końca bufora
        while (pos < totalLength && cmdData[pos] != 0x3B) {
            pos++;
        }
        uint16_t cmdLength = pos - start;
        if (cmdLength > 0) {
            processCommand(&cmdData[start], cmdLength);  // Przekazuje surowe bajty
        }
    }
}

void HDLC_ParseFrame(const uint8_t* frame, uint16_t length) {
    if (length < HDLC_MIN_FRAME_SIZE)
        return;

    uint8_t addrSrc = frame[0];
    uint8_t addrDst = frame[1];
    uint16_t dataLen = (frame[2] << 8) | frame[3];

    if (dataLen + 5 > length) {
        uint8_t message[] = "LEN NOT MATCH";
        HDLC_SendFrame(addrSrc, addrDst, message, sizeof(message) - 1);
        return;
    }

    const uint8_t* dataPtr = &frame[4];
    uint8_t crcRecv = frame[4 + dataLen];

    // Przygotowanie bufora do obliczenia CRC
    uint8_t tempBuf[4 + dataLen];
    tempBuf[0] = addrSrc;
    tempBuf[1] = addrDst;
    tempBuf[2] = frame[2];
    tempBuf[3] = frame[3];
    memcpy(&tempBuf[4], dataPtr, dataLen);

    // Oblicz CRC
    uint8_t crcCalc = computeCRC8(tempBuf, 4 + dataLen);
    if (crcCalc != crcRecv) {
        uint8_t message[] = "INVALID CRC";
        HDLC_SendFrame(addrSrc, addrDst, message, sizeof(message) - 1);
        return;
    }

    static uint8_t cmdBuff[MAX_DATA_LEN];
    if (dataLen >= MAX_DATA_LEN)  // Zabezpieczenie przed przepełnieniem
        return;

    uint16_t cmdLen = dataLen;
    memcpy(cmdBuff, dataPtr, cmdLen);

    // Przetwarzam wiele komend oddzielonych średnikiem
    processMultipleCommands(cmdBuff, cmdLen);

    // Jeśli bufor odpowiedzi zawiera same "ERROR", ramka jest odrzucana
    if (!allResponsesAreError()) {
        HDLC_SendFrame(addrSrc, addrDst, tempbufanswer, (uint16_t)bufIndex);

        // Reset bufora
        memset(tempbufanswer, 0, sizeof(tempbufanswer));
        bufIndex = 0;
    }
}





	int8_t HDLC_ProcessInput(void)
	{
	    while (USART_kbhit())
	    {
	        uint8_t rxByte = (uint8_t)USART_getchar();

	        // sprawdzenie czy bufor jest pełny
	        if (hdlcInPos >= HDLC_MAX_FRAME_SIZE)
	           {
	               hdlcRxState = WAITING_FOR_FLAG;
	               hdlcInPos = 0;
	               continue;
	           }

	           switch (hdlcRxState)
	           {
	               case WAITING_FOR_FLAG:
	                   if (rxByte == HDLC_FLAG)
	                   {
	                       // Flaga oznacza początek ramki
	                       hdlcInPos = 0;
	                       hdlcRxState = READING_FRAME;
	                   }
	                   break;

	               case READING_FRAME:
	                   if (rxByte == HDLC_FLAG)
	                   {
	                       // Otrzymano flagę - traktujemy ją jako koniec bieżącej ramki
	                       if (hdlcInPos > 0)
	                       {
	                           HDLC_ParseFrame(hdlcInBuf, hdlcInPos);
	                       }
	                       // Resetuj bufor, ale ustaw stan na READING_FRAME,
	                       // bo ten sam bajt 0x7E ma być traktowany jako początek nowej ramki
	                       hdlcInPos = 0;
	                       hdlcRxState = READING_FRAME;
	                   }
	                   else if (rxByte == HDLC_ESCAPE)
	                   {
	                       hdlcRxState = ESCAPING_BYTE;
	                   }
	                   else
	                   {
	                       hdlcInBuf[hdlcInPos++] = rxByte;
	                   }
	                   break;

	               case ESCAPING_BYTE:
	               {
	                   uint8_t decodedByte;
	                   if (rxByte == HDLC_ESCAPE_7E)
	                       decodedByte = HDLC_FLAG;
	                   else if (rxByte == HDLC_ESCAPE_7D)
	                       decodedByte = HDLC_ESCAPE;
	                   else
	                   {
	                       // Błędna sekwencja escape – odrzucamy ramkę
	                       hdlcRxState = WAITING_FOR_FLAG;
	                       hdlcInPos = 0;
	                       break;
	                   }
	                   hdlcInBuf[hdlcInPos++] = decodedByte;
	                   //powracamy do stanu odbioru ramki
	                   hdlcRxState = READING_FRAME;
	               }
	               break;

	               default:
	                   hdlcRxState = WAITING_FOR_FLAG;
	                   hdlcInPos = 0;
	                   break;
	           }
	       }
	       return 0;
	}


	void HDLC_SendFrame(uint8_t addrSrc, uint8_t addrDst,
	                    const uint8_t* data, uint16_t dataLen)
	{
	    uint8_t outBuf[HDLC_MAX_FRAME_SIZE] = {0};
	    uint16_t outPos = 0;

	    // Dodaj flagę startu
	    outBuf[outPos++] = HDLC_FLAG;

	    // Przygotowanie bufora do obliczenia CRC
	    uint8_t tempBuf[4 + dataLen];
	    tempBuf[0] = addrDst;
	    tempBuf[1] = addrSrc;
	    tempBuf[2] = (uint8_t)(dataLen >> 8);
	    tempBuf[3] = (uint8_t)(dataLen & 0xFF); //0xff = 255 to "filtr" & i pozostawia 8 bitów
	    memcpy(&tempBuf[4], data, dataLen);


	    // Dodaj adresy, długość i dane z escapowaniem
	    escapeByte(addrDst, outBuf, &outPos, HDLC_MAX_FRAME_SIZE);
	    escapeByte(addrSrc, outBuf, &outPos, HDLC_MAX_FRAME_SIZE);
	    escapeByte((uint8_t)(dataLen >> 8), outBuf, &outPos, HDLC_MAX_FRAME_SIZE);
	    escapeByte((uint8_t)(dataLen & 0xFF), outBuf, &outPos, HDLC_MAX_FRAME_SIZE);

	    for (uint16_t i = 0; i < dataLen; i++) {
	        escapeByte(data[i], outBuf, &outPos, HDLC_MAX_FRAME_SIZE);
	    }
	    // Oblicz CRC
	    uint8_t crcVal = computeCRC8(tempBuf, 4 + dataLen);
	    // Dodaj CRC z escapowaniem
	    escapeByte(crcVal, outBuf, &outPos, HDLC_MAX_FRAME_SIZE);

	    // Flaga końca
	    outBuf[outPos++] = HDLC_FLAG;

	    // Wysłanie ramki
	    HAL_UART_Transmit(&huart2, outBuf, outPos, 1000);

	}



	// czujnik

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
	    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	    {
	        if (echo_captured == 0)
	        {
	            // Pierwsze zbocze narastające start impulsu
	            echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            echo_captured = 1;

	            // Zmień polaryzację na falling
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	        }
	        else if (echo_captured == 1)
	        {
	            // Drugie zbocze opadające koniec impulsu
	            echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            echo_captured = 2; // pomiar zakończony

	            // Przywróć polaryzację na rising (następny pomiar)
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
	        }
	    }
	}


		float dist = 0.0f;

		float GetUltrasonicDistance(void)
		{
			//Jeżeli echo zostało przechwycobe
			if (echo_captured == 2)
			{
				// czas trwania
				uint32_t duration;
				if (echo_end >= echo_start)
					duration = echo_end - echo_start;
				else
				{
					uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
					duration = (arr - echo_start + echo_end);
				}

				dist = (float)duration / 58.0f;
				dist = roundf(dist*100.0f)/100.0f; // zaokrąglenie do 2 miejsc

				CircularBuffer_Put(&cb, dist);

				echo_captured = 0; // Reset
				return dist;
			}
			else
			{
				return -1.0f;
			}
		}

	void SetUltrasonicInterval(uint16_t ms) {
	    // Oblicz wartości prescalera i period
	    uint32_t timer_freq = HAL_RCC_GetPCLK1Freq(); // Częstotliwość zegara
	    uint16_t prescaler = (timer_freq / 1000) - 1; // Dzielnik częstotliwości
	    uint16_t period = ms - 1; // Okres

	    htim6.Init.Prescaler = prescaler;
	    htim6.Init.Period = period;

	    HAL_TIM_Base_Init(&htim6);
	}



	void TriggerUltrasonic(void) {
	    // impuls TRIG używając timera
	    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	    HAL_TIM_Base_Start_IT(&htim7);  // skonfigurowany na 10us


	}

	//  przerwania timera
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	    if (htim->Instance == TIM7) {
	        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	        HAL_TIM_Base_Stop_IT(&htim7);
	    }
	    //  TIM6
	    if (htim->Instance == TIM6) {
	        TriggerUltrasonic();
	    }
	}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty], 1);
  // Start TIM6 do generowania triggera w czujniku
 	    HAL_TIM_Base_Start_IT(&htim6);

 	    // Start TIM3 Input Capture na chan 1
 	    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); //włącza przerwanie ale w sumie nie wiem czy to tu powinno być w ogóle

 	   // startuje pwm
 	   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //servo 1
 	   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //servo 2

 		//nie mam pojęcia co się dzieje i jak działają te przerwania ale buja nieźle


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  //ServoWiper(); // na testowanie serva

		  HDLC_ProcessInput();

		  //czujnik
		  // żeby działał to trzeba zrobić tigger na high przez minimum 10 us
		  GetUltrasonicDistance();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 54;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65453;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 44;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 38000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 179;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin|TRIG_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin TRIG_Pin_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin|TRIG_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
