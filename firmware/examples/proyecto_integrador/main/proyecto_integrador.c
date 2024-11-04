/*! @mainpage Sistema de monitoreo y control de clima en invernadero
 *
 * @section genDesc General Description
 *
 * Este proyecto permite, por un lado, monitorear el clima dentro de un invernadero a través de la medición de variables físicas como temperatura-humedad ambiente y humedad de suelo con la ayuda de sensores específicos. Por otro lado, permite realizar el control del clima mediante los actuadores, para ello con la activación de una electroválvula se realiza el riego del suelo y con un sistema de ventilación (simulado con un ventilador dc) se controla el clima del interior del invernadero. El sistema además, permitirá fijar horarios de riego para un mayor control, ya que muchas veces no se recomienda el riego en horarios nocturnos. El rango de los parámetros (humedad de suelo y sensación térmica) para la activación de los actuadores se fijará mediante una aplicación móvil con comunicación vía Bluetooth. A su vez, el dispositivo contará con un display oled para mostrar de forma constante las variables censadas y el estado del sistema.
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN A0 YL-69 	 	| 	CH1	 |
 * | 		 	           | 	CH0	|
 * | 		 	           | 	GND	|
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 16/10/2024 | Document creation		                         |
 *
 * @author Cristian Schmidt (cristian.schmidt@ingenieria.uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "timer_mcu.h"
#include "gpio_mcu.h"
#include "analog_io_mcu.h"
#include "switch.h"
#include "rtc_mcu.h"
#include "dht11.h"
#include "uart_mcu.h"
#include "ble_mcu.h"
#include "led.h"

/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD_GENERAl_US 1000 * 1000
#define CONFIG_BLINK_PERIOD_BLUETOOTH_US 500 * 1000
#define LED_BT LED_1

/*==================[internal data definition]===============================*/
typedef struct
{
    gpio_t pin; /*!< GPIO pin number */
    io_t dir;   /*!< GPIO direction '0' IN;  '1' OUT*/
} gpioConf_t;

gpioConf_t encender_riego = {GPIO_20, GPIO_OUTPUT};
gpioConf_t encender_ventilacion = {GPIO_21, GPIO_OUTPUT};
gpioConf_t pin_dht11 = {GPIO_22, GPIO_INPUT};

TaskHandle_t sensor_hum_suelo_task_handle = NULL;
TaskHandle_t control_clima_task_handle = NULL;
TaskHandle_t sensor_dht11_task_handle = NULL;
TaskHandle_t uart_task_handle = NULL;
TaskHandle_t bluetooth_task_handle = NULL;

rtc_t horarios_riego[4];
uint8_t t_control = 0;
uint8_t t_pausa = 0;
uint8_t humedad_suelo = 0;
float float_humedad_ambiente = 0;
float float_temperatura_ambiente = 0;
uint8_t humedad_ambiente = 0;
uint8_t temperatura_ambiente = 0;
uint8_t sensacion_termica = 0;
uint8_t hum_suelo_control_min = 0;
uint8_t hum_suelo_control_max = 0;
uint8_t temp_amb_control_min = 0;
uint8_t temp_amb_control_max = 0;
rtc_t hora_prueba;
bool encendido = false;
bool riego_encendido = false;
bool ventilacion_encendida = false;

/*==================[internal functions declaration]=========================*/
rtc_t sumaHora(rtc_t hora, uint8_t minutos)
{
    rtc_t hora_final;
    uint8_t minutos_final;
    uint8_t h = hora.hour * 100;
    uint8_t m = hora.min * 10;
    uint8_t cant_horas = minutos / 60;
    uint8_t min_restantes = minutos - (60 * cant_horas);
    uint8_t hora_restante;
    if ((min_restantes + m) > 60)
    {
        minutos_final = (min_restantes + m) - 60;
        cant_horas++;
    }
    else
    {
        minutos_final = min_restantes + m;
    }
    hora_restante = (h + cant_horas * 100) / 100;
    if (hora_restante > 24)
    {
        hora_final.hour = (hora_restante - 24) * 100;
        hora_final.min = minutos_final;
    }
    else
    {
        hora_final.hour = h + cant_horas * 100;
        hora_final.min = minutos_final;
    }
    return hora_final;
}

void teclaUno()
{
    encendido = !encendido;
}

void FuncTimerA(void *param)
{
    vTaskNotifyGiveFromISR(sensor_hum_suelo_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(sensor_dht11_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(control_clima_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(uart_task_handle, pdFALSE); /* Envía una notificación a la tarea asociada al uart */
}

void FuncTimerB(void *param)
{
    vTaskNotifyGiveFromISR(bluetooth_task_handle, pdFALSE);
}

static void medirHumedadSuelo(void *pvParameter)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (encendido)
        {
            AnalogInputReadSingle(CH1, &humedad_suelo);
            humedad_suelo = -0.78 * humedad_suelo + 176;
            if (humedad_suelo > 100)
            {
                humedad_suelo = 100;
            }
        }
    }
}

static void medirHumTempAmbiente(void *pvParameter)
{
    uint8_t temp_fahrenheit;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (encendido)
        {
            dht11Read(&float_humedad_ambiente, &float_temperatura_ambiente);
            humedad_ambiente = (uint8_t)(float_humedad_ambiente);
            temperatura_ambiente = (uint8_t)(float_temperatura_ambiente);
            if (temperatura_ambiente > 26.6 && temperatura_ambiente < 43.3 && humedad_ambiente > 40)
            {
                temp_fahrenheit = (temperatura_ambiente * 1.8) + 32;
                sensacion_termica = (-42.379 + (2.04901523 * temp_fahrenheit) + (10.14333127 * humedad_ambiente) - (0.22475541 * temp_fahrenheit * humedad_ambiente) - (6.83783e-3 * temp_fahrenheit * temp_fahrenheit) - (5.481717e-2 * humedad_ambiente * humedad_ambiente) + (1.22874e-3 * temp_fahrenheit * temp_fahrenheit * humedad_ambiente) + (8.5282e-4 * temp_fahrenheit * humedad_ambiente * humedad_ambiente) - (1.99e-6 * temp_fahrenheit * temp_fahrenheit * humedad_ambiente * humedad_ambiente));
                sensacion_termica = (sensacion_termica - 32) / 1.8;
            }
            else
            {
                sensacion_termica = temperatura_ambiente;
            }
        }
    }
}

static void controlClima(void *pvParameter)
{
    uint8_t t_medio = (t_control - t_pausa) / 2;
    uint8_t t_suma = t_medio + t_pausa;
    rtc_t hora_actual;
    rtc_t horario_riego_actual;
    uint16_t hora_riego_entero = 0;
    uint16_t hora_tmedio_entero = 0;
    uint16_t hora_tsuma_entero = 0;
    uint16_t hora_tcontrol_entero = 0;
    bool primer_riego = false;
    bool segundo_riego = false;
    bool riego_sin_horario = false;
    bool hora_de_regar = false;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        RtcRead(&hora_actual);
        uint16_t hora_actual_entero = hora_actual.hour * 100 + hora_actual.min;
        for (uint8_t i = 0; i < 4; i++)
        {
            if (hora_actual_entero == (horarios_riego[i].hour * 100 + horarios_riego[i].min))
            {
                hora_de_regar = true;
                horario_riego_actual = horarios_riego[i];
            }
            else
            {
                hora_de_regar = false;
            }
        }
        if (encendido && hora_de_regar)
        {
            hora_riego_entero = horario_riego_actual.hour * 100 + horario_riego_actual.min;
            hora_tmedio_entero = sumaHora(horario_riego_actual, t_medio).hour * 100 + sumaHora(horario_riego_actual, t_medio).min;
            hora_tsuma_entero = sumaHora(horario_riego_actual, t_suma).hour * 100 + sumaHora(horario_riego_actual, t_suma).min;
            hora_tcontrol_entero = sumaHora(horario_riego_actual, t_control).hour * 100 + sumaHora(horario_riego_actual, t_control).min;

            primer_riego = (hora_actual_entero >= hora_riego_entero) && (hora_actual_entero < hora_tmedio_entero);
            segundo_riego = (hora_actual_entero >= hora_tsuma_entero) && (hora_actual_entero < hora_tcontrol_entero);
            riego_sin_horario = (horario_riego_actual.hour == 0) && (horario_riego_actual.min == 0);

            if (primer_riego || segundo_riego || riego_sin_horario)
            {
                if ((humedad_suelo >= hum_suelo_control_min) && (humedad_suelo <= hum_suelo_control_max))
                {
                    GPIOOn(encender_riego.pin);
                    riego_encendido = true;
                }
                else
                {
                    GPIOOff(encender_riego.pin);
                    riego_encendido = false;
                }
            }
            else
            {
                GPIOOff(encender_riego.pin);
                riego_encendido = false;
            }
            if ((sensacion_termica >= temp_amb_control_min) && (sensacion_termica <= temp_amb_control_max))
            {
                GPIOOn(encender_ventilacion.pin);
                ventilacion_encendida = true;
            }
            else
            {
                GPIOOff(encender_ventilacion.pin);
                ventilacion_encendida = false;
            }
        }
        /*Envío de los datos para visualizarlos en la aplicacion movil*/
        char msg[12];
        if (riego_encendido)
        {
            sprintf(msg, "*RR255G0B0*");
        }
        else if (ventilacion_encendida)
        {
            sprintf(msg, "*VR255G0B0*");
        }
        BleSendString(msg);
        sprintf(msg, "*T%d*", temperatura_ambiente);
        BleSendString(msg);
        sprintf(msg, "*H%d*", humedad_ambiente);
        BleSendString(msg);
        sprintf(msg, "*t%d:%d*", hora_actual.hour, hora_actual.min);
        BleSendString(msg);
    }
}

static void uartTask(void *pvParameter)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* La tarea espera en este punto hasta recibir una notificación */
        if (encendido)
        {
            RtcRead(&hora_prueba);
            UartSendString(UART_PC, (char *)UartItoa(humedad_ambiente, 10));
            UartSendString(UART_PC, "% -- ");
            UartSendString(UART_PC, (char *)UartItoa(temperatura_ambiente, 10));
            UartSendString(UART_PC, "°C -- ");
            UartSendString(UART_PC, (char *)UartItoa(sensacion_termica, 10));
            UartSendString(UART_PC, "°C -- ");
            UartSendString(UART_PC, (char *)UartItoa(hora_prueba.hour % MAX_HOUR, 10));
            UartSendString(UART_PC, ":");
            UartSendString(UART_PC, (char *)UartItoa(hora_prueba.min % MAX_MIN, 10));
            UartSendString(UART_PC, "\r\n");
        }
    }
}

void read_data(uint8_t *data, uint8_t length)
{
    uint8_t i = 2;

    switch (data[0])
    {
    case 'E':
        encendido = true;
        break;
    case 'e':
        encendido = false;
        break;
    }

    if (data[0] == 't' && data[1] == 'C')
    {
        /* El slidebar Tiempo de Control envía los datos con el formato "tC" + value + "A" */
        t_control = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            t_control = t_control * 10;
            t_control = t_control + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 't' && data[1] == 'P')
    {
        /* El slidebar Tiempo de Pausa envía los datos con el formato "tP" + value + "A" */
        t_pausa = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            t_pausa = t_pausa * 10;
            t_pausa = t_pausa + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'T' && data[1] == 'L')
    {
        /* El slidebar Temperatura de Control Mínima envía los datos con el formato "TL" + value + "A" */
        temp_amb_control_min = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            temp_amb_control_min = temp_amb_control_min * 10;
            temp_amb_control_min = temp_amb_control_min + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'T' && data[1] == 'H')
    {
        /* El slidebar Temperatura de Control Máxima envía los datos con el formato "TH" + value + "A" */
        temp_amb_control_max = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            temp_amb_control_max = temp_amb_control_max * 10;
            temp_amb_control_max = temp_amb_control_max + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'H' && data[1] == 'L')
    {
        /* El slidebar Humedad Suelo de Control Mínima envía los datos con el formato "HL" + value + "A" */
        hum_suelo_control_min = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            hum_suelo_control_min = hum_suelo_control_min * 10;
            hum_suelo_control_min = hum_suelo_control_min + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'H' && data[1] == 'H')
    {
        /* El slidebar Humedad Suelo de Control Máxima envía los datos con el formato "HH" + value + "A" */
        hum_suelo_control_max = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            hum_suelo_control_max = hum_suelo_control_max * 10;
            hum_suelo_control_max = hum_suelo_control_max + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'H' && data[1] == '1')
    {
        /* El slidebar Hora de Riego 1 envía los datos con el formato "H1" + value + "A" */
        horarios_riego[0].hour = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[0].hour = horarios_riego[0].hour * 10;
            horarios_riego[0].hour = horarios_riego[0].hour + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'M' && data[1] == '1')
    {
        /* El slidebar Minutos de Riego 1 envía los datos con el formato "M1" + value + "A" */
        horarios_riego[0].min = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[0].min = horarios_riego[0].min * 10;
            horarios_riego[0].min = horarios_riego[0].min + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'H' && data[1] == '2')
    {
        /* El slidebar Hora de Riego 1 envía los datos con el formato "H1" + value + "A" */
        horarios_riego[1].hour = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[1].hour = horarios_riego[1].hour * 10;
            horarios_riego[1].hour = horarios_riego[1].hour + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'M' && data[1] == '2')
    {
        /* El slidebar Minutos de Riego 1 envía los datos con el formato "M1" + value + "A" */
        horarios_riego[1].min = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[1].min = horarios_riego[1].min * 10;
            horarios_riego[1].min = horarios_riego[1].min + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'H' && data[1] == '3')
    {
        /* El slidebar Hora de Riego 1 envía los datos con el formato "H1" + value + "A" */
        horarios_riego[2].hour = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[2].hour = horarios_riego[2].hour * 10;
            horarios_riego[2].hour = horarios_riego[2].hour + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'M' && data[1] == '3')
    {
        /* El slidebar Minutos de Riego 1 envía los datos con el formato "M1" + value + "A" */
        horarios_riego[2].min = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[2].min = horarios_riego[2].min * 10;
            horarios_riego[2].min = horarios_riego[2].min + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'H' && data[1] == '4')
    {
        /* El slidebar Hora de Riego 1 envía los datos con el formato "H1" + value + "A" */
        horarios_riego[3].hour = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[3].hour = horarios_riego[3].hour * 10;
            horarios_riego[3].hour = horarios_riego[3].hour + (data[i] - '0');
            i++;
        }
    }
    else if (data[0] == 'M' && data[1] == '4')
    {
        /* El slidebar Minutos de Riego 1 envía los datos con el formato "M1" + value + "A" */
        horarios_riego[3].min = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horarios_riego[3].min = horarios_riego[3].min * 10;
            horarios_riego[3].min = horarios_riego[3].min + (data[i] - '0');
            i++;
        }
    }
}

static void bluetoothTask(void *pvParameter)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* La tarea espera en este punto hasta recibir una notificación */
        switch (BleStatus())
        {
        case BLE_OFF:
            LedOff(LED_BT);
            break;
        case BLE_DISCONNECTED:
            LedToggle(LED_BT);
            break;
        case BLE_CONNECTED:
            LedOn(LED_BT);
            break;
        }
    }
}
/*==================[external functions definition]==========================*/
void app_main(void)
{
    ble_config_t ble_configuration = {
        "ESP_EDU_1",
        read_data};

    LedsInit();
    BleInit(&ble_configuration);

    SwitchesInit();

    GPIOInit(encender_riego.pin, encender_riego.dir);
    GPIOInit(encender_ventilacion.pin, encender_ventilacion.dir);
    GPIOInit(pin_dht11.pin, pin_dht11.dir);

    dht11Init(pin_dht11.pin);

    timer_config_t timer_1 = {
        .timer = TIMER_A,
        .period = CONFIG_BLINK_PERIOD_GENERAl_US,
        .func_p = FuncTimerA,
        .param_p = NULL};
    TimerInit(&timer_1);

    timer_config_t timer_2 = {
        .timer = TIMER_B,
        .period = CONFIG_BLINK_PERIOD_BLUETOOTH_US,
        .func_p = FuncTimerB,
        .param_p = NULL};
    TimerInit(&timer_2);

    analog_input_config_t sensorHumSuelo = {
        .input = CH1,
        .mode = ADC_SINGLE,
        .func_p = NULL,
        .param_p = NULL,
        .sample_frec = 0};
    AnalogInputInit(&sensorHumSuelo);

    serial_config_t my_uart = {
        .port = UART_PC,
        .baud_rate = 115200,
        .func_p = NULL,
        .param_p = NULL};

    UartInit(&my_uart);

    rtc_t config_time = {
        .year = 2024,
        .month = 4,
        .mday = 8,
        .wday = 1,
        .hour = 12,
        .min = 32,
        .sec = 0};
    RtcConfig(&config_time);

    AnalogOutputInit();

    SwitchActivInt(SWITCH_1, teclaUno, NULL);

    xTaskCreate(&medirHumTempAmbiente, "sensorHumTemAmbiente", 512, NULL, 5, &sensor_dht11_task_handle);
    xTaskCreate(&medirHumedadSuelo, "sensorHumSuelo", 512, NULL, 5, &sensor_hum_suelo_task_handle);
    xTaskCreate(&controlClima, "controlClima", 512, NULL, 5, &control_clima_task_handle);

    xTaskCreate(&uartTask, "UART", 4096, NULL, 5, &uart_task_handle);
    xTaskCreate(&bluetoothTask, "BLUETOOTH", 4096, NULL, 5, &bluetooth_task_handle);

    TimerStart(timer_1.timer);
    TimerStart(timer_2.timer);
}
/*==================[end of file]============================================*/