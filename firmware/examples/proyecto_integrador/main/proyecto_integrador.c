/*! @mainpage Sistema de Monitoreo y Control de Clima en Invernadero
 *
 * @section genDesc General Description
 *
 * Este proyecto permite, por un lado, monitorear el clima dentro de un invernadero a través de la medición de variables físicas como temperatura-humedad ambiente y humedad de suelo con la ayuda de sensores específicos. Por otro lado, permite realizar el control del clima mediante los actuadores, para ello con la activación de una electroválvula se realiza el riego del suelo y con un sistema de ventilación (simulado con un ventilador dc) se controla el clima del interior del invernadero. El sistema además, permitirá fijar horarios de riego para un mayor control, ya que muchas veces no se recomienda el riego en horarios nocturnos. El rango de los parámetros (humedad de suelo y sensación térmica) para la activación de los actuadores se fijará mediante una aplicación móvil con comunicación vía Bluetooth, la cual a su vez, permite la visualización del estado del clima en el interior del invernadero.
 *
 * @section hardConn Hardware Connection
 *
 * |    Sensor DHT11  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	VCC	    	|   	3V3 	|
 * | 	GND	        | 	    GND	    |
 * | 	DAT	        |  	  GPIO_22 	|
 *
 * 
 * |    RELÉS       |   ESP32   	|
 * |:--------------:|:--------------|   
 * | 	VCC	    	|   	3V3 	|
 * | 	GND	        | 	    GND	    |
 * | 	IN8	        |  	  GPIO_20 	|   
 * | 	IN2	        |  	  GPIO_21 	|   
 * 
 * 
 * |    Sensor HW-390  |   ESP32   	|   
 * |:--------------:|:--------------|
 * | 	VCC	    	|   	3V3 	|
 * | 	GND	        | 	    GND	    |
 * | 	AUOT	    |   	CH3 	|
 * 
 * 
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 16/10/2024 | Document creation		                         |
 * | 12/11/2024 | Document completion	                         |
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
#define CONFIG_BLINK_PERIOD_GENERAl_US 500 * 1000
#define CONFIG_BLINK_PERIOD_BLUETOOTH_US 500 * 1000
#define LED_BT LED_1

/*==================[internal data definition]===============================*/
typedef struct
{
    gpio_t pin; /*!< GPIO pin number */
    io_t dir;   /*!< GPIO direction '0' IN;  '1' OUT*/
} gpioConf_t;

gpioConf_t sistema_riego = {GPIO_20, GPIO_OUTPUT};
gpioConf_t sistema_ventilacion = {GPIO_21, GPIO_OUTPUT};
gpioConf_t sensor_dht11 = {GPIO_22, GPIO_INPUT};

TaskHandle_t sensor_hum_suelo_task_handle = NULL;
TaskHandle_t control_clima_task_handle = NULL;
TaskHandle_t sensor_dht11_task_handle = NULL;
TaskHandle_t bluetooth_task_handle = NULL;

rtc_t horarios_riego[4];
uint8_t t_control = 0;
uint8_t t_pausa = 0;
uint16_t humedad_suelo = 0;
float float_humedad_ambiente = 0;
float float_temperatura_ambiente = 0;
uint8_t humedad_ambiente = 0;
uint8_t temperatura_ambiente = 0;
uint8_t sensacion_termica = 0;
uint8_t hum_suelo_control = 0;
uint8_t temp_amb_control = 0;
rtc_t hora_actual;
bool encendido = false;
bool riego_encendido = false;
bool ventilacion_encendida = false;
bool riego_sin_horario = false;

/*==================[internal functions declaration]=========================*/

/**
 * @brief Función que permite la suma de una cierta cantidad de minutos a una hora en formato rtc_t
 * 
 * @param hora      Hora a la cual se le realiza la suma de los minutos
 * @param minutos   Cantidad de minutos a sumar a la hora pasada como parámetro
 * @return rtc_t    Retorno del resultado de la suma en formato rtc_t
 */
rtc_t sumaHora(rtc_t hora, uint16_t minutos)
{
    rtc_t hora_final;
    uint16_t minutos_final;
    uint16_t h = hora.hour;
    uint16_t m = hora.min;
    uint16_t cant_horas = minutos / 60;
    uint16_t min_restantes = minutos - (60 * cant_horas);
    uint16_t hora_restante;
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

/**
 * @brief Función del Timer A encargada de notificar a las tareas de medición de humedad de suelo,
 * temperatura y humedad ambiente y la tarea encargada del control del clima
 * 
 */
void FuncTimerA(void *param)
{
    vTaskNotifyGiveFromISR(sensor_hum_suelo_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(sensor_dht11_task_handle, pdFALSE);
    vTaskNotifyGiveFromISR(control_clima_task_handle, pdFALSE);
}

/**
 * @brief Función del Timer A encargada de notificar a la tarea del manejo de comunicación Bluetooth
 * 
 * @param param 
 */
void FuncTimerB(void *param)
{
    vTaskNotifyGiveFromISR(bluetooth_task_handle, pdFALSE);
}

/**
 * @brief Tarea encargada de la medición de la humedad del suelo mediante un canal AD y la conversión 
 * a un porcentaje de humedad
 *  
 */
static void medirHumedadSuelo(void *pvParameter)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (encendido)
        {
            uint16_t aux;
            AnalogInputReadSingle(CH3, &aux);
            humedad_suelo = aux;
            if ((-0.269 * humedad_suelo + 171) < 0)
            {
                humedad_suelo = 0;
            }
            else
            {
                humedad_suelo = -0.269 * humedad_suelo + 171;
            }
            if (humedad_suelo > 100)
            {
                humedad_suelo = 100;
            }
        }
    }
}

/**
 * @brief Tarea encargada de la medición de la temperatura y humedad ambiente del
 * sensor DHT11 y el cálculo de la sensación térmica. 
 *  
 */
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
                //Fórmula de cálculo del índice de calor derivado (sensación térmica) de la NWS-NOAA
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

/**
 * @brief Tarea encargada del control del clima dentro del invernadero, mediante la comparación
 * del estado de las variables con los parámetros de control establecidos se determina el encedido o apagado 
 * del sistema de riego y ventilación. A su vez, se realiza el envío de los datos para la recepción mediante
 * la aplicación Bluetooth.
 *  
 */
static void controlClima(void *pvParameter)
{
    uint16_t t_medio = 0;
    uint16_t t_suma = 0;
    rtc_t horario_riego_actual;
    rtc_t aux;
    horario_riego_actual.hour = 33;
    horario_riego_actual.mday = 66;
    uint16_t hora_riego_entero = 0;
    uint16_t hora_tmedio_entero = 0;
    uint16_t hora_tsuma_entero = 0;
    uint16_t hora_tcontrol_entero = 0;
    bool primer_riego = false;
    bool segundo_riego = false;
    bool hora_de_regar = false;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        RtcRead(&hora_actual);
        if (hora_actual.hour == 0)
        {
            hora_actual.hour = 24;
        }
        uint16_t hora_actual_entero = hora_actual.hour * 100 + hora_actual.min;
        t_medio = (t_control - t_pausa) / 2;
        t_suma = t_medio + t_pausa;
        uint8_t i = 0;
        while (i < 4)
        {
            uint16_t hora_riego = horarios_riego[i].hour * 100 + horarios_riego[i].min;
            uint16_t hora_t_control = sumaHora(horarios_riego[i], t_control).hour * 100 + sumaHora(horarios_riego[i], t_control).min;
            if ((hora_actual_entero >= hora_riego) && (hora_actual_entero < hora_t_control))
            {
                hora_de_regar = true;
                horario_riego_actual = horarios_riego[i];
                i = 4;
            }
            else
            {
                i++;
            }
        }
        hora_riego_entero = horario_riego_actual.hour * 100 + horario_riego_actual.min;
        aux = sumaHora(horario_riego_actual, t_medio);
        hora_tmedio_entero = aux.hour * 100 + aux.min;
        aux = sumaHora(horario_riego_actual, t_suma);
        hora_tsuma_entero = aux.hour * 100 + aux.min;
        aux = sumaHora(horario_riego_actual, t_control);
        hora_tcontrol_entero = aux.hour * 100 + aux.min;
        primer_riego = (hora_actual_entero >= hora_riego_entero) && (hora_actual_entero < hora_tmedio_entero);
        segundo_riego = (hora_actual_entero >= hora_tsuma_entero) && (hora_actual_entero < hora_tcontrol_entero);
        if (primer_riego || segundo_riego)
        {
            hora_de_regar = true;
        }
        else
        {
            hora_de_regar = false;
        }

        if (encendido)
        {
            if (hora_de_regar || riego_sin_horario)
            {
                if (humedad_suelo <= hum_suelo_control)
                {
                    GPIOOn(sistema_riego.pin);
                    riego_encendido = true;
                }
                else
                {
                    GPIOOff(sistema_riego.pin);
                    riego_encendido = false;
                }
            }
            else
            {
                GPIOOff(sistema_riego.pin);
                riego_encendido = false;
            }
            if (sensacion_termica >= temp_amb_control)
            {
                GPIOOn(sistema_ventilacion.pin);
                ventilacion_encendida = true;
            }
            else
            {
                GPIOOff(sistema_ventilacion.pin);
                ventilacion_encendida = false;
            }
        }
        else
        {
            GPIOOff(sistema_riego.pin);
            GPIOOff(sistema_ventilacion.pin);
            sensacion_termica = 0;
            humedad_suelo = 0;
            riego_encendido = false;
            ventilacion_encendida = false;
        }
        /*Envío de los datos para visualizarlos en la aplicacion movil*/
        char msg[16];
        if (riego_encendido)
        {
            sprintf(msg, "*RR0G255B0*");
            BleSendString(msg);
        }
        else
        {
            sprintf(msg, "*RR155G155B155*");
            BleSendString(msg);
        }
        if (ventilacion_encendida)
        {
            sprintf(msg, "*VR0G255B0*");
            BleSendString(msg);
        }
        else
        {
            sprintf(msg, "*VR155G155B155*");
            BleSendString(msg);
        }
        sprintf(msg, "*T%d*", sensacion_termica);
        BleSendString(msg);
        sprintf(msg, "*H%d*", humedad_suelo);
        BleSendString(msg);
        sprintf(msg, "*X%d:%d*", hora_actual.hour, hora_actual.min);
        BleSendString(msg);
        sprintf(msg, "*O%d*", t_control);
        BleSendString(msg);
        sprintf(msg, "*P%d*", t_pausa);
        BleSendString(msg);
        sprintf(msg, "*F%d*", temp_amb_control);
        BleSendString(msg);
        sprintf(msg, "*G%d*", hum_suelo_control);
        BleSendString(msg);
    }
}

/**
 * @brief Función a ejecutarse ante un interrupción de recepción 
 * a través de la conexión BLE.
 * 
 * @param data      Puntero a array de datos recibidos
 * @param length    Longitud del array de datos recibidos
 */
void read_data(uint8_t *data, uint8_t length)
{
    uint8_t i = 1;
    uint16_t horario_aux;
    switch (data[0])
    {
    case 'C':
        riego_sin_horario = true;
        break;

    case 'c':
        riego_sin_horario = false;
        break;

    case 'E':
        encendido = true;
        break;

    case 'e':
        encendido = false;
        break;

    case 'O':
        t_control = 0;
        /* El cuadro de texto Tiempo de Control envía los datos con el formato "O" + value + "A" */
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            t_control = t_control * 10;
            t_control = t_control + (data[i] - '0');
            i++;
        }
        break;

    case 'P':
        t_pausa = 0;
        /* El cuadro de texto Tiempo de Pausa envía los datos con el formato "P" + value + "A" */
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            t_pausa = t_pausa * 10;
            t_pausa = t_pausa + (data[i] - '0');
            i++;
        }
        break;

    case 'T':
        temp_amb_control = 0;
        /* El cuadro de texto Temperatura de Control envía los datos con el formato "T" + value + "A" */
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            temp_amb_control = temp_amb_control * 10;
            temp_amb_control = temp_amb_control + (data[i] - '0');
            i++;
        }
        break;

    case 'H':
        hum_suelo_control = 0;
        /* El cuadro de textor Humedad Suelo de Control envía los datos con el formato "H" + value + "A" */
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            hum_suelo_control = hum_suelo_control * 10;
            hum_suelo_control = hum_suelo_control + (data[i] - '0');
            i++;
        }
        break;

    case 'J':
        /* El cuadro de texto Hora de Riego 1 envía los datos con el formato "J" + value + "A" */
        horario_aux = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horario_aux = horario_aux * 10;
            horario_aux = horario_aux + (data[i] - '0');
            i++;
        }
        horarios_riego[0].hour = (uint8_t)(horario_aux / 100);
        horarios_riego[0].min = (uint8_t)(horario_aux % 100);
        break;

    case 'K':
        /* El cuadro de texto Hora de Riego 2 envía los datos con el formato "K" + value + "A" */
        horario_aux = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horario_aux = horario_aux * 10;
            horario_aux = horario_aux + (data[i] - '0');
            i++;
        }
        horarios_riego[1].hour = (uint8_t)(horario_aux / 100);
        horarios_riego[1].min = (uint8_t)(horario_aux % 100);
        break;

    case 'L':
        /* El cuadro de texto Hora de Riego 3 envía los datos con el formato "L" + value + "A" */
        horario_aux = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horario_aux = horario_aux * 10;
            horario_aux = horario_aux + (data[i] - '0');
            i++;
        }
        horarios_riego[2].hour = (uint8_t)(horario_aux / 100);
        horarios_riego[2].min = (uint8_t)(horario_aux % 100);
        break;

    case 'M':
        /* El cuadro de texto Hora de Riego 1 envía los datos con el formato "M" + value + "A" */
        horario_aux = 0;
        while (data[i] != 'A')
        {
            /* Convertir el valor ASCII a un valor entero */
            horario_aux = horario_aux * 10;
            horario_aux = horario_aux + (data[i] - '0');
            i++;
        }
        horarios_riego[3].hour = (uint8_t)(horario_aux / 100);
        horarios_riego[3].min = (uint8_t)(horario_aux % 100);
        break;
    default:
        break;
    }
}


/**
 * @brief Tarea encargada del control del estado de la comunicación Bluetooth
 *  
 */
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

/**
 * @brief Aplicación principal donde se inicializan los periféricos, timers, comunicación Bluetooth,
 * y se realiza la creación de las distintas tareas.
 * 
 */
void app_main(void)
{
    ble_config_t ble_configuration = {
        "Control y Monitoreo de Clima",
        read_data};

    LedsInit();
    BleInit(&ble_configuration);

    GPIOInit(sistema_riego.pin, sistema_riego.dir);
    GPIOInit(sistema_ventilacion.pin, sistema_ventilacion.dir);
    GPIOInit(sensor_dht11.pin, sensor_dht11.dir);

    dht11Init(sensor_dht11.pin);

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
        .input = CH3,
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
        .hour = 10,
        .min = 32,
        .sec = 0};
    RtcConfig(&config_time);

    xTaskCreate(&medirHumTempAmbiente, "sensorHumTemAmbiente", 2048, NULL, 5, &sensor_dht11_task_handle);
    xTaskCreate(&medirHumedadSuelo, "sensorHumSuelo", 2048, NULL, 5, &sensor_hum_suelo_task_handle);
    xTaskCreate(&controlClima, "controlClima", 4096, NULL, 5, &control_clima_task_handle);
    xTaskCreate(&bluetoothTask, "BLUETOOTH", 4096, NULL, 5, &bluetooth_task_handle);

    TimerStart(timer_1.timer);
    TimerStart(timer_2.timer);
}
/*==================[end of file]============================================*/